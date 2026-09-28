"""Recover the committed prefix of a preallocated FCS2 log, or read legacy STFC."""
import binascii
import struct

IMU_FMT = '<HIII6h6hBBH'
CONTROL_FMT = '<HI2h3h3hHBBH'
RECORDS = {0x5AA5: 86, 0xA16D: struct.calcsize(IMU_FMT), 0xC17D: struct.calcsize(CONTROL_FMT)}

def crc16(data):
    return binascii.crc_hqx(data, 0xffff)

def chunks(path, stats):
    with open(path, 'rb') as f:
        first = f.read(512)
        if first[:4] != b'FCS2':
            if first[:4] != b'STFC':
                raise ValueError('No valid STM32FC file header; the first sector may not have committed')
            stats['container'] = 'legacy'
            yield first
            while block := f.read(65536):
                yield block
            return
        stats.update(container='FCS2', sectors=0, stop='end-of-file')
        session = None
        block = first
        sequence = 0
        while block:
            if len(block) != 512:
                stats['stop'] = 'partial-sector'; break
            tag, nonce, seq, used = struct.unpack_from('<4sQIH', block)
            if tag != b'FCS2' or used > 492 or struct.unpack_from('<H', block, 510)[0] != crc16(block[:510]):
                stats['stop'] = 'unwritten-or-corrupt-sector'; break
            if session is None:
                session = nonce
                stats['session'] = f'{session:016x}'
            if not session or nonce != session or seq != sequence:
                stats['stop'] = 'session-or-sequence-mismatch'; break
            stats['sectors'] += 1
            yield block[18:18+used]
            sequence += 1
            block = f.read(512)

def read_payload(path):
    stats = {}
    return b''.join(chunks(path, stats)), stats

def records(path, stats):
    """Bounded-memory iterator; yields (magic, raw), skipping damaged records."""
    pending = bytearray()
    header = False
    for chunk in chunks(path, stats):
        pending.extend(chunk)
        if not header:
            if len(pending) < 20:
                continue
            if pending[:4] != b'STFC':
                raise ValueError('Bad stream header inside sector envelope')
            del pending[:20]
            header = True
        i = 0
        while i+2 <= len(pending):
            magic = struct.unpack_from('<H', pending, i)[0]
            size = RECORDS.get(magic)
            if size is None:
                i += 1; continue
            if len(pending)-i < size:
                break
            raw = bytes(pending[i:i+size])
            if struct.unpack_from('<H', raw, size-2)[0] != crc16(raw[:-2]):
                stats['bad_records'] = stats.get('bad_records', 0)+1
                i += 1; continue
            yield magic, raw
            i += size
        del pending[:i]
    stats['trailing_bytes'] = len(pending)
