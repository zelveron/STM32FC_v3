#!/usr/bin/env python3
"""Audit saved BMM350 OTP/trim/raw telemetry without connecting to hardware.

Independent Python arithmetic for the equations/packing in Bosch SensorAPI
3daf377ccaf589319c0d41af192105e9275987a2 (lib/bmm350). No calibration is fitted
or installed. Agreement validates conversion consistency, not field accuracy.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import struct

TRIM_NAMES = ('t_off', 'x_off', 'y_off', 'z_off', 't_sens', 'x_sens', 'y_sens',
              'z_sens', 'x_tco', 'y_tco', 'z_tco', 'x_tcs', 'y_tcs', 'z_tcs',
              't0', 'cxy', 'cyx', 'czx', 'czy')


def signed(value, bits):
    value &= (1 << bits) - 1
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def float_bits(value):
    return struct.unpack('<I', struct.pack('<f', value))[0]


def decode_otp(words):
    if len(words) != 32 or any(not 0 <= w <= 65535 for w in words):
        raise ValueError('Expected 32 unsigned 16-bit OTP words')
    w = words
    c = {'t_off': signed(w[13], 8) / 5,
         'x_off': signed(w[14], 12),
         'y_off': signed(((w[14] >> 12) << 8) | (w[15] & 255), 12),
         'z_off': signed((w[15] & 0xF00) | (w[16] & 255), 12),
         't_sens': signed(w[13] >> 8, 8) / 512,
         'x_sens': signed(w[16] >> 8, 8) / 256,
         'y_sens': signed(w[17], 8) / 256,
         'z_sens': signed(w[17] >> 8, 8) / 256,
         't0': signed(w[24], 16) / 512 + 23,
         'cxy': signed(w[21], 8) / 800,
         'cyx': signed(w[21] >> 8, 8) / 800,
         'czx': signed(w[22], 8) / 800,
         'czy': signed(w[22] >> 8, 8) / 800}
    for axis, idx in zip('xyz', (18, 19, 20)):
        c[axis + '_tco'] = signed(w[idx], 8) / 32
        c[axis + '_tcs'] = signed(w[idx] >> 8, 8) / 16384
    return c


def compensate(raw, c):
    if len(raw) != 4 or any(not -8388608 <= r <= 8388607 for r in raw):
        raise ValueError('Expected four signed 24-bit raw counts')
    adc_gain, lut_gain = 1 / 1.5, 0.714607238769531
    xy_scale = (1e6 / 1048576) / (14.55 * 19.46 * adc_gain * lut_gain)
    z_scale = (1e6 / 1048576) / (9 * 31 * adc_gain * lut_gain)
    t_scale = 1 / (0.00204 * adc_gain * lut_gain * 1048576)
    untrimmed = [raw[0] * xy_scale, raw[1] * xy_scale, raw[2] * z_scale]
    temperature = (raw[3] * t_scale - 25.49) * (1 + c['t_sens']) + c['t_off']
    dt = temperature - c['t0']
    pre_cross = [(v * (1 + c[a + '_sens']) + c[a + '_off'] + c[a + '_tco'] * dt)
                 / (1 + c[a + '_tcs'] * dt) for a, v in zip('xyz', untrimmed)]
    # Solve the coupled X/Y equations, then remove X/Y contributions to Z.
    denominator = 1 - c['cxy'] * c['cyx']
    x = (pre_cross[0] - c['cxy'] * pre_cross[1]) / denominator
    y = (pre_cross[1] - c['cyx'] * pre_cross[0]) / denominator
    z = pre_cross[2] - c['czx'] * x - c['czy'] * y
    result = [x, y, z, temperature]
    if not all(math.isfinite(v) for v in result):
        raise ValueError('Nonfinite compensated result')
    return {'raw_scaled_uT': untrimmed, 'before_cross_axis_uT': pre_cross,
            'compensated': dict(zip(('x', 'y', 'z', 'temp'), result))}


def measurement(fields):
    row = {k: float(fields[k]) for k in ('x', 'y', 'z', 'temp')}
    if not all(math.isfinite(v) for v in row.values()):
        raise ValueError('Nonfinite board measurement')
    row['raw'] = [int(v) for v in fields['raw'].split('/')]
    return row


def read_capture(path):
    otp = {}; trim = {}; status = []; samples = []
    def insert(mapping, key, value):
        if key in mapping and mapping[key] != value:
            raise ValueError('Diagnostic snapshot changed within capture; use one boot only')
        mapping[key] = value
    for line in Path(path).read_text().splitlines():
        parts = line.strip().split(','); tag = parts[0]
        if tag not in ('MAG_OTP', 'MAG_TRIM', 'MAG_OTP_STATUS', 'MAG_DATA'):
            continue
        fields = dict(p.split('=', 1) for p in parts[1:])
        if tag == 'MAG_OTP':
            p, start = int(fields['pass']), int(fields['offset'])
            values = [int(v, 16) for v in fields['words'].split('/')]
            if p not in range(3) or start not in (0, 8, 16, 24) or len(values) != 8:
                raise ValueError('Malformed OTP chunk')
            for i, v in enumerate(values):
                if not 0 <= v <= 65535: raise ValueError('Invalid OTP word')
                insert(otp, (p, start + i), v)
        elif tag == 'MAG_TRIM':
            start = int(fields['offset']); values = [int(v, 16) for v in fields['bits'].split('/')]
            if start not in (0, 8, 16) or len(values) != min(8, 19-start):
                raise ValueError('Malformed trim chunk')
            for i, v in enumerate(values):
                if not 0 <= v <= 0xFFFFFFFF: raise ValueError('Invalid float bits')
                insert(trim, start + i, v)
        elif tag == 'MAG_OTP_STATUS':
            status.append((int(fields['passes']), int(fields['mismatch'])))
        else:
            samples.append(measurement(fields))
    if len(otp) != 96 or len(trim) != 19 or not status:
        raise ValueError('Incomplete OTP/trim snapshot; capture at least one full 15-chunk cycle')
    if any(s != (3, 0) for s in status):
        raise ValueError('Firmware did not verify three matching complete OTP reads')
    passes = [[otp[(p, w)] for w in range(32)] for p in range(3)]
    if not passes[0] == passes[1] == passes[2]:
        raise ValueError('OTP words differ across boot reads')
    if all(v == 0 for v in passes[0]) or all(v == 65535 for v in passes[0]):
        raise ValueError('Uniform empty OTP image; matching reads alone are insufficient')
    return passes, trim, samples


def audit(path, samples_csv=None):
    passes, trim, samples = read_capture(path)
    if samples_csv:
        with Path(samples_csv).open(newline='') as f:
            samples = [measurement(r) for r in csv.DictReader(f)]
    if not samples: raise ValueError('No MAG_DATA samples to compare')
    coeff = decode_otp(passes[2])
    differences = [name for i, name in enumerate(TRIM_NAMES) if float_bits(coeff[name]) != trim[i]]
    max_error = {k: 0.0 for k in ('x', 'y', 'z', 'temp')}; failures = 0
    first = None
    for row in samples:
        stages = compensate(row['raw'], coeff)
        errors = {k: abs(stages['compensated'][k] - row[k]) for k in max_error}
        # Board output has two decimals; allow rounding and float32 arithmetic.
        failures += any(e > 0.02 for e in errors.values())
        max_error = {k: max(max_error[k], errors[k]) for k in max_error}
        if first is None: first = {'raw': row['raw'], 'board': {k: row[k] for k in max_error}, **stages}
    return {'calculation_matches': not differences and not failures,
            'otp_passes': 3, 'otp_identical': True,
            'otp_words_hex': [f'{w:04X}' for w in passes[0]],
            'coefficients': coeff, 'coefficient_mismatches': differences,
            'samples': len(samples), 'samples_outside_tolerance': failures,
            'max_absolute_error': max_error, 'first_sample_stages': first,
            'scope': 'Conversion consistency only; does not establish sensor accuracy or valid heading.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--samples-csv', type=Path, help='Optional raw samples from the same board to recompute')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        result = audit(args.capture, args.samples_csv)
    except (ValueError, KeyError, ZeroDivisionError) as exc:
        result = {'calculation_matches': False, 'error': str(exc)}
    encoded = json.dumps(result, indent=2, allow_nan=False) + '\n'
    if args.output: args.output.write_text(encoded)
    print(encoded, end='')
    return 0 if result['calculation_matches'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
