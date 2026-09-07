#!/usr/bin/env python3
"""
parse_bin_log.py -- decode an STM32FC FLTxxxxx.BIN flight log to CSV.

Layout must match src/core/log_frame.hpp. 20-byte LogFileHeader, then packed
86-byte LogFrames, each magic 0x5AA5 + CRC-16/CCITT over the first 84 bytes.

    python3 tools/parse_bin_log.py FLT00007.BIN            # -> FLT00007.csv
    python3 tools/parse_bin_log.py FLT00007.BIN -o out.csv
    python3 tools/parse_bin_log.py FLT00007.BIN --summary  # stats only
"""
import argparse
import struct
import sys

HDR_FMT   = "<4sBBHfff"
HDR_SIZE  = struct.calcsize(HDR_FMT)          # 20
FRAME_FMT = "<HI3h3h3hfhiiiiHBB8H8HBBH"
FRAME_SIZE = struct.calcsize(FRAME_FMT)       # 86
MAGIC = 0x5AA5

COLS = ("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,yaw_deg,"
        "press_pa,temp_c,alt_m,lat,lon,gps_alt_m,gps_speed_ms,gps_sats,gps_fix,"
        "rc1,rc2,rc3,rc4,rc5,rc6,rc7,rc8,"
        "out1,out2,out3,out4,out5,out6,out7,out8,"
        "mode,armed,failsafe,req_mode")


def crc16_ccitt(buf):
    crc = 0xFFFF
    for byte in buf:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("infile")
    ap.add_argument("-o", "--outfile")
    ap.add_argument("--summary", action="store_true", help="print stats, no CSV")
    args = ap.parse_args()

    data = open(args.infile, "rb").read()
    if len(data) < HDR_SIZE:
        sys.exit("file shorter than the header")

    tag, ver, fsize, magic, accs, gyrs, angs = struct.unpack(HDR_FMT, data[:HDR_SIZE])
    if tag != b"STFC" or magic != MAGIC:
        sys.exit(f"bad header (tag={tag!r} magic={magic:#06x})")
    if fsize != FRAME_SIZE:
        sys.exit(f"frame size {fsize} != parser's {FRAME_SIZE} -- version skew")
    print(f"header: version={ver} frame={fsize}B "
          f"acc/{accs:g} gyr/{gyrs:g} ang/{angs:g}", file=sys.stderr)

    body = data[HDR_SIZE:]
    rows, ok, bad_crc, resyncs = [], 0, 0, 0
    i = 0
    while i + FRAME_SIZE <= len(body):
        if body[i] | (body[i + 1] << 8) != MAGIC:
            i += 1
            resyncs += 1
            continue
        raw = body[i:i + FRAME_SIZE]
        f = struct.unpack(FRAME_FMT, raw)
        if f[-1] != crc16_ccitt(raw[:FRAME_SIZE - 2]):
            bad_crc += 1
            i += 1
            continue
        (mg, t, ax, ay, az, gx, gy, gz, rr, pp, yy, press, temp_dC, alt_mm,
         lat, lon, galt, gspd, gsat, gfix, *rest) = f
        rc  = rest[0:8]
        out = rest[8:16]
        mode, flags, _crc = rest[16], rest[17], rest[18]
        rows.append((t, ax/accs, ay/accs, az/accs, gx/gyrs, gy/gyrs, gz/gyrs,
                     rr/angs, pp/angs, yy/angs, press, temp_dC/10.0, alt_mm/1000.0,
                     lat/1e7, lon/1e7, galt/1000.0, gspd/100.0, gsat, gfix,
                     *rc, *out, mode, flags & 1, (flags >> 1) & 1, (flags >> 2) & 3))
        ok += 1
        i += FRAME_SIZE

    dur = (rows[-1][0] - rows[0][0]) / 1000.0 if len(rows) > 1 else 0.0
    print(f"frames: {ok} ok, {bad_crc} bad-crc, {resyncs} resync bytes; "
          f"{dur:.1f} s, {ok/dur:.0f} Hz" if dur else f"frames: {ok} ok",
          file=sys.stderr)
    if args.summary or not rows:
        return

    outpath = args.outfile or args.infile.rsplit(".", 1)[0] + ".csv"
    with open(outpath, "w") as fp:
        fp.write(COLS + "\n")
        for r in rows:
            fp.write(",".join(f"{v:.6g}" if isinstance(v, float) else str(v)
                              for v in r) + "\n")
    print(f"wrote {outpath}", file=sys.stderr)


if __name__ == "__main__":
    main()
