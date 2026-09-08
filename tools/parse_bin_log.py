#!/usr/bin/env python3
"""
parse_bin_log.py -- decode an STM32FC FLTxxxxx.BIN flight log.

Layout must match src/core/log_frame.hpp: a 20-byte LogFileHeader, then packed
86-byte LogFrames, each magic 0x5AA5 + CRC-16/CCITT over the first 84 bytes.

CLI:
    python3 tools/parse_bin_log.py FLT00007.BIN            # -> FLT00007.csv
    python3 tools/parse_bin_log.py FLT00007.BIN -o out.csv
    python3 tools/parse_bin_log.py FLT00007.BIN --summary  # stats only

Library (used by tools/plot_log.py):
    from parse_bin_log import decode, write_csv, FIELDS
    meta, rows = decode("FLT00007.BIN")   # rows: list of tuples, one per frame
"""
import argparse
import struct
import sys

HDR_FMT    = "<4sBBHfff"
HDR_SIZE   = struct.calcsize(HDR_FMT)          # 20
FRAME_FMT  = "<HI3h3h3hfhiiiiHBB8H8HBBH"
FRAME_SIZE = struct.calcsize(FRAME_FMT)        # 86
MAGIC      = 0x5AA5

# Column names, in row order. Index 0 is the timestamp; the rest are channels.
FIELDS = ("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,yaw_deg,"
          "press_pa,temp_c,alt_m,lat,lon,gps_alt_m,gps_speed_ms,gps_sats,gps_fix,"
          "rc1,rc2,rc3,rc4,rc5,rc6,rc7,rc8,"
          "out1,out2,out3,out4,out5,out6,out7,out8,"
          "mode,armed,failsafe,req_mode").split(",")

COLS = ",".join(FIELDS)   # back-compat


def crc16_ccitt(buf):
    crc = 0xFFFF
    for byte in buf:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def decode(path):
    """Decode a .BIN file.

    Returns (meta, rows):
      meta -- dict: version, frame_size, acc_scale, gyr_scale, ang_scale,
              n_ok, n_bad_crc, n_resync, duration_s, hz
      rows -- list of tuples, each matching FIELDS (physical units).
    Raises ValueError on a bad/incompatible header.
    """
    with open(path, "rb") as fp:
        data = fp.read()
    if len(data) < HDR_SIZE:
        raise ValueError("file shorter than the 20-byte header")

    tag, ver, fsize, magic, accs, gyrs, angs = struct.unpack(HDR_FMT, data[:HDR_SIZE])
    if tag != b"STFC" or magic != MAGIC:
        raise ValueError(f"bad header (tag={tag!r} magic={magic:#06x}) -- not an STM32FC log")
    if fsize != FRAME_SIZE:
        raise ValueError(f"frame size {fsize} != parser's {FRAME_SIZE} -- log_frame.hpp version skew")

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
        (_mg, t, ax, ay, az, gx, gy, gz, rr, pp, yy, press, temp_dC, alt_mm,
         lat, lon, galt, gspd, gsat, gfix, *rest) = f
        rc  = rest[0:8]
        out = rest[8:16]
        mode, flags = rest[16], rest[17]
        rows.append((t,
                     ax / accs, ay / accs, az / accs,
                     gx / gyrs, gy / gyrs, gz / gyrs,
                     rr / angs, pp / angs, yy / angs,
                     press, temp_dC / 10.0, alt_mm / 1000.0,
                     lat / 1e7, lon / 1e7, galt / 1000.0, gspd / 100.0, gsat, gfix,
                     *rc, *out,
                     mode, flags & 1, (flags >> 1) & 1, (flags >> 2) & 3))
        ok += 1
        i += FRAME_SIZE

    dur = (rows[-1][0] - rows[0][0]) / 1000.0 if len(rows) > 1 else 0.0
    meta = dict(version=ver, frame_size=fsize,
                acc_scale=accs, gyr_scale=gyrs, ang_scale=angs,
                n_ok=ok, n_bad_crc=bad_crc, n_resync=resyncs,
                duration_s=dur, hz=(ok / dur if dur else 0.0))
    return meta, rows


def write_csv(rows, outpath):
    """Write decoded rows to a CSV with a header line. lat/lon get extra precision."""
    hi_prec = {i for i, n in enumerate(FIELDS) if n in ("lat", "lon")}
    with open(outpath, "w") as fp:
        fp.write(COLS + "\n")
        for r in rows:
            fp.write(",".join(
                (f"{v:.9g}" if i in hi_prec else
                 f"{v:.6g}" if isinstance(v, float) else str(v))
                for i, v in enumerate(r)) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("infile")
    ap.add_argument("-o", "--outfile")
    ap.add_argument("--summary", action="store_true", help="print stats, no CSV")
    args = ap.parse_args()

    try:
        meta, rows = decode(args.infile)
    except (OSError, ValueError) as e:
        sys.exit(str(e))

    print(f"header: version={meta['version']} frame={meta['frame_size']}B "
          f"acc/{meta['acc_scale']:g} gyr/{meta['gyr_scale']:g} ang/{meta['ang_scale']:g}",
          file=sys.stderr)
    if meta["duration_s"]:
        print(f"frames: {meta['n_ok']} ok, {meta['n_bad_crc']} bad-crc, "
              f"{meta['n_resync']} resync bytes; "
              f"{meta['duration_s']:.1f} s, {meta['hz']:.0f} Hz", file=sys.stderr)
    else:
        print(f"frames: {meta['n_ok']} ok", file=sys.stderr)

    if args.summary or not rows:
        return

    outpath = args.outfile or args.infile.rsplit(".", 1)[0] + ".csv"
    write_csv(rows, outpath)
    print(f"wrote {outpath}", file=sys.stderr)


if __name__ == "__main__":
    main()
