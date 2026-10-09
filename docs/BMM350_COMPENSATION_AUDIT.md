# Checking BMM350 factory compensation

This check separates repeatable factory-calibration reads, coefficient decoding
and arithmetic from installation calibration or magnetic-field accuracy.
It never estimates offsets from the observed large readings and never changes
the heading acceptance limits.

## What the firmware records

Before starting the scheduler, the driver runs the unmodified Bosch initialization
three times. Each initialization reads all 32 factory OTP words and performs the
normal reset sequence. The third pass supplies the active coefficients. Any
transport/status failure or differing word blocks acquisition at startup.
Only read and power-off OTP commands are used; no OTP programming is performed.

Saved snapshots are emitted over USB for late connections:

- `MAG_OTP_STATUS,passes=3,mismatch=0`: three completed matching acquisitions.
  `mismatch` is a decimal bit mask, with bit N identifying OTP word N.
- `MAG_OTP,pass=P,offset=N,words=...`: eight hexadecimal 16-bit words from pass
  0/1/2, starting at word 0/8/16/24.
- `MAG_TRIM,offset=N,bits=...`: up to eight hexadecimal IEEE-754 float32 bit
  patterns, starting at coefficient 0/8/16.
- `MAG_DATA`: the raw signed 24-bit counts and corresponding compensated
  measurements from the same burst, including rejected measurements.

The 19 coefficients are, in order: temperature offset, X/Y/Z offsets,
temperature sensitivity, X/Y/Z sensitivities, X/Y/Z temperature-offset
coefficients, X/Y/Z temperature-sensitivity coefficients, reference temperature,
and cross-axis coefficients XY/YX/ZX/ZY. Bit patterns avoid decimal precision
loss in the coefficient comparison.

One snapshot chunk is emitted per status interval, cycling through 15 chunks.
Runtime telemetry reads saved RAM only, not OTP. The runtime sensor polling,
read-recovery behavior, motor gates and flight controls remain unchanged.

## Run the check

1. With this firmware installed, record USB for at least 30 seconds using the
   GUI's **Record USB** control. Keep one boot per capture. Rotation is not
   needed for a compensation arithmetic audit.
2. On the Pi, run:

   ```sh
   python3 tools/check_bmm350_compensation.py /path/to/capture.txt --output build/bmm-audit.json
   ```

3. To recompute an earlier raw-data CSV from the **same board**, use:

   ```sh
   python3 tools/check_bmm350_compensation.py /path/to/capture.txt --samples-csv /path/to/readings.csv --output build/bmm-earlier-audit.json
   ```

The tool requires three matching complete OTP images, rejects all-zero/all-ones
images, independently decodes their signed bit fields, and compares the resulting
float32 coefficients exactly against the board's saved coefficients. Python
double-precision arithmetic then computes raw scaling, temperature correction,
gain/offset/thermal compensation and cross-axis correction. It compares those
results with the board output, allowing 0.02 µT/°C for two-decimal telemetry and
float32 arithmetic. It exits nonzero for missing/inconsistent snapshots or a
calculation mismatch. The JSON retains per-stage values and maximum differences.

Agreement means the observed output is consistent with the captured OTP and
raw data. It cannot prove that the factory data are intrinsically correct, that
every I²C byte is correct despite matching repeated reads, or that a measured
field is physically accurate. Matching calculations do not make out-of-range
data suitable for heading.

## Software verification

`python3 tests/bmm_compensation.py` uses GCC on Linux to build a small test
wrapper around the exact unmodified vendor C file, then compares independent
Python decoding/arithmetic against 1,000 deterministic generated fixtures.
It also checks incomplete/conflicting snapshots and intentional output errors.
Driver tests cover repeatable OTP copies and a changed word in the middle pass.
Run these alongside `python3 tools/test_host.py` and the ARM builds.

Reference: [pinned Bosch SensorAPI](https://github.com/boschsensortec/BMM350_SensorAPI/blob/3daf377ccaf589319c0d41af192105e9275987a2/bmm350.c).

## Connected-board result — 2026-10-09

On USB serial `3154365B3034`, all 32 OTP words matched across three complete
boot acquisitions. No I²C or OTP-status errors were reported. All 19 independently
decoded coefficient float32 bit patterns matched the firmware exactly.

The factory offset coefficients are **X = −192, Y = −126, Z = +49 µT**.
They are applied together with sensitivity, temperature and cross-axis terms;
they are not installation hard-iron calibration values.

The independent calculation matched 27 new stationary measurements and all
100 readings from the user's earlier rotation capture. Across the 100 rotation
readings, maximum absolute differences were 0.005818 µT on X, 0.005417 µT on Y,
0.004987 µT on Z and 0.004999 °C on temperature, consistent with float32
arithmetic and two-decimal output rounding. No sample exceeded the 0.02 tolerance.

For the first recorded rotation sample:

| Stage | X (µT) | Y (µT) | Z (µT) |
|---|---:|---:|---:|
| Raw counts scaled, before factory compensation | −3620.9815 | −3144.3730 | −15.6773 |
| Independent full compensation | −3892.3439 | −3344.0989 | −48.2616 |
| Actual firmware output | −3892.34 | −3344.10 | −48.26 |

Thus the large X/Y baseline already exists in raw data before factory
compensation. The captured conversion arithmetic is consistent with the official
Bosch equations and the captured OTP. The physical cause of the baseline remains
unestablished; magnetic measurements remain out of range and heading remains
unavailable. No fitted offsets or altered acceptance limits were installed.

The motor-enabled **`v2_flight`** image was programmed and all 150,220 payload
bytes matched DFU readback. Image including its 16-byte suffix: 150,236 bytes,
SHA-256 `b5ce78e7965857867d2e1f7cf74d57b5476077c26bb8becd0d973e316dddf06c`.
Payload/readback SHA-256:
`57d30b03b8f44065dd0ddeb44ea9dc42cbd6605c8bbf047c526c2c6710a971d2`.

Both BMI270s and BMP581 remained healthy. Runtime reported
`flight_enabled=1,armed=0,failsafe=1,timing_fault=0`. The updated source GUI
was reopened after capture. No motor/flight test was performed.

Validation: 388 C/C++ host checks, both ARM profiles (`v2`, `v2_flight`), and
six compensation-audit tests including 1,000 generated comparisons passed.
The previous full 1 MiB flash backup, new image/readback, USB capture and JSON
reports are retained locally under `build/deploy-bmm-otp-20261009/` (ignored by
Git). The 100-reading rotation capture remains under
`build/mag-rotation-20261009-143003/`.
