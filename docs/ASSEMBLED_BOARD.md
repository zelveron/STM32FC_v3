# Current BMI270 card — 2026-10-08

The user identified the Raspberry Pi-connected controller as the dual-BMI270
assembly and confirmed USB-only power. This document records software and USB
bench observations. Earlier assembly results are retained in Git history and
must not be treated as validation of this card.

## Firmware configuration

Use `pio run` or `pio run -e v2_bmi270`. BMI270 is the only IMU implementation.
The default profile disables magnetometer polling (`FC_MAG_ENABLED=0`) and
inhibits both motors (`FC_FLIGHT_ENABLED=0`). The optional `v2` profile also uses
BMI270, with BMM350 support enabled. Sensor transforms, control selection,
failsafe behavior and motor gates have not been changed by this update.

Each BMI270 has its own Bosch device state and SPI bus. Initialization uses
1 MHz SPI and the vendored Bosch API, including SPI selection/reset, the full
8192-byte configuration upload and INTERNAL_STATUS verification. After setting
400 Hz accel/gyro, +/-8 g, +/-2000 deg/s and paired filtered headerless FIFO,
measurement is enabled and allowed to settle for 80 ms. The driver then switches
to 5 MHz and flushes startup frames before runtime validation starts.

The settling interval fixes a demonstrated software fault: startup dummy FIFO
frames previously caused the strict paired-frame parser to latch IMU 1 as
failed. The fix restored its readings, calibration and attitude output. It does
not bypass runtime identity, error-register, FIFO, sensor-clock, clipping,
freshness or dual-sensor disagreement checks. See the [Bosch BMI270 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)
and the pinned vendor provenance in `lib/bmi270/UPSTREAM.md`.

## Observed USB bench results

The motor-inhibited image was programmed through ROM DFU. Every programmed
payload byte was read back and compared; the original 1 MiB flash was saved
locally before updating. USB returned as `0483:5740`, `/dev/ttyACM0` on this Pi.

| Component | Observation | What it establishes |
|---|---|---|
| BMI270 #1 / SPI1 | `error0=0`, `regs0=240100`, healthy=1, calibration ready; changing accel/gyro/attitude | Initialization and runtime samples are working |
| BMI270 #2 / SPI2 | `error1=103`, `regs1=0`, healthy=0 | ID read returned 0x00 instead of expected 0x24; failure occurs before configuration upload |
| BMP581 | `valid=0,error=2` | The initial reset-register write failed; reliable communication was not established |
| GNSS | Recent UART bytes and checksum-valid NMEA at 9600 baud; fix=0, used=0; GSV visibility varied | Receiver-to-STM32 communication works; no position fix was observed |
| Motor gate | `armed=0,flight_enabled=0` | Both motors remain inhibited in this build |

The IMU 2 and barometer observations do not identify a physical cause. Lowering
BMI270 initialization speed to 1 MHz did not change IMU 2's ID result. The BMP
HAL error does not distinguish NACK from timeout. RC operation, GNSS fix,
real-card logging, actuator directions and flight behavior remain unqualified.

## Telemetry and GUI

`IMU_CONFIG` reports the model, per-device error codes and hexadecimal register
snapshots: `CHIP_ID << 16 | INTERNAL_STATUS << 8 | ERR_REG`; normal is `240100`.
Bosch initialization failures use `100 - return_code`: 103 is unexpected chip
identity, and 109 is configuration-load failure. Driver stages 1–12 are listed
in README.md and decoded by the GUI. `IMU_HEALTH` reports both sensor health
flags, the selected IMU, disagreement and estimator eligibility. The displayed
accel/gyro values are from the selected IMU.

`GPS_HEALTH` is emitted continuously, including without a fix and when a GUI
connects late. Fields are recent UART bytes (`rx`), checksum-valid NMEA (`nmea`),
fix, GGA satellites used (`used`, -1 unknown), GSV visibility (`visible`: -1
unknown, 0 none reported, 1 in view), baud, and cumulative bytes/messages.
Positive GSV evidence lasts five seconds so an empty report for a different
constellation cannot immediately erase it. Missing GNSS data, malformed traffic,
no satellites, no fix and stale controller telemetry are distinct GUI states.

The Windows x64 GUI includes its Python/Tk/serial runtime and DFU utility.
See [Windows packaging](GUI_WINDOWS.md) and [DFU operation](DFU.md).

## Verification

On October 8, both ARM profiles (`v2_bmi270` and `v2`) compiled successfully.
The current-card image is byte-for-byte identical to the readback-verified
image already flashed: SHA256 (including DFU suffix)
`4b4da0d0ad9ff2b225bf1bbd0bebf42505a5731e14190ef5d6c533270209bc1a`.
The complete GCC host suite passed **299 C/C++ checks**, plus **28 GUI parser
checks, 2 serial-connection tests and 7 DFU helper tests**. Real Tk widget and
system DFU-tool smoke tests also passed on the Raspberry Pi. Windows package
verification is recorded separately in the release's workflow and smoke report.


The BMI270 driver model runs the real Bosch API, checks SPI/reset/power-save
settling, verifies all configuration bytes and their word-addressed upload,
models startup dummy frames, and exercises two independent buses, wrong chip
identity, configuration failure, runtime FIFO faults and IMU selection.

Firmware host tests cover the control/estimation modules, regressions, bench
motor inhibition, SD fault handling, BMI270/BMP581 models and flight scenarios.
GUI tests cover parsing, stale data, individual sensor diagnostics, GNSS status,
serial ownership and DFU discovery. A read-only real-widget smoke test checks
Tk, serial imports and the bundled DFU program without opening the board's port.
These checks are software evidence, not flight qualification.
