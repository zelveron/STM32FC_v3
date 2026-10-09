# Current BMI270 card — 2026-10-08

**SD update, 2026-10-09:** the first-sector DMA logging failure is repaired.
Recording and independent card readback recovered both IMU streams, controller
records and flight state with valid sector/record CRCs. The full motor-enabled
flight firmware was restored and verified afterward, still disarmed on USB
power. See [the repair and bench evidence](SD_FIX_2026-10-09.md). Earlier
inactive-SD observations below describe their original captures.

**Later deployment, 2026-10-09:** [BMM350 transport was repaired and the
motor-enabled flight firmware was flashed and verified](BMM350_FIX_2026-10-09.md). That record includes current
IMU/BMP health and BMM350 acquisition/calibration limitations. The dated
observations below remain historical.

The user identified the Raspberry Pi-connected controller as the dual-BMI270
assembly and confirmed USB-only power. This document records software and USB
bench observations. Earlier assembly results are retained in Git history and
must not be treated as validation of this card.

## Firmware configuration

### Current AETR6 aircraft mapping, final update on 2026-10-08

The user selected SERVO1 aileron, SERVO2 elevator, SERVO3 throttle, SERVO4
rudder and SERVO6 reversed aileron. The earlier sections below describe the
previous output layout. Physical GPIO/timer routing is unchanged; the mixer,
motor-output classification, startup pulses, stabilization transitions,
controller logging, simulator and GUI labels now use the new layout.
SERVO5 stays centered; SERVO8/9 remain at 1000 us.

The updated `v2_flight` image was programmed with **USB power only**, after
the user removed battery/BEC/receiver power. All **142,424 payload bytes**
matched flash readback; the prior complete 1 MiB flash was backed up.
Image SHA256 including DFU suffix:
`9cfeaa562712a515d48f20d3b060ebc4cc0975014f068817a5084125905b266f`.

Over 16 seconds / 320 `OUT` reports, SERVO3 stayed at 1000 us, SERVO5 stayed
at 1500 us, SERVO8/9 stayed at 1000 us, and SERVO1 + SERVO6 always equaled
3000 us. One report was `OUT,1244,1303,1000,1512,1500,1756,1000,1000`.
The application reported `flight_enabled=1,armed=0,failsafe=1,timing_fault=0`.
RC loss is expected with the receiver unpowered; healthy-sensor failsafe
continues surface leveling. Both BMI270s and BMP581 were healthy and gyro
calibration completed. BMM350 remains unresolved; SD logging was inactive.

Both ARM profiles (`v2`, `v2_flight`), the complete host suite including
startup/arming/failsafe/automatic recovery/reversed-surface regressions,
and 33 GUI tests passed. The GUI EXE was rebuilt as `2026.10.08-aetr6`.
Electrical pulse measurements and physical servo motion were not tested
with BEC power removed. Evidence and backup are retained beside the checkout
in `build-tools/aetr6-flash-2026-10-08/`.

### Later Windows BMM350 check on 2026-10-08

After the user connected the BMM350, the board enumerated as `0483:5740`,
COM6. A 12-second read reported `mag=0` and no magnetic samples. Both BMI270s
now reported `error0=0,error1=0,regs0=240100,regs1=240100`, both healthy, and
BMP581 reported `valid=1,error=0`. These newer observations supersede the
IMU 2 / barometer failures in the earlier Pi bench table below; they do not
identify what changed physically.

With explicit user authorization, the complete 1 MiB original flash was backed
up, then the motor-inhibited `v2` image was programmed. Every programmed byte
was read back and compared. `mag=1` was confirmed, but no `MAG` samples appeared
in 15 seconds. GNSS obtained a fix with five satellites during that window.

A second, tested image added persistent BMM350 startup diagnostics. All
142,372 payload bytes matched readback before restart. In a 12-second capture:

```text
MAG_STATUS,0
MAG_HEALTH,enabled=1,initialized=0,healthy=0,stage=2,result=-3,chip_id=0,id14=-1,id15=-1,bus_errors=1,last_reg=126,status=0,samples=0
```

The configured I2C2 bus is PB10/SCL and PB11/SDA. Bosch initialization's first
write to command register `0x7E` failed at configured address `0x14`; separate
read-only CHIP_ID probes failed at both `0x14` and `0x15`. This establishes a
communication failure on that bus, not the electrical cause. `chip_id=0` here
is an unfilled initialization field, **not a successful read of a zero ID**.
The probe values `-1` explicitly identify failed register transfers. The
sensor's expected chip ID is `0x33` (decimal 51).

After this check, the board was left on the BMM350-enabled diagnostic `v2` image with
`armed=0,flight_enabled=0`; both BMI270s and BMP581 remained healthy. Default
`pio run` still selects the magnetometer-disabled `v2_bmi270` profile. The
complete Windows host suite passed, including eight new BMM350 diagnostics
checks against the real Bosch API; the ARM build passed. GUI files were not
changed during this sensor check. Raw captures, readback images, checksums and
the original backup are retained locally in
`build-tools/bmm350-check-2026-10-08/` beside the source checkout.

### ER8 wiring and live reception, later on 2026-10-08

ER8 TX was initially connected to the FC's `MCU_TX_PI_RX` output. With the
receiver bound, USB still showed `receiving=0,frames_ok=0,telem_tx=0`. After the
user corrected the wiring to ER8 TX → `MCU_RX_PI_TX` (J2 pin 2) and ER8 RX →
`MCU_TX_PI_RX` (J2 pin 3), a 12-second capture contained 120 USB channel reports.
Across the first/last CRSF status snapshots, valid frames increased by 5,773
and transmitted telemetry by 259, with no new CRC errors or resyncs. Uplink LQ
was 100%, RSSI -14 dBm and SNR 9–10 dB. All 16 channels decoded; CH3 was 989 us,
CH5/CH7 were 1000 us. State was MANUAL, disarmed, failsafe clear and motors
inhibited. The GUI's Receiver page displayed the real incoming values.

This verifies FC reception and outgoing serial telemetry counters; handset
sensor reception and physical surface/motor response were not measured.

### Published flight image programmed, 2026-10-08

After publishing release `gui-2026.10.08-modern`, the user explicitly requested
the full motor-enabled flight firmware. The exact published
`STM32FC-v3-flight-v2.2.bin` from source revision
`11676b227a38e36740699cdb4a38a56982f69db7` was flashed through ROM DFU. Its SHA256
is `6971b8b7fecd3feef2129c8e48c04fdb10c270ace680e517fc6033e8e128bf4d`.
The previous complete 1 MiB flash was backed up and all **142,476 programmed
payload bytes** matched readback before restart.

The subsequent 16-second USB capture confirmed:

- `flight_enabled=1,armed=0,failsafe=0,timing_fault=0`, MANUAL requested/active.
- CH3 throttle 989 us, CH5 1000 us; both ESC commands 1000 us while disarmed.
- Both BMI270s healthy, no driver errors, and gyro calibration ready.
- BMP581 healthy; ER8 receiving with LQ 100%, valid channels and increasing
  outgoing telemetry. No additional CRC errors during the observed window.
- GNSS communicating with valid NMEA at 9600 baud, but no fix in this window.
- BMM350's previously reported I2C startup failure unchanged; SD log inactive.

This is the full flight application with CH5 motor authorization, not a
simulation. Motors were not armed or run as part of this verification. Servo
response, RF-loss behavior on the aircraft, SD-card timing and flight behavior
remain separate physical acceptance work. The standalone Windows GUI was
reopened on COM6. Local evidence and the backup are in
`build-tools/flight-release-2026-10-08/` beside the checkout.

The release EXE also passed a standalone test from an independent Unicode path
with Python/Qt development environment paths removed. GitHub's Windows GUI
workflow passed for the release revision. The release packages preserve that
exact code revision; this later note records the physical programming result.

### Available build profiles

Use `pio run` or `pio run -e v2_bmi270`. BMI270 is the only IMU implementation.
As of 2026-10-09 the default profile enables magnetometer polling (`FC_MAG_ENABLED=1`) and
inhibits motor authorization (`FC_FLIGHT_ENABLED=0`). Magnetic yaw/hold requires measured installation calibration; see [MAGNETIC_HEADING.md](MAGNETIC_HEADING.md). The optional `v2` profile also uses
BMI270, with BMM350 support enabled. IMU transforms, IMU selection,
failsafe behavior and motor gates remain unchanged. Magnetic heading and
ASSIST hold are described separately in the heading guide.

`v2_flight` is an explicit alias of the full motor-enabled `v2_motor_test`
application. It enables CH5 motor authorization while preserving low-throttle
arming, CH5 edge requirements, link-loss handling and fault inhibits. Bench
and flight images use the same controllers and sensors; this is not a switch
from simulated control to real control. The user selected the motor-enabled
image for subsequent aircraft checks on 2026-10-08.

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

## Earlier Raspberry Pi USB bench results

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

The current Windows x64 GUI includes its Python/Qt/serial runtime and DFU utility.
See [Windows packaging](GUI_WINDOWS.md) and [DFU operation](DFU.md).

## Earlier Raspberry Pi verification

On October 8, both ARM profiles (`v2_bmi270` and `v2`) compiled successfully.
The image used for that earlier check was byte-for-byte identical to its
readback-verified image: SHA256 (including DFU suffix)
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
