# Assembled-board bring-up: 2026-10-02

## Confirmed assembly and selected firmware

The original sensor console exposed USB CDC on COM5 (0483:5740); the flashed v3 application now uses COM6. Its existing sensor-test firmware 0.3.1 reported IMU IDs `[67,67]` (`0x43`, BMI323), and `mag=not_assembled`. The user confirmed **two BMI323s and no magnetometer**, and authorized adapting/flashing this controller. This supersedes the planned BMI270/BMM350 component assumption, not the reviewed GPIO map.

The user confirmed that the documented board corrections were checked, motors disabled, and servos safe. Actual rail values, physical output timing, airframe directions and flight behavior are not inferred from that confirmation.

Use **`pio run -e v2_bmi323`**, now the default environment. Both ESCs remain inhibited by `FC_FLIGHT_ENABLED=0`. Do not use the earlier `v2_motor_test`: it is the separate BMI270 assembly target. The `v2` target still builds the BMI270 implementation for that assembly. No automatic sensor-type guessing occurs in flight.

## BMI323 implementation

- Each chip has an independent Bosch API device, SPI bus, bias calibration, filtering and FIFO health state.
- SPI mode 0, explicit SPI selection before reset and a 250 us guard after the post-reset dummy read (datasheet requires 200 us), initial 1 MHz request; actual bus clock depends on the STM32 divider. This is hardware SPI, not the earlier test firmware's GPIO SPI.
- Both sensors run at 400 Hz, +/-8 g and +/-2000 deg/s, high-performance mode. Acceleration bandwidth is ODR/4, gyro bandwidth ODR/2, followed by the existing software filters.
- FIFO is enabled before measurement. It stores six 16-bit words per frame: acceleration XYZ then gyro XYZ. Fill level is interpreted in **words**, not bytes. Startup allows settling and flushes old/dummy frames.
- Startup flush clears historical I3C interface flags once, without ignoring fatal/configuration/feature errors. Runtime errors remain strict. A bounded read returns at most eight paired samples. Chip ID, error register, FIFO backlog, sensor-clock progress, clipping/dummy frames and transport failures are checked. Faults latch until restart; no silent decimation or reinitialization during flight.
- The existing four-second per-IMU calibration, primary/backup selection, disagreement detection, estimator and logging observer are retained. BMM350 is neither initialized nor polled in this profile.
- `IMU_CONFIG,model=BMI323,mag=0,error0=...,error1=...` is emitted periodically so a late USB connection can identify the assembly and driver failures. `regs0`/`regs1` are hexadecimal snapshots: high word = CHIP_ID/revision, low word = ERR_REG; healthy values on this board are `11430000`. `BMP_HEALTH` reports barometer validity/error.

Driver error codes: 0 = no driver fault; 1 = SPI setup; 2 = FIFO setup; 3 = configuration read; 4 = configuration write; 5 = configuration verification; 6 = FIFO flush; 7 = identity/error-register read; 8 = FIFO length/backlog; 9 = sensor clock; 10 = FIFO transfer; 11 = invalid/clipped/dummy data. Initialization failure is `100 - Bosch return code` (for example, 101 corresponds to Bosch -1). Freshness or estimator eligibility may fail even with driver error 0; consult `IMU_HEALTH` as well.

The register layout and frame order follow the [Bosch BMI323 datasheet, sections 5.6/5.7](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi323-ds000.PDF), using the already-vendored Bosch BMI3/BMI323 API. No vendor source was modified. Mounting transforms remain the reviewed PCB transform and require physical tilt-direction checks on this assembly.

## GUI

```text
python tools/gui.py --port COM6 --capture bench-session.txt
```

The GUI detects one STM32 CDC port on Windows when `--port` is omitted; multiple boards require an explicit port. Its title identifies BMI323/BMI270 from live `IMU_CONFIG`, and optional `--capture` appends received diagnostics to a local file while the GUI owns the serial port. Sensor-test JSON from firmware 0.3.1 is not the flight application's tagged-CSV protocol.

## Evidence before flashing

- A ten-second capture of the existing 0.3.1 sensor-test firmware showed both IMUs streaming at approximately 64.9 samples/s each and the barometer at 25.3 samples/s, with zero reported USB drops, GPS overruns or barometer errors during the observed interval. These rates describe that test firmware, not this 400 Hz flight build.
- The BMI323/no-magnetometer ARM build passed: 47,248 bytes static RAM and 121,808 bytes flash.
- All existing host checks plus 22 new real-Bosch-API/BMI323 register/FIFO model checks passed (296 C/C++ assertions total). Cases cover byte/word interpretation, signed axes, device identity, frozen clock, FIFO backlog, dummy/clipped frames, two independent buses, calibration, failover, ambiguity and stale-data eligibility.
- GUI parser checks include live sensor-model labeling. These are host checks, not physical acceptance.

## Physical bring-up results

On 2026-10-02, the supplied `FlightController.zip` was inspected for its USB command and sensor startup behavior. Sending `\ndfu\n` to the original 0.3.1 firmware entered **0483:df11**, ROM serial **3154365B3034**. A complete 1 MiB backup was saved locally before any programming; SHA-256: `5cb2bc806696f873c4d01b9c889844fd7b258bc00d1b08acd1ef4e66791301b8`.

The motor-inhibited `v2_bmi323` firmware was flashed and every programmed byte read back and compared. **Software DFU was then verified with the application watchdog running**, followed by a return to CDC on COM6. The GUI's **Enter DFU** button was also physically tested: it stopped monitoring, released the port and displayed verified ROM enumeration. See [DFU operation and implementation](DFU.md).

Two real-board issues were found and corrected:

1. BMI323 SPI selection needs 200 us after the post-reset dummy read before further accesses. The Bosch API's generic 2 us inter-access delay was insufficient on this board. An explicit 250 us guard removed the recurring IMU 1 `ERR_REG=0x0800` condition. Both hardware SPI buses now report chip/revision `0x1143`, zero errors and healthy data.
2. A warm MCU/DFU reset leaves the BMP581 powered and measuring. Resetting the sensor before the Bosch NVM-ready check restored pressure/temperature readings at approximately 50 Hz. A regression models this warm restart.

The GUI displays sensor errors, barometer health, SD/USB statistics, RC input, servo commands, attitude and GNSS. Its **All live parameters** table retains the latest tagged-CSV message per tag/task, including all scheduler and estimator fields. Raw session data is captured locally; it is not committed because it may contain GNSS coordinates.

The USB queue was increased from 384 to 2048 bytes after status bursts caused diagnostic drops on the first build. This is a bounded queue: it still drops whole messages rather than blocking flight control. Dropped diagnostics are visible via `usb_drops`.

### Limits of the bench result

- Motors remain inhibited; reported ESC commands are 1000/1000 microseconds. Physical pin timing, servo direction and airframe behavior have not been measured.
- RC input was absent during these captures, so the controller correctly reported failsafe. With healthy calibrated attitude, its requested response is level stabilization. Radio/transmitter telemetry and live RC-loss/recovery are not physically qualified yet.
- GNSS emits checksum-valid NMEA at 9600 baud, but no usable position fix was available indoors during the capture.
- SD logging was inactive (`SD_DBG,0`, zero bytes). Runtime SD behavior under a fitted card, sustained writes and power loss remains unqualified.
- Scheduler `overruns` count dispatch more than half a period late OR execution longer than a period; they are not a pure execution-time counter. The 4 kHz logging-service target is not guaranteed by this cooperative scheduler; inspect actual control/IMU timing and logger throughput under real SD load before flight.
- Stationary sensor validity, calibration and brief USB sessions are not flight qualification. Mounting/sign checks, vibration, thermal drift, actuator failsafe behavior and airframe gain tuning remain required.

### Final image and verification

- Final `v2_bmi323` image SHA-256 (including DFU suffix): `79a0da740a19fdfa6f5e8f8155ea12887401704b15748c0bb7baa1cdcf393472`.
- Payload/readback: **122,552 bytes**, all identical. Build report: **48,928 bytes static RAM**, **122,080 bytes flash** (binary padding/data-load accounting differs from the size report).
- Both `v2_bmi323` and the retained `v2` BMI270 target compile. Host suite: **306 passing C/C++ checks** across the complete suite and latest targeted BMI323 timing tests; **12 GUI parser checks**, **6 DFU helper tests**. These are software tests, not hardware fault-injection qualification.
- The final 12-second capture received 240 attitude/output messages (about 20 Hz), 606 BMP messages (about 50 Hz), all 14 scheduler records per second, and **zero reported USB drops**. Both IMUs stayed healthy and calibrated, error registers zero, with no disagreement or control-fault latch.
- That interval's largest observed scheduler pass was **4,369 us**, BMI task **1,316 us**, BMP task **1,293 us**, control task **109 us**. Eleven control late-dispatch events were reported; this is not a claim of jitter-free 400 Hz control. The GUI exposes these counters for further bench qualification.
- Original backup, every intermediate image/readback, final image/readback and raw serial captures remain under the local workspace's `build-tools/bench-2026-10-02/` directory, outside the Git source tree.
