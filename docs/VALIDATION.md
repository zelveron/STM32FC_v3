# Validation and acceptance status

Hardware evidence is in [the 2026-10-08 assembled-board record](ASSEMBLED_BOARD.md).
That Windows session verified both BMI270s, BMP581, GNSS communication/fix,
ER8 reception and outgoing CRSF telemetry counters. The user reports BMM350
online as of 2026-10-09. No new hardware or flight tests were performed for
the magnetic-heading software change; installation calibration is still pending.

## 2026-10-09 magnetic-heading software checks

| Check | Result |
|---|---|
| Complete Linux GCC host suite, `tools/test_host.py` | 366 C/C++ checks PASS; includes 34 magnetic-heading checks, production application integration and simulated heading recovery |
| Python parser / connection / Qt UI / DFU tests | 44 PASS |
| Offline magnetic calibration / existing log-tool tests | 6 + 10 PASS |
| ARM `v2`, `v2_bmi270`, `v2_flight` | All compile and link; no board flashed |
| Qt source GUI self-test and visual preview | Five pages rendered offscreen; source/hold/fallback status checked; no serial connection opened |

ARM images use 49,452 bytes of RAM and at most 145,976 bytes of flash.
The Windows workflow includes the new firmware and calibration tests but
was not run in this Raspberry Pi session; no new Windows EXE was released.
Magnetic tests use synthetic observations, and aircraft simulation is not
an identified model of this airframe. See [MAGNETIC_HEADING.md](MAGNETIC_HEADING.md).

`v2_flight` enables motor authorization but does not add flight qualification.
After these software checks, the user explicitly requested deployment:
[the flight image was flashed and readback-verified](DEPLOYMENT_2026-10-09.md).
The runtime check confirmed motor authorization enabled, healthy IMUs/BMP581,
and the outstanding BMM350 sample-read error and calibration gate.
The September results below are historical; their statements about untested
hardware and the former Tk GUI describe that date only.

Date: 2026-09-28. Starting revision: `3daf518b5e1759f63fb84008d8d34c5e17c0dda6` from STM32FC_v2. These results cover the firmware carried into STM32FC_v3 for the v2.2 PCB. **No board was flashed, no physical hardware test was performed, and no flight was conducted.**

## Automated results

| Check | Result | What it establishes |
|---|---|---|
| STM32 `v2` build | PASS | Application, Bosch APIs, board HAL, pinned framework and linker compile/link |
| STM32 `v2_motor_test` build | PASS | Separate STM32 build with motor authorization enabled; no hardware flashed |
| STM32 `crsf_probe` build | PASS | Independent ER8 UART4/USB diagnostic firmware compiles/links |
| Existing portable native checks | 83 PASS | Estimator/control/mixer/arming/scheduler/log primitives |
| Application fault regressions | 63 PASS | Real application with simulated HAL/devices, including RC/failsafe recovery, USB commands, asynchronous SD service across arming, GPS validity, mode transitions and control deadline fault |
| Default bench profile | 4 PASS | No RC at boot, asynchronous logger service, motor arming inhibited, both ESC outputs held at minimum |
| BMI270 real-driver model | 17 PASS | Official Bosch C API plus real driver/dual manager with SPI registers and FIFO emulated |
| BMP581 real-driver model | 10 PASS | Official Bosch C API plus real driver with I2C registers and ready events emulated |
| SD busy-wait fault tests | 2 PASS | Actual vendored read/write wait loops terminate under simulated permanently busy card |
| Async logger/card model | 22 PASS | Immutable DMA buffer, consume-on-ready, asynchronous tail, CRC/session layout, command/data timeout, timer wrap, card errors and restart rejection |
| Control/estimator/telemetry additions | 36 PASS | Turn geometry, wrapped roll, command shaping, downstream anti-windup, rudder authority, pitch trim, flight-state changes, budget/fairness/fix deduplication, notch and innovation rejection |
| Extended flight scenarios | 24 PASS | Four 70-second toy-model flights, 15/18/24 m/s, wind, with/without simulated pitot, 50 Hz hold/frame delay/servo lag, vibration and residual bias |
| Python log/calibration/spectrum tools | 10 PASS | Mixed records, torn/corrupt/stale tails, CSV units, six-face fitting, known-frequency spectrum and gap rejection |
| Nominal flight simulation | 9 PASS | Toy-aircraft trim, input sign, bounded flight and attitude-estimator regression |
| ASSIST simulation | 4 PASS | Toy-aircraft bank tracking, return toward level and bounded response |
| GUI parser/freshness | 7 PASS | Headless parsing of initialization, attitude, dual-IMU ambiguity and stale pressure/GPS/attitude |

Total: **274 C/C++ behavioral checks, including 37 simulation checks, plus 17 Python checks**. These counts represent assertions, not independent hardware safety guarantees. The legacy TKOFF trace is not counted as a self-checking simulation; dedicated native/application tests cover roll assistance and pitch/yaw/throttle passthrough.

Build memory report for the final application: **47,592 bytes static RAM of 131,072** and **139,560 bytes flash of 1,048,576**. This does not measure worst-case stack depth, runtime timing, heap use or power stability. The motor-enabled profile uses the same static RAM and 139,672 bytes flash. The CRSF probe uses 9,228 bytes static RAM and 36,424 bytes flash.

The final builds reported no compiler warnings/errors. The repository diff whitespace check passed under Windows CRLF handling. Build outputs are under `.pio/build/<environment>/`; host executables and logs are under `build/host/`. Captured summaries are in [validation-results](validation-results/host.txt).

## Reproduce

Windows with Visual Studio 2022 C++ Build Tools and PlatformIO Core:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/test_host.ps1
python tests/gui_parser.py
python tests/log_tools.py  # requires NumPy for spectrum checks
pio run -e v2 -e v2_motor_test -e crsf_probe
```

The host script accepts `-VcVars` for another Visual Studio installation. It compiles Bosch sources as C and application/tests as C++17, preserving separate objects for vendor and application files with similar names. The main regression executable explicitly enables motor authorization and asynchronous SD service in the **simulated HAL only**, so tests exercise those policies; the `bench` executable and default STM32 binary retain motor inhibit.

The driver emulators verify protocol/configuration/conversion and fault handling, not analog sensor response, vibration, SPI signal integrity or electrical behavior. The BMM350 API is included in the ARM build but has not been run on real hardware or a magnetic-physics model. GUI tests do not verify visual layout or serial-port operation.

## Scope of the new simulation evidence

The extended scenarios exercise the real filters/controllers at 400 Hz, with 50 Hz output hold, one PWM-frame delay, a 50 ms actuator time constant and normalized slew limit. They include 25-second turns in each direction, modest deterministic vibration/residual gyro bias, 15/18/24 m/s conditions and a wind case. A simulated pilot adjusts throttle to maintain cruise speed; that logic is outside ASSIST and does **not** add autothrottle to the firmware. One case supplies ideal simulated pitot data; the production application still marks airspeed unavailable.

The host card model tests the portable state machine, not physical SDIO electrical/DMA behavior. The ARM builds compile the actual register adapter. No real SD card has been written or removed during these tests. GNSS velocity fusion, EKF replacement and hardware-derived gains/filter frequencies have not been invented or claimed complete.

## Required physical acceptance work

Record results and measured values for each step. The following work is outstanding; passing host tests does not mark it complete.

### 1. Correct and measure the board

- Complete the VCAP, reference and BMM350 supply/pull-up review/rework in [HARDWARE_V2.md](HARDWARE_V2.md). Inspect the assembled board, not only the design files.
- Resolve the regulator input-voltage limit before using 6S. Use a current-limited source within a verified board input range for initial bring-up.
- Verify no shorts, then measure all rails, VREF+/VDDA and each VCAP node. Check startup sequencing, ripple, temperatures and current draw before attaching actuators.
- Verify actual crystal frequency and peripheral timing. Check SPI/I2C waveforms, including the BMP581 series resistors and BMM350 pull-up high levels.
- Verify the servo rail and receiver supply. Prevent unintended paralleling of the ESC BECs and board regulator.

### 2. Bench firmware, motors mechanically disabled

- Keep `FC_FLIGHT_ENABLED=0` for initial checks. Confirm `FLIGHT_GATE,bench_motor_inhibit` on USB and B-prefixed mode text when the link is healthy.
- Confirm each device's identity/initialization, both IMUs producing samples, and `IMU_HEALTH` active index/ambiguity/attitude-valid fields. Indices in firmware are 0=U4, 1=U7.
- Hold still through calibration; deliberately disturb the board before four seconds and confirm that a new stationary window is needed. Check selected bias readiness. Repeat with only the backup available to test its independent calibration.
- Test level, right wing down, left wing down, nose up/down and yaw-rate signs. Measure six-face acceleration and apply each IMU's configured offset/scale using the workflow in [CONTROL_AND_TUNING.md](CONTROL_AND_TUNING.md); values currently remain identity until measured.
- Measure **all eight actual output pins** with an oscilloscope/logic analyzer: 50 Hz, center/endpoints, minimum motor pulse and power-up/reset behavior. Confirm physical SERVO7 is inactive and ESC outputs are on SERVO8/9.
- Verify each servo moves correctly in MANUAL and corrects in the opposite direction of an imposed disturbance in ASSIST. Test TKOFF aileron correction and direct pitch/yaw response independently. Check for binding before full travel.
- Leave the USB host disconnected, reconnect it, and stall host reads; verify control timing does not depend on the computer.

### 3. Receiver and motor authorization tests

- Configure the TX16S and ER8 exactly as [TX16S_SETUP.md](TX16S_SETUP.md); inspect actual received channels, not just transmitter mixes.
- After hardware/output checks, build/select `v2_motor_test` deliberately while keeping motors mechanically disabled. Verify boot with CH5 high does not arm; high throttle blocks initial arming; low/high CH5 at idle succeeds; CH5 low immediately returns both ESC signals to minimum.
- Remove RF reception while observing CRSF input and PWM. Confirm stale channel packets stop, motor cutoff follows the last valid packet by the intended timeout plus output-frame delay, and surfaces level/center according to sensor eligibility.
- Restore the link with CH5 high: verify the requested **automatic throttle restoration after 300 ms stable reception**. Repeat with CH5 low: motors must stay off. Do not test this with powered propellers/EDFs capable of thrust.
- Verify loss while previously disarmed and controller reboot do not create arming authorization. Measure the receiver's own detection delay; FC timeout alone is not the total RF-loss response time.

### 4. Fault and timing measurements

- Safely isolate one IMU's communication without shorting powered nets; verify backup takeover only when calibrated. Test both unavailable and persistent disagreement with controlled injected data where practical.
- Verify no new GNSS/pressure data becomes unavailable and the radio/GUI distinguishes old values. Validate GNSS speed as ground speed and yaw as relative yaw.
- Exercise sensor-bus failure with proper fault-injection equipment. A shared power rail short cannot be contained by software driver failover.
- Measure worst-case `SCHED` timings, including telemetry, missing sensors, USB traffic and receiver bursts. Confirm margins for the 2.5 ms target and the 20 ms control-fault threshold.
- Deliberately stall a test firmware task and measure watchdog timeout, output behavior through reset, and inhibited post-watchdog startup. Nominal 100 ms depends on the MCU's imprecise LSI oscillator and is not an exact timing guarantee.
- SD runtime logging now continues after arming. Qualify it on the bench first: several FAT32 cards, fragmented/near-full media, card removal, forced long busy intervals, power-cut recovery, immutable DMA buffers and control timing under load. Confirm asynchronous timeouts never wait inside a service call. See [LOGGING.md](LOGGING.md).

### 5. Airframe qualification

Verify mechanical control signs, throws, CG, trim, servo supply capacity, airspeed/stall behavior and ESC operation for the actual aircraft. Gains and slew limits require controlled tuning and a staged flight-test process with an experienced pilot. Validate MANUAL before depending on stabilization, then characterize ASSIST/TKOFF and failure transitions. Centered pitch in ASSIST/failsafe is not a tested best-glide or stall-protection command.

The present firmware provides no return-to-home or navigation rescue. BMM350 magnetic calibration/fusion and ASSIST heading hold are now implemented (see [MAGNETIC_HEADING.md](MAGNETIC_HEADING.md)); measured installation calibration and airframe validation remain outstanding, along with airspeed sensing, navigation estimation and physical logging qualification. A second IMU is useful but does not qualify the system to tolerate arbitrary single failures.
