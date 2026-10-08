# STM32FC_v3

Custom flight-controller firmware for RC fixed-wing aircraft, targeting the **manufactured v2.2 STM32F407VGT6 PCB**. It provides MANUAL, roll/pitch ASSIST, TKOFF wing leveling, dual-IMU sensing, CRSF radio control/telemetry, USB diagnostics, and asynchronous SD flight logging.

**v3 is the software repository generation; v2.2 is the PCB revision.** PlatformIO profiles retain their `v2` names to identify that board. This project continues [STM32FC_v2](https://github.com/zelveron/STM32FC_v2); older board instructions are archived in [docs/history](docs/history/README-v1.md).

> **Status — 2026-10-08:** Both BMI270s and BMP581 produce healthy data on the current board. ER8 CRSF reception works after correcting the J2 TX/RX wiring; the FC sends attitude, vario and mode telemetry. GNSS has obtained a position fix. BMM350 initialization still fails on I2C2 and remains a hardware investigation item. `v2_flight` is the full MANUAL/ASSIST/TKOFF firmware with CH5 motor authorization; the default build retains motor inhibition. These are bench observations, not flight qualification. See [current results](docs/ASSEMBLED_BOARD.md).

## Current card — 2026-10-08

The user identified the card connected to the Raspberry Pi as the **BMI270** assembly. The default environment in this checkout is now **`v2_bmi270`**, using the existing dual-BMI270 driver. It preserves the previous profile's disabled magnetometer and motor inhibition (`FC_MAG_ENABLED=0`, `FC_FLIGHT_ENABLED=0`). Build with `pio run` or `pio run -e v2_bmi270`. BMI270 is the only supported IMU; the legacy driver, vendor library and build profile have been removed.

The BMI270 path runs Bosch's reset/SPI-selection sequence and uploads all 8192 configuration bytes independently to each IMU. Initialization uses 1 MHz SPI, then waits 80 ms after enabling measurement before switching to 5 MHz FIFO service and flushing startup frames. This covers the gyroscope's documented 45 ms startup; without settling, dummy FIFO frames can latch the strict paired-frame parser as failed. See the [Bosch BMI270 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf), sections 1 and 4.4. Runtime FIFO, freshness, clipping and failover checks remain active.

`IMU_CONFIG` reports actual BMI270 failure stages and register snapshots: `regsN = CHIP_ID << 16 | INTERNAL_STATUS << 8 | ERR_REG` (hexadecimal, normally `240100`). Initialization errors are `100 - Bosch return code`: `103` means chip ID mismatch and `109` means configuration-load failure. Other codes: 1 SPI setup, 2 power-save setup, 3 configuration read, 4 sensor configuration/enable, 5 FIFO setup, 6 flush, 7 identity/health, 8 FIFO length/backlog, 9 sensor clock, 10 FIFO transfer, 11 paired-frame parsing, 12 clipping. Earlier IMU 2/BMP581 failures are retained in the dated bench record; both devices were healthy in the later Windows checks. `MAG_HEALTH` preserves BMM350 startup errors and identity-probe results continuously.

The GUI now separates GNSS communication from navigation status. Firmware emits `GPS_HEALTH` every second, including after the initial bring-up period: `rx` (recent UART bytes), `nmea` (recent checksum-valid sentences), `fix`, `used` (fresh GGA satellites used, `-1` unknown), `visible` (fresh GSV: `-1` unknown, `0` none reported, `1` satellites in view), `baud`, cumulative `bytes` and `messages`. A missing fix no longer hides receiver communication or GGA satellite counts. Positive GSV evidence is retained for five seconds so an empty report from another constellation cannot immediately erase it. Stale controller telemetry is displayed separately from a silent GNSS receiver.

## Windows GUI

The current source includes a modern Qt ground station with five pages: **Flight deck, Receiver, Sensors, Diagnostics and System**. It provides a 3D aircraft view, artificial horizon, offline GPS ground track, speed/altitude/climb, satellite and component indicators, all 16 RC channels, eight PWM commands, sensor diagnostics and a searchable telemetry table. Explicit **Preview** mode works without hardware and cannot send commands. Stale or invalid telemetry clears live instruments; last raw values remain available with their ages.

Download the [standalone Windows x64 EXE](https://github.com/zelveron/STM32FC_v3/releases/download/gui-2026.10.08-modern/STM32FC-GUI.exe) or the [complete portable ZIP](https://github.com/zelveron/STM32FC_v3/releases/download/gui-2026.10.08-modern/STM32FC-GUI-2026.10.08-modern-Windows-x64.zip) from the [modern ground-station release](https://github.com/zelveron/STM32FC_v3/releases/tag/gui-2026.10.08-modern). Python, Qt, serial support and DFU utilities are bundled. The target is **Windows 10/11 x64**, with no separately installed Python or Qt required; operating-system USB drivers remain separate. See the [Windows guide](docs/GUI_WINDOWS.md) for setup, source, licenses and reproducible packaging. Flight and bench firmware downloads are separate, explicitly named release assets; opening the GUI never flashes either image.

For source-based use:

```text
pio run -e v2_bmi270
python tools/gui.py --port /dev/ttyACM0 --capture bench-session.txt
```

On Windows use the appropriate COM port instead. The GUI includes **System → Enter ROM DFU** and a **Diagnostics** table; use `--demo` for the simulated preview. See [DFU instructions](docs/DFU.md).

## Contents

- [What is implemented](#what-is-implemented)
- [Hardware](#hardware)
- [Transmitter, modes and failsafe](#transmitter-modes-and-failsafe)
- [Sensor fusion and two IMUs](#sensor-fusion-and-two-imus)
- [Controllers and filters](#controllers-and-filters)
- [Telemetry and ground tools](#telemetry-and-ground-tools)
- [Non-blocking SD logging](#non-blocking-sd-logging)
- [Build and configure](#build-and-configure)
- [Validation](#validation)
- [Roadmap](#roadmap)
- [Documentation and source layout](#documentation-and-source-layout)
- [Third-party code and licensing](#third-party-code-and-licensing)

## What is implemented

This table describes code present in the repository. Physical validation is a separate milestone.

| Area | Implemented | Current boundary |
|---|---|---|
| MANUAL | Direct pilot surface/throttle commands through the mixer | Motor authorization still applies; direct control needs no IMU |
| ASSIST | Roll/pitch angle control, rate controllers, trim, turn geometry and rudder damping | Pilot throttle; no altitude, speed, position or heading hold |
| TKOFF | Wing leveling with limited bank demand | Pilot pitch, rudder and throttle; no automatic launch sequence |
| Outputs | Two ailerons, two elevators, rudder, nosewheel, two ESCs | Standard PWM; endpoints/directions require aircraft setup |
| IMUs | Two BMI270 drivers, 400 Hz FIFO processing, individual calibration/filtering/health | Primary/backup selection, one attitude estimator |
| Attitude | Quaternion Mahony-style fusion, gravity trust gates, bounded bias correction | Relative yaw; no absolute heading or navigation EKF |
| Other sensors | BMP581 pressure/vario, SAM-M10Q GNSS, BMM350 compensated diagnostics | GNSS/magnetometer are not fused into attitude |
| RC/failsafe | CRSF, arming checks, link-loss throttle cut and level/center response | Automatic throttle recovery after stable reception, as selected for this aircraft |
| Telemetry | CRSF attitude, vario, GPS and mode; USB diagnostics | Radio refresh depends on ELRS settings/link |
| Logging | Bounded ring, preallocated FAT32 file, asynchronous SDIO DMA | Boot allocation can block; physical card qualification pending |
| Fault handling | Sensor freshness, disagreement latch, stabilization lockout, control-deadline inhibit, watchdog | Shared MCU, power and output failures remain common failure points |
| Development | Application/device/card models, flight simulation, GUI, log/calibration/spectrum tools | Host evidence is not flight qualification |

See the [complete feature catalog](docs/FEATURES.md) for all startup, mode, health and diagnostic behavior. There is currently **no RTH, autonomous mission, automatic landing, altitude hold, stall protection, propulsion-battery/current measurement, DShot or MAVLink implementation**.

## Hardware

### Board and intended aircraft equipment

The GPIO map was checked against the supplied v2.2 KiCad schematic and PCB. The [hardware guide](docs/HARDWARE_V2.md) records the source PCB hash, electrical evidence and datasheet links. The original KiCad project and BOM are not included in this firmware repository.

| Item | Part / configuration | Use |
|---|---|---|
| MCU U26 | STM32F407VGT6; 1 MiB flash, 128 KiB ordinary SRAM plus 64 KiB CCM | DMA buffers use ordinary SRAM; CCM is not general DMA memory |
| Crystal Y2 | Confirmed 8 MHz | 168 MHz core / 48 MHz USB clock |
| Primary IMU U4 | Bosch BMI270 on SPI1 | 400 Hz paired accel/gyro FIFO, +/-8 g and +/-2000 deg/s |
| Backup IMU U7 | Bosch BMI270 on SPI2 | Independent driver, calibration and health |
| Barometer U9 | Bosch BMP581 | 50 Hz pressure/temperature, relative pressure altitude and vario |
| Magnetometer U29 | Bosch BMM350 | 25 Hz compensated diagnostics; not heading fusion |
| GNSS U28 | u-blox SAM-M10Q | NMEA position, ground speed, course, MSL altitude, satellites |
| microSD | SDIO, FAT32, 512-byte sectors | Preallocated flight logs |
| Other storage | W25Q16 U3; 24AA32 U5 | W25Q16 unused/deselected; 24AA32 is GNSS-side, not MCU parameter storage |
| Transmitter | RadioMaster TX16S MK3 MAX, EdgeTX + ELRS | AETR controls, CH5 arm, CH7 mode |
| Receiver | RadioMaster ER8, ExpressLRS | Dedicated serial CRSF connection to J2 |
| Servos | EMAX ES08MD II 13 g | Initial 50 Hz PWM; actual supply/mechanics need verification |
| ESC options | AT55A or Flycolor Waterproof 120 A 2-6S, 5.5 V / 5 A BEC option | Standard PWM; verify protocol, endpoints and BEC arrangement per installed ESC |
| Intended propulsion battery | 3-6S LiPo, aircraft dependent | **6S needs a corrected/qualified FC power input**; ESC ratings do not qualify the FC |

### Electrical findings on the manufactured design

These findings come from design files, not measurements of the assembled board. Firmware cannot repair them. Read [required hardware corrections](docs/HARDWARE_V2.md#manufactured-board-corrections-come-first) before powering the board.

| Finding | Required resolution |
|---|---|
| C47 connects VCAP1 to VCAP2 | Separate specified 2.2 uF low-ESR decoupling from each VCAP pin to ground |
| VREF+ has a capacitor but no DC reference feed | Correct reference supply/decoupling; verify VDDA/VSSA |
| BMM350 VDD uses `STM1V3`, calculated at approximately 1.2 V | Provide a verified 1.8 V core rail and inspect shared loads |
| BMM350 SDA/SCL pull-ups use that low rail despite 3.3 V VDDIO | Correct the pull-up supply separately from the core supply |
| TPS513885 regulators connect to VBAT; recommended VIN ends at 24 V | A full 6S pack is 25.2 V before transients; resolve input design before direct 6S use |

Servo headers share **VOUT2**. An ESC's red BEC lead connects its supply to that rail. Verify the receiver/servo power arrangement; do not directly parallel two BECs or a BEC and the board regulator without a designed sharing arrangement. Rail voltage, load capacity, ripple and startup behavior remain bench-test items.

### GPIO and bus assignments

| Function | STM32 pins | Notes |
|---|---|---|
| BMI270 U4 / SPI1 | PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS | Primary; interrupt outputs unconnected |
| BMI270 U7 / SPI2 | PB13 SCK, PB14 MISO, PB15 MOSI, PB12 CS | Backup; FIFO polling replaces DRDY interrupts |
| BMP581 / I2C1 | PB6 SCL, PB7 SDA | Address `0x47`; 100 kHz bus |
| BMM350 / I2C2 | PB10 SCL, PB11 SDA | Address `0x14`; PE12 interrupt wired but unused |
| SAM-M10Q / USART2 | PA2 FC TX, PA3 FC RX | Starts at 38400 baud; passive detection also tries 9600/115200/57600/19200; GNSS navigation rate is not configured |
| ER8 / UART4 | PA0 FC TX, PA1 FC RX | Uninverted full-duplex CRSF, 420000 baud, 8N1 |
| USB CDC | PA11 D-, PA12 D+ | Diagnostics and ground commands |
| microSD / SDIO | PC8-PC12, PD2 | DMA2 stream 6 / channel 4 reserved for logger |
| W25Q16, inactive | PB0 CS, PB3 SCK, PB4 MISO, PB5 MOSI | Possible SPI3 bus, not active parameter storage |
| BMP power gate | PA15 LOW | Enables supply through Q2 |
| GPS power/reset | PE3 LOW enables power; PE2 HIGH asserts reset | Initialization enables power and releases reset |

**ER8 on J2:** pin 1 = GND; pin 2 = PA1 / FC RX (connect ER8 TX); pin 3 = PA0 / FC TX (connect ER8 RX). **J2 has no power pin.** Supply ER8 separately from a suitable regulated source and share ground. Use actual connector labels, not cable colors. ER8 EXT voltage sense is not the receiver power input.

### Physical outputs

All listed output headers have **pin 1 = GND, pin 2 = VOUT2, pin 3 = signal**. Identify pin 1 from the PCB footprint. Logical numbers below correspond to firmware arrays and `OUT` reports, human-numbered 1-8.

| Logical output | Function | PCB connector | GPIO / timer |
|---|---|---|---|
| 1 | Left aileron | SERVO1 / J4 | PC7 / TIM3 CH2 |
| 2 | Right aileron | SERVO2 / J5 | PC6 / TIM3 CH1 |
| 3 | Left elevator | SERVO3 / J6 | PD15 / TIM4 CH4 |
| 4 | Right elevator | SERVO4 / J7 | PD14 / TIM4 CH3 |
| 5 | Rudder | SERVO5 / J10 | PD13 / TIM4 CH2 |
| 6 | Nosewheel | SERVO6 / J12 | PD12 / TIM4 CH1 |
| 7 | Left ESC | **SERVO8 / J20** | PA8 / TIM1 CH1 |
| 8 | Right ESC | **SERVO9 / J21** | PA9 / TIM1 CH2 |
| Unused | No timer PWM | **SERVO7 / J19** | PD11 |
| Spare, inactive | Future output | SERVO10 / J22 | PA10 / TIM1 CH3 |

Logical output 7 is **not** physical SERVO7. Defaults are **50 Hz**, **1000-2000 us**, with 1500 us surface centers. Each timer group shares a frequency. A 400 Hz control loop does not produce 400 Hz servo pulses. Set individual servo reversals for mirrored mechanics; the default reversal array is all false and has not been matched to an airframe.

### Mounting

Mount **components up, nose toward the left / Y2 crystal side in KiCad's unrotated top view (decreasing PCB X)**. Body axes are forward/right/down. Both IMU transforms are `{sensor X, -sensor Y, -sensor Z}`; acceleration is subsequently converted from specific force to the estimator's gravity convention.

Level stationary processed acceleration should be near `0, 0, +1 g`. Right wing down must give positive roll; nose up must give positive pitch. This mapping is based on design review and still requires physical sign checks. See the [orientation drawing](docs/HARDWARE_V2.md#chosen-mounting-orientation).

## Transmitter, modes and failsafe

Follow the [TX16S MK3 MAX / ER8 guide](docs/TX16S_SETUP.md) for EdgeTX mixes, ELRS, telemetry discovery and alarms. Installed firmware versions and a radio model export are not yet recorded. SF for arm and SC for modes are example switch choices; verify received values.

| TX channel | Purpose | Required behavior |
|---|---|---|
| CH1 | Roll | Right roll increases value |
| CH2 | Pitch | Pull back / nose-up increases value |
| CH3 | Throttle | Low about 1000 us, high about 2000 us |
| CH4 | Rudder | Right yaw increases value |
| CH5 | Arm | OFF about 1000, ON about 2000; threshold >1700 us |
| CH6 | Spare | Unused |
| CH7 | Mode | <1333 MANUAL; 1333-1666 ASSIST; >=1667 TKOFF |
| CH8-CH16 | Spare | Unused by the FC |

Start arm OFF, low throttle, MANUAL. After initialization, hold still for **four seconds of valid IMU samples**. Movement restarts calibration; this is not four seconds from battery connection. `EST,bias_ready=1` confirms the selected IMU calibration, and `IMU_HEALTH` reports both devices.

- **MANUAL:** direct pilot surfaces and throttle through the mixer; no IMU required. Motors still follow arming/failsafe rules.
- **ASSIST:** approximately +/-40 degrees bank and +/-26 degrees pitch around configured pitch trim. Centered sticks request level roll and trim pitch. Pilot retains throttle and direct rudder with bounded transient damping.
- **TKOFF:** roll-only leveling with approximately +/-10 degrees bank demand. Pilot retains pitch, rudder/nosewheel and throttle, and chooses when to exit. There is no launch detector, autothrottle or climb schedule.

Initial motor authorization requires a usable link, CH5 observed low, then a rising CH5 transition at throttle <=5%. CH5 high at boot cannot arm. CH5 low cuts motor commands; disarmed surfaces still operate. **Default firmware rejects motor authorization entirely.**

**RC loss:** after 200 ms without valid channel packets, throttle goes to minimum. Healthy calibrated attitude permits level stabilization; otherwise surfaces center. This does not provide RTH, stall protection or a guaranteed glide.

**RC recovery:** after 300 ms continuously usable reception, previously authorized throttle returns at the current stick position if CH5 remains high. This is the selected automatic-recovery policy: no low-throttle rearm or recovery throttle ramp. CH5 low, a new boot or a permission fault prevents restoration. Test with motors mechanically disabled. Receiver detection and PWM-frame delays add to the overall response time.

Loss of healthy attitude during stabilization latches a MANUAL fallback until restart. A control gap over 20 ms after first arming latches motor inhibit. An approximately 100 ms watchdog checks critical progress; watchdog recovery also inhibits motors. Actual timing and reset-output behavior still require measurement.

## Sensor fusion and two IMUs

Each BMI270 receives the official Bosch configuration image and maintains its own driver, FIFO, calibration, filters and health state. Both run at 400 Hz, processing every complete sample in a FIFO batch. Each gyro independently calibrates from 1600 stationary samples; movement, invalid samples or acceleration outside the stationary gate restart the window. Startup calibration stops after first arming for that boot.

The selected IMU feeds one quaternion Mahony-style estimator. Acceleration magnitude, body rate and gravity-direction innovation limit accelerometer correction during maneuvers. Additional quiet-window gates bound residual gyro-bias learning. Yaw is relative and drifts: **BMM350 and GNSS are not fused into attitude**.

U4 is primary; healthy calibrated U7 can take over if U4 becomes unhealthy, retaining attitude and clearing residual bias from the previous sensor. It does not automatically switch back that boot. Persistent disagreement between otherwise valid sensors latches ambiguity, since two sensors cannot establish a majority winner.

The second IMU provides backup sensing and fault detection, not a second independent FC or protection against shared power failures. Exact sample-clock synchronization, dual independent estimators and temperature calibration remain future work. See [sensor fusion](docs/SENSOR_FUSION.md) for health checks and thresholds.

## Controllers and filters

Roll/pitch assistance uses wrapped attitude errors, Euler-to-body-rate geometry, feedforward plus PI rate control, command acceleration limits, a 0.25-second entry blend, surface slew limits and applied-command feedback for anti-windup. Derivative support exists, but default D gains are zero. A debounced flight-state heuristic controls integration; it is not a reliable airborne/stall detector.

Rudder remains a pilot command with bounded transient yaw damping; full rudder overrides damping. Measured rates support turn geometry without a pitot. Airspeed-based scaling/coordination is implemented as an input path but inactive in the production application because there is no valid airspeed source. **GNSS ground speed is never substituted for airspeed.**

Defaults are a **30 Hz gyro low-pass**, **15 Hz acceleration low-pass**, and an optional gyro notch **disabled until measured**. Six-face acceleration calibration is supported per IMU; offsets/scales remain identity pending measurements. Gains, trim, filters, authority and slew limits require tuning on the actual aircraft. See [control and tuning](docs/CONTROL_AND_TUNING.md).

## Telemetry and ground tools

TX -> ER8 -> FC carries the pilot channels above. FC -> ER8 -> TX sends these CRSF frames. Rates are FC generation targets, **not guaranteed transmitter-screen refresh rates**.

| Frame | Data | Target |
|---|---|---|
| Attitude `0x1E` | Pitch, roll, relative yaw | 100 ms / 10 Hz |
| Vario `0x07` | Filtered pressure vertical speed | 100 ms / 10 Hz |
| GPS `0x02` | Latitude, longitude, ground speed, course, MSL altitude, satellites | At most 400 ms / 2.5 Hz; new timed valid fixes only |
| Mode `0x21` | Actual mode, bench/disarm/fault label | 400 ms / 2.5 Hz periodic; prioritized changes at most 10 Hz |

A 100 Hz scheduler sends at most one whole frame per tick under a configurable **240 CRSF bytes/s** budget and 32-byte burst allowance. Groups have independent enable/rate settings. Fairness prevents GPS starvation; UART backpressure retries latest data. This budget does not automatically adapt to ELRS RF settings.

ER8/ELRS separately generates RSSI, link quality, SNR, antenna, RF mode and power statistics. No FC battery telemetry is sent without a validated measurement path. Raw IMUs, PID traces and logs stay off the radio link. No telemetry group has been removed yet.

Actual refresh depends on ELRS packet rate, telemetry ratio, packet format, reception and EdgeTX. **420000 baud is the wired link rate, not wireless telemetry refresh.** See the [full inventory](docs/TELEMETRY.md) before simplifying streams.

For the USB GUI, use Python 3.12 with PySide6-Essentials and pySerial; `numpy` is used by separate spectrum tools/tests:

```text
python -m pip install PySide6-Essentials==6.8.3 pyserial==3.5 numpy
python tools/gui.py --port COM7
```

Replace `COM7` with the board's actual port. The GUI displays attitude, dual-IMU health, pressure, GNSS, RC and outputs, and expires stale data. USB also reports estimator, telemetry-budget, scheduler and SD diagnostics. Magnetic axes are compensated sensor data, not calibrated aircraft heading. Newline-terminated `RESET_STATS` clears scheduler counters. `dfu` (alias `REBOOT_BL`) enters ROM DFU through a clean reset; the GUI also has an **Enter DFU** button. Disarm/idle and post-flight RC recovery gates apply; see [USB firmware updates](docs/DFU.md). See the [feature catalog](docs/FEATURES.md) for tags and fault labels.

## Non-blocking SD logging

`FC_SD_LOGGING=1` is the default. At boot, the logger mounts FAT32 and creates the next unused `FLT00000.BIN` ... `FLT09999.BIN`, reserving **128 MiB contiguously** and closing the filesystem handle before runtime. Existing files are not overwritten. An unusable card disables logging without blocking MANUAL operation. Mounting and allocation can take time at startup.

During runtime a **32 KiB ring** feeds single-sector SDIO DMA writes through a bounded command/card-ready state machine serviced at 4 kHz. The runtime path performs no filesystem calls, allocations or blocking card waits. Longer stalls drop complete new records and count drops rather than waiting in control. Errors/timeouts latch logging off until restart.

| Stream | Rate | Contents |
|---|---|---|
| U4 and U7 independently | 400 Hz each | Sequence/timing, pre-software-filter and filtered/calibrated gyro/accel, calibration state |
| Controller | 100 Hz | Demanded/measured roll/pitch rates, applied commands, throttle, mode, flags |
| Flight state | 50 Hz | Attitude, selected IMU, pressure/altitude/GNSS, RC and output pulses |

Calculated default payload is approximately **40.7 kB/s**, giving about **0.8 seconds** of ring capacity and **53 minutes** per file after overhead. Each sector carries a session ID, sequence and CRC; recovery accepts the consecutive valid prefix and rejects stale/torn tails. Apparent file size stays 128 MiB. Card caches/power loss can still lose data; real-card qualification is outstanding.

```text
python tools/parse_bin_log.py FLT00000.BIN --summary
python tools/parse_bin_log.py FLT00000.BIN -o flight.csv
python tools/export_diagnostics.py FLT00000.BIN --out flight
python tools/analyze_imu.py flight_imu0.csv --out spectrum0.csv
```

The “raw” stream already includes sensor-side filtering and mounting/sign conversion. Reconstructed FIFO timestamps are not exactly synchronized acquisition times. Calibration/spectrum tools do not change firmware parameters automatically. See [logging/recovery](docs/LOGGING.md) and [calibration/tuning](docs/CONTROL_AND_TUNING.md).

## Build and configure

Install Git and PlatformIO Core, then clone with an account that has repository access:

```text
git clone https://github.com/zelveron/STM32FC_v3.git
cd STM32FC_v3
pio run -e v2_bmi270
pio run -e crsf_probe
```

[platformio.ini](platformio.ini) pins `ststm32@19.7.1` and Arduino STM32 framework `4.21200.0`, uses C++17, and takes F407VG startup/linker support from `black_f407vg` with the project's actual GPIO map.

| Environment | Purpose | Result |
|---|---|---|
| `v2_bmi270` (default) | Current dual-BMI270 card, magnetometer disabled | Motor-inhibited bench firmware; `.pio/build/v2_bmi270/firmware.bin` |
| `v2` (optional BMM350) | BMI270 development with magnetometer support enabled | `.pio/build/v2/firmware.bin`; `FC_FLIGHT_ENABLED=0`, both ESCs at minimum |
| `v2_motor_test` | Explicit motor-enabled qualification | `FC_FLIGHT_ENABLED=1`; select only after electrical/motor-disabled checks |
| `v2_flight` | Full motor-enabled flight firmware | Same application as `v2_motor_test`: MANUAL/ASSIST/TKOFF, CH5 arm, CH7 mode, CRSF/USB and SD logging |
| `crsf_probe` | Independent UART4/USB diagnostic | Isolates receiver wiring/protocol issues |
| `native` | Portable module checks | Host executable |
| `sitl` | Simplified aircraft simulation | Host model, not flight qualification |

After electrical repairs and propeller-off acceptance checks, the separate motor-enabled artifact can be built deliberately:

```text
pio run -e v2_flight
```

The flag enables software authorization, not flight readiness. The configured upload protocol is DFU. Building does not flash anything. The current BMI270 card was flashed and readback-verified on 2026-10-08; see [bring-up evidence](docs/ASSEMBLED_BOARD.md).

Configuration is compile-time:

- [src/config/airframe.hpp](src/config/airframe.hpp): output endpoints/reversal/rates, IMU rotation/calibration/filters, telemetry, logging and motor gates.
- [src/control/command_shape.hpp](src/control/command_shape.hpp): assist tuning defaults, limits, trim and command shaping.
- [src/core/telemetry_schedule.hpp](src/core/telemetry_schedule.hpp): telemetry groups and budget logic.
- [src/hal/stm32/board_pins.hpp](src/hal/stm32/board_pins.hpp): board pin assignments.

Persistent/versioned parameters and a parameter-editing ground station remain future work. Hardware, channel, axis or failsafe changes require matching documentation and regression updates.

## Validation

On 2026-09-28, **274 C/C++ behavioral assertions and 17 Python checks passed**. Coverage includes portable modules, real-application fault paths with simulated devices, Bosch BMI270/BMP581 driver models, asynchronous card/logger faults, controllers/telemetry, GUI parsing, log recovery, calibration and simplified flight scenarios.

| ARM build | Static RAM / 131,072 bytes | Flash / 1,048,576 bytes | Result |
|---|---:|---:|---|
| `v2` | 47,592 | 139,560 | PASS |
| `v2_motor_test` | 47,592 | 139,672 | PASS |
| `crsf_probe` | 9,228 | 36,424 | PASS |

All three builds finished without compiler warnings/errors. Static RAM does not measure worst-case stack use or timing. Four 70-second toy-aircraft scenarios include turns, wind, actuator delay/lag, vibration and modest bias; they do not validate actual-aircraft gains. No hardware or flight test is claimed.

Reproduce on Windows with Visual Studio 2022 C++ Build Tools, PlatformIO Core and the Python dependencies above:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/test_host.ps1
python tests/gui_parser.py
python tests/log_tools.py
pio run -e v2 -e v2_motor_test -e crsf_probe
```

Use `-VcVars` for a different Visual Studio installation. Generated binaries/caches are excluded from Git. Captured [host](docs/validation-results/host.txt), [ARM](docs/validation-results/arm-build.txt) and [Python](docs/validation-results/python.txt) results are included. See [validation and physical acceptance](docs/VALIDATION.md) for test scope and remaining checks.

## Roadmap

These are outstanding milestones, not completed features or a release schedule. Autonomy follows board, estimator and actuator qualification.

### 1. Bring up the v2.2 board

- [ ] Inspect the actual assembly; correct VCAP, VREF+ and BMM350 supply/pull-up findings.
- [ ] Resolve input voltage, particularly 6S; qualify receiver/servo/BEC power and startup.
- [ ] Measure clocks/buses, sensor orientation, both IMUs and all actual PWM pins/endpoints.
- [ ] Record per-IMU six-face calibration and motor vibration; choose filters from measurements.
- [ ] Qualify SD DMA with multiple/near-full/fragmented cards, removal, long stalls and power cuts while measuring control timing.
- [ ] Test actual ER8 RF loss/recovery, automatic throttle return, telemetry arrival rates and watchdog-reset outputs.

### 2. Validate and tune the aircraft

- [ ] Verify directions, throws, CG, trim, servo response and ESC behavior, initially with motors disabled.
- [ ] Validate MANUAL, then tune roll/pitch gains, trim, rudder damping, limits and filters through staged tests.
- [ ] Test ASSIST/TKOFF transitions and failures across realistic speeds, turns and gusts.
- [ ] Establish measured operating limits; attitude control alone is not stall/envelope protection.

### 3. Improve estimation and configuration

- [ ] Add versioned persistent parameters with validation and migration/reset behavior.
- [ ] Improve FIFO timestamp alignment, temperature calibration and characterization of IMU failover.
- [ ] Calibrate BMM350 in the aircraft and reject magnetic interference before heading fusion.
- [ ] Add timestamped GNSS velocity/accuracy, receiver configuration and validated navigation fusion; evaluate an EKF when justified.
- [ ] Integrate a calibrated, health-checked airspeed sensor before relying on airspeed/energy control.
- [ ] Add calibrated battery voltage/current monitoring if supported by the hardware.
- [ ] Measure radio bandwidth, then simplify telemetry profiles; export a radio model after versions/switches are fixed.

### 4. Future autonomy

- [ ] Define launch detection, automatic takeoff throttle/pitch/climb sequencing, aborts and pilot override.
- [ ] Develop and validate altitude/airspeed energy control, heading/track control and navigation health.
- [ ] Add loiter, waypoints and RTH with explicit link/navigation failure policies and flight testing.
- [ ] Evaluate automatic landing after navigation and energy control are qualified.
- [ ] Add a documented ground-station protocol, potentially MAVLink, with commands and parameter handling.

## Documentation and source layout

| Guide | Contents |
|---|---|
| [Hardware](docs/HARDWARE_V2.md) | Electrical evidence, power, pins, connectors, outputs and orientation |
| [TX16S / ER8](docs/TX16S_SETUP.md) | EdgeTX mixes, ELRS, telemetry, arming, modes and recovery |
| [Feature catalog](docs/FEATURES.md) | Implemented behavior and limitations |
| [Sensor fusion](docs/SENSOR_FUSION.md) | Dual-IMU rationale, calibration, estimator and faults |
| [Control and tuning](docs/CONTROL_AND_TUNING.md) | Filters, geometry, anti-windup, calibration and tuning |
| [Telemetry](docs/TELEMETRY.md) | Both radio directions, fields, rates and budgets |
| [Logging](docs/LOGGING.md) | DMA state machine, formats, recovery and tools |
| [Validation](docs/VALIDATION.md) | Evidence and physical acceptance procedure |
| [Change review](docs/CHANGE_REVIEW.md) | Migration changes and original review findings |
| [Decisions](docs/DECISIONS.md) | Historical record; later entries supersede earlier behavior |

```text
src/
  config/       Airframe, calibration, output and feature settings
  control/      Attitude/rate controllers, PID, shaping and mixer
  core/         Arming, failsafe, scheduling, health, telemetry and logs
  drivers/      CRSF, GNSS, BMI270, BMP581 and BMM350
  estimation/   IMU preparation, filters, attitude and barometer
  hal/          STM32 access and native test implementation
  modes/        MANUAL, ASSIST and TKOFF
  probe/        CRSF diagnostic
  sitl/         Simplified aircraft/sensor model
lib/            Vendored sensor and SD/filesystem libraries
tests/          Application, device-model and Python regressions
tools/          Host runner, USB GUI, log/calibration/spectrum tools
docs/           Guides, evidence, validation captures and history
```

## Third-party code and licensing

Vendor notices are retained. BMI270 and BMM350 include licenses and pinned provenance in `lib/*/UPSTREAM.md`; BMP5, FatFs and STM32SD retain their existing notices. STM32SD includes GPLv3-marked source. This repository assigns no new blanket license to those components or the original project; a project-wide licensing decision is still outstanding.
