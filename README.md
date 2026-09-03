# STM32FC — Fixed-Wing Flight Controller

Firmware and host tools for a custom **STM32F407VET6** flight controller board,
built to fly a **1.5 m 3D-printed twin-EDF Boeing 787** model aircraft.

> **This README is written for future AI coding agents as much as for humans.**
> It records every gotcha learned the hard way — marginal solder joints, DFU
> quirks, sensor protocol traps, vendored-library patches. Read
> [Known issues](#known-issues--lessons) before touching code.
> See `CLAUDE.md` for architecture rules and hard constraints.

---

## Project status

**Today the firmware is a working sensor streamer, not a flight controller.**
It reads three sensors, streams tagged CSV over USB CDC, logs to SD, and drives
a tkinter GUI on a Raspberry Pi 5. Everything below the "Flight controller" line
is not yet built.

| Capability | Status |
|---|---|
| BMI323 IMU read | working — **bit-bang SPI only, see blocker #1** |
| BMP581 pressure/temp/altitude | working |
| u-blox GNSS | working (NMEA @ 9600) — no fix indoors |
| SD logging | working (CSV @ 50 Hz) |
| USB CDC + GUI | working — **bench scaffolding, removed before flight** |
| — *flight controller* — | |
| CRSF RC input | not started |
| Servo / ESC PWM output | not started |
| Scheduler, params, binary log | not started |
| Attitude estimation (flight-grade) | basic roll/pitch/yaw exists, **not flight-grade** |
| MANUAL / ASSIST / AUTO modes | not started |

Removed since the streamer era: the **ALS31300 Hall sensor** (see
[Known issues](#known-issues--lessons)) and all sensor **enable-pin** GPIO
(no sensor has a power-gate line; everything is always on).

---

## Target aircraft and radio

- Airframe: 1.5 m 3D-printed Boeing 787, twin EDF
- 2× 120 A ESC
- 6 servos: 2 aileron, 2 elevator, 1 rudder, 1 nosewheel
- TX: RadioMaster TX16S MK3 (EdgeTX)
- RX: RadioMaster ER8 (ExpressLRS), CRSF @ 420000 baud

### Planned flight modes

Selected by one three-position transmitter channel.

1. **MANUAL** — direct passthrough. Must work even if the IMU has faulted.
2. **ASSIST** — IMU roll/pitch angle stabilisation with angle limits
   (equivalent to ArduPlane FBWA / Spektrum SAFE).
3. **AUTO** — IMU + GPS + baro. Altitude and track hold first, then loiter,
   then waypoints and RTL.

Autopilot only ever engages on the pilot's switch. Sensor faults demote
downward (AUTO → ASSIST → MANUAL), never upward, and every demotion is
announced over CRSF telemetry.

---

## Hardware pin map (verified)

| Device | Signal | STM32 pin | Notes |
|---|---|---|---|
| BMI323 | SCK | PA5 | SPI mode 0, currently bit-bang (~200 kHz) |
| BMI323 | SDO/MISO | PA6 | |
| BMI323 | SDI/MOSI | PA7 | |
| BMI323 | CS | PA4 | |
| BMP581 | SCL | PB6 | I2C1 |
| BMP581 | SDA | PB7 | I2C1, 7-bit addr **0x47** |
| u-blox | TX (MCU) | PA9 | USART1_TX — PA9 is **TX-only** on F407 |
| u-blox | RX (MCU) | PA10 | USART1_RX — PA10 is **RX-only** |
| SD card | D0..D3, CK, CMD | PC8..PC12, PD2 | SDIO |
| USB CDC | D+/D− | PA12/PA11 | native USB, `SerialUSB` |

No device has an enable/power pin. All sensors are powered from the board
regulators and are always on.

### Reserved for flight I/O — do not reallocate

| Function | Pins |
|---|---|
| Servo/ESC 1–4 | TIM4 CH1–4 — PD12, PD13, PD14, PD15 |
| Servo/ESC 5–8 | TIM1 CH1–4 — PE9, PE11, PE13, PE14 |
| CRSF | USART2 (PA2/PA3), fallback USART3 (PB10/PB11) |

**Output channel map:** 1 aileron L · 2 aileron R · 3 elevator L ·
4 elevator R · 5 rudder · 6 nosewheel · 7 ESC L · 8 ESC R.

The nosewheel gets its own channel. It is **not** mechanically Y-cabled to the
rudder — it needs a separate gain, a gain that falls off with ground speed, and
full centring once airborne.

Servos should run at **200–333 Hz**, not 50 Hz, driven from timer compare
registers directly. Do not use `analogWrite`.

---

## Blockers before flight code

### 1. BMI323 bit-bang SPI must be fixed in hardware

The BMI323's solder joints are **marginal**. Hardware SPI fails even at 1 MHz
(chip ID reads `0xFF` = MISO floating). It currently only works via the
bit-bang implementation in `src/drivers/bmi323.cpp` (`bb_xfer()`):

- `bb_xfer()` — bit-bangs one byte (mode 0, `hal::delay_us(2)` per edge)
- `spi_read()` / `spi_write()` — CS + bit-bang, wired to the Bosch driver
- `raw_chip_id()` — raw reg 0x00 read (`0x43` = present)

**This is disqualifying for flight.** The Bosch core reads 26 data bytes + a
dummy + the address per sample = 28 byte-transfers. At bit-bang speed one
accel+gyro read costs **~1.1–1.7 ms of blocking CPU** (the old README's
"~500 µs" figure assumed only 6+6 data bytes and was wrong). Flight control
needs 1 kHz sampling — one read already exceeds a 1 ms tick budget by itself.
Hardware SPI at 10 MHz does the same read in ~10 µs, and with DMA it costs
essentially nothing.

The failure signature (bit-bang OK at 200 kHz, hardware SPI fails at 1 MHz) is a
textbook resistive cold joint: the weak connection plus pin capacitance forms an
RC filter that smears edges.

**Fix:**
1. Reflow VDD, VDDIO, GND and especially **SDO** with fresh flux.
2. Check the init does a dummy read (CS falling edge) to latch the BMI3xx into
   SPI mode — part of the problem may not be solder at all.
3. If reflow fails, hot-air the chip off and replace it. It costs a few euros.
4. **Verify before proceeding:** chip ID `0x43` at 10 MHz, then several million
   reads with zero mismatches.

Do **not** replace the bit-bang code with hardware SPI until this passes.

### 2. Sampling rates are datalogger rates

| Sensor | Current | Target | Why |
|---|---|---|---|
| BMI323 | 200 Hz ODR, ~100 Hz sampled | **≥1.6 kHz ODR, internal filter ON, FIFO + DMA, decimate to 1 kHz** | Two EDFs at 30–45k RPM put energy above 1 kHz. Sampling slower **aliases** it down into the control band as phantom motion the PIDs will chase. |
| BMI323 accel | ±4 g | **±8 g or ±16 g** | Launch, gusts and landing clip ±4 g, corrupting attitude exactly when it matters. |
| BMP581 | ~10 Hz | **≥50 Hz** | Climb rate is d(altitude)/dt. 10 Hz gives an unusably laggy derivative. |
| u-blox | NMEA 9600, 1 Hz | **UBX binary, 115200, 5 Hz, `UBX-NAV-PVT`** | One message gives position, NED velocity, fix type, sats and accuracy. NED velocity is what the nav filter needs. |
| SD log | CSV, 50 Hz, 1-bit @ 4 MHz | **binary, ring buffer, 512-byte aligned, 4-bit** | `sprintf` of 15 floats costs hundreds of µs. First flights need raw gyro at 500–1000 Hz for FFT. |

### 3. ~~Everything lives in one file~~ — done

`src/main.cpp` has been split into the `CLAUDE.md` layered layout: `hal/`
(interface + `stm32/` and `native/` backends), `drivers/` (bmi323, bmp581,
ublox), `estimation/ahrs`, `core/` (bench `usb_stream` + `sd_csv_log`),
`main_stm32.cpp`. `[env:native]` builds the portable layers. The bit-bang IMU
transfer still blocks (blocker #1) — the split does not change that.

---

## Debugging without a debugger

There is **no ST-Link / SWD** on this project. Flashing is USB DFU; the only
debug channel is USB CDC. Three consequences shape every design:

1. **All logging must be non-blocking.** `SerialUSB.write()` on the STM32
   Arduino core spins up to `USB_CDC_TRANSMIT_TIMEOUT` (3 ms) per call when the
   host is connected but not draining, and can emit torn lines. Every USB write
   goes through a bounded wrapper that drops whole lines when the queue is full
   and never spins.
2. **Software jump to the DFU bootloader** — a CDC command that jumps to system
   memory, so BOOT0 does not have to be toggled by hand every flash cycle.
   *(planned — early task)*
3. **Hard fault handler that survives reset** — stash the stacked PC, LR and the
   CFSR/HFSR into RTC backup registers, print them on the next boot. Without
   this a crash is a silent reboot. *(planned — early task)*

---

## Build

```bash
cd ~/Desktop/STM32FC_Claude
pio run
```

Board `black_f407ve`, framework Arduino (`Arduino_Core_STM32`), USB CDC enabled
via `-D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC`.
Output: `.pio/build/black_f407ve/firmware.bin`.

We stay on the Arduino framework for now. It sits on top of STM32Cube HAL, so
timing-critical drivers can call `HAL_*` / `LL_*` directly without throwing away
the working vendored SD stack.

---

## Flash

### Recommended: ST-Link over SWD

Get an ST-Link V2 (clones are ~€5). It gives real breakpoints and live variable
watch through the **Cortex-Debug** VS Code extension, plus SWO/RTT printf that
doesn't fight the USB CDC stream. This is the single best time investment in the
whole project. *(not yet acquired — see "Recommended hardware additions")*

### Current: DFU (STM32 ROM bootloader)

Flashes over USB DFU (`0483:df11`). **Enumeration is flaky on the Raspberry
Pi** — `dfu-util -l` often shows nothing (kernel logs
`device descriptor read/64, error -110`).

1. Set **BOOT0 = 1** and press **reset** (keep BOOT0 high).
2. If `dfu-util -l` is empty, unplug and replug USB while BOOT0 stays high.
   Confirm the device shows `0483:df11`, not `0483:5740` (5740 = the running
   app, meaning BOOT0 is not actually high).
3. Flash:
   ```bash
   dfu-util -a 0 -s 0x08000000:leave -D .pio/build/black_f407ve/firmware.bin
   ```
4. Set **BOOT0 = 0** and press **reset** to run.

---

## Serial protocol (current — bench only)

USB CDC, 115200 baud, tagged CSV. The GUI parses by leading tag. **This whole
path is bench scaffolding and will be deleted once the restructure is
validated.** The flying aircraft has no USB connection.

```
BMP,<pressure_hPa>,<temp_C>,<altitude_m>
BMI,<acc_x>,<acc_y>,<acc_z>,<gyr_x>,<gyr_y>,<gyr_z>     (g, deg/s)
ATT,<roll_deg>,<pitch_deg>,<yaw_deg>
GPS_STAT,<fix>,<sats>,<time_HH:MM:SS>,<speed_kmh>       (1 Hz, even without fix)
GPS,<lat>,<lon>,<alt_m>,<sats>,<fix>,<time>,<speed_kmh> (only with a fix)
GPS_RAW,<last NMEA sentence>                            (1 Hz debug)
GPS_DBG,<rx_bytes>,<baud>,<locked>                      (2 s debug)
GPS_FIRST,<len>,<hex boot bytes>                        (3 s debug)
BMI_STATUS,0|1   BMI_RAW,0xNN   BMP_STATUS,0|1
SD_STATUS,1|<file>   SD_DBG,<ok>,<file>,<usb_log_drops>  (5 s debug)
```

`usb_log_drops` counts whole telemetry lines dropped because the USB host was
not draining the CDC queue fast enough. Non-zero is expected when the GUI is
busy; it means the drop-on-full logger did its job instead of stalling the
loop.

GPS speed comes from `$GxRMC` knots × 1.852 → km/h. GPS time is **UTC** from
`$GxGGA` / `$GxRMC`.

Flight telemetry to the pilot is a separate path: **CRSF** back to the TX16S so
EdgeTX can announce mode, battery and GPS status audibly in the air.

---

## GUI

```bash
python3 tools/gui.py                      # auto-detect /dev/ttyACM0
python3 tools/gui.py --port /dev/ttyACM0
```

Sections: BMI323 status, BMP581 (p/t/alt), BMI323 (accel/gyro), u-blox GNSS
(position/alt/sats/fix/time/speed), attitude (roll/pitch/yaw + artificial
horizon). Auto-reconnects, has a data watchdog. *(the GUI still references an
ALS/heading field that the firmware no longer emits — harmless, will be pruned
with the rest of the bench tooling.)*

---

## SD card logging

SDIO, FatFs. Firmware writes `FLTxxxxx.CSV` (next free index each boot):

```
t_ms,ax,ay,az,gx,gy,gz,roll,pitch,yaw,press_hPa,temp_c,alt_m,gps_time,gps_sats,gps_speed_kmh
```

~50 Hz, flushed every 1 s. Vendored `lib/STM32SD/` and `lib/FatFs/` contain
critical fixes — **do not regenerate them from upstream** (see Known issues #2).

This CSV format is bench-only and will be replaced by binary logging. Once USB
is removed, the SD log becomes the primary flight recorder.

---

## Known issues & lessons

**Read this before coding. Do not "clean up" anything here without asking.**

1. **BMI323 marginal solder → bit-bang SPI only.** See blocker #1 above. Reflow
   VDD / VDDIO / GND / SDO before switching to hardware SPI.
2. **SD hang.** The stock STM32 HAL SD timeout was `100000000 ms` (≈27.8 h),
   which looked like an infinite hang. Fixed in vendored code:
   `lib/STM32SD/src/bsp_sd.h` — `SD_DATATIMEOUT` = 2000;
   `lib/STM32SD/src/bsp_sd.c` — `SD_CLK_DIV` = 10U (4 MHz), `SD_BUS_WIDE` = 1B.
   `lib/FatFs/` **must** keep its `ffsystem/` folder or the build fails.
   *Note: 1-bit @ 4 MHz costs ~8× bandwidth. Revisit 4-bit mode once logging is
   binary and DMA-driven.*
3. **ALS31300 removed.** It used to live on PB8=SCL / PB9=SDA (software bit-bang
   I2C, addr `0x60`). It is gone from the flight build and from the firmware:
   its range is hundreds of gauss while Earth's field is ~0.5 G, so it cannot
   serve as a magnetometer, and its bit-bang I2C blocked the loop. A mystery
   device at 0x7E on that bus ACKed but ignored register commands — also gone.
   PB8/PB9 are now free (they are I2C1's alternate pins; I2C1 itself is on
   PB6/PB7 for the BMP581). Do not re-add the ALS.
4. **u-blox must be crossed-wired** (module TX → PA10, module RX → PA9). USART1
   has no remap on the F407: PA9 is TX-only, PA10 is RX-only. Wired straight
   (TX→TX, RX→RX) the MCU receives nothing — proven by edge-counting both pins.
   Without an antenna it still sends `$GPTXT` boot messages and 1 Hz NMEA with
   an empty fix; time fills in as soon as any satellite is heard. Indoors you
   may see `sats=0` forever.
5. **DFU enumeration is flaky** — see Flash section. `:leave` makes the board run
   the app immediately after flashing.
6. **Serial capture contention.** `tools/gui.py` and any `cat /dev/ttyACM0` open
   the same CDC port and split the stream. Kill the GUI before a clean `cat`
   capture. `tools/capture.py` survives USB re-enumeration.
7. **Boot prints are lost** unless the firmware waits for the USB host. `setup()`
   has a bounded `while (!SerialUSB)` wait for exactly this reason.
8. **GPS baud auto-detect** locks onto the first baud producing a
   checksum-valid NMEA sentence — it does *not* wait for a satellite fix — then
   stops cycling the 9 candidate bauds.
9. **No sensor enable pins.** Every sensor is always powered. The old firmware
   drove PB11 / PE11 / PC14 as "enables"; those were removed (PB11 and PE11 are
   reserved flight-I/O pins). Do not add sensor power-gate code.

---

## Roadmap

Phases are ordered. Do not skip ahead — steps 1 and 5 of Phase 0 are the ones
people skip and then spend months confused about why the aircraft oscillates.

**Phase 0 — unblock**
1. Reflow the BMI323, verify hardware SPI at 10 MHz with zero errors.
2. Get ST-Link + Cortex-Debug working in VS Code.
3. Split `main.cpp` into the `CLAUDE.md` layout, add `[env:native]`.
4. Rewrite the BMI323 driver: hardware SPI, DMA, FIFO, ≥1.6 kHz ODR, ±8 g.
5. **Vibration test.** Restrain the airframe, spool the EDFs to 50%, log raw
   gyro at 1 kHz, FFT it. Do this *before* tuning a single gain — it tells you
   whether you need soft mounting, better fan balancing, or a notch filter.
6. CRSF driver: DMA circular RX + UART IDLE interrupt, generic frame parser,
   link statistics, telemetry TX.
7. Scheduler with per-task timing, binary logger, flash parameter store, IWDG.
8. Software DFU-bootloader jump over CDC; hard-fault handler into RTC backup regs.
9. Define and verify the sensor axis → body-frame mapping (CLAUDE.md).

**Phase 1 — estimation.** Gyro bias calibration, 6-point accel calibration,
anti-alias + low-pass filtering, complementary AHRS with accel gating, BMP581
ground reference, UBX parser.

**Phase 2 — manual mode.** `RC_Channel` / `SRV_Channel` abstractions, mixer with
flaperon and differential-thrust scaffolding, arming state machine, basic
failsafe. *Exit: flying through the FC in MANUAL feels identical to direct
RX-to-servo.*

**Phase 3 — SITL.** Desktop 6DOF model, scripted stick input, CSV state output.
Build this early — it is what lets an AI agent verify its own changes instead of
guessing.

**Phase 4 — ASSIST.** PID library with anti-windup and D-term filtering, rate
loops, angle loops with limits, turn compensation, sideslip damping, bumpless
transitions, pilot override.

**Phase 5 — position estimation.** Baro+accel vertical complementary filter,
GPS+accel horizontal filter, per-sensor health monitoring.

**Phase 6 — AUTO: cruise.** Altitude and ground-track hold, stick nudging.

**Phase 7 — AUTO: navigation.** L1 guidance, loiter, waypoints, RTL.

Realistic budget for Phases 0–6: **4–6 months of evenings.** That is normal.

---

## Recommended hardware additions

1. **Pitot-static airspeed sensor** (DLVR-L05D or SDP33; MS4525DO is cheaper but
   drifts). Biggest single improvement to auto-mode viability. Control gains
   scale with dynamic pressure, and TECS needs a real minimum-airspeed floor to
   keep the aircraft from stalling in a climbing turn. Mount the probe on the
   nose or a wing boom, well clear of the EDF inlets.
2. **ST-Link V2** — see Flash section.
3. **Dedicated 6–8 A switching BEC for the servos.** Do **not** parallel the two
   ESC BECs — cut one red wire, or better, both. Six servos on a jet this size
   pull 5–8 A on a gust. Run the FC and receiver from a separate clean rail. A
   brownout in ASSIST mode is a crash.
4. **Soft-mount the FC** on vibration-damping gel or O-ring standoffs, and
   **balance the EDF rotors**. Worth an evening.
5. **GPS antenna placement.** Keep the module far from the two 120 A ESCs and
   their battery leads — 240 A of switching noise destroys fix quality. Elevate
   it, put a ground plane under it, twist the ESC power leads.
6. Optional: upgrade to an **M9N/M10** GNSS module (10 Hz, multi-constellation)
   and add a magnetometer (**RM3100** good, **IST8310** adequate) if you want a
   pre-takeoff heading reference. Compass-less operation using GPS ground course
   is acceptable for fixed-wing, but heading is only valid once moving and
   crosswind means ground course ≠ heading.

---

## Git

Remote: `https://github.com/zelveron/STM32FC` (branch `main`).
Author identity `zelveron <zelveron@users.noreply.github.com>` is set
repo-locally. Push with `git push origin main` — credentials are cached on the Pi.
