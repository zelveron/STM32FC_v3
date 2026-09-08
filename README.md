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

### 1. ~~BMI323 bit-bang SPI~~ — moved to hardware SPI

**History:** the BMI323's solder joints were marginal — hardware SPI failed
(chip ID `0xFF`), so the driver bit-banged SPI at ~200 kHz. One accel+gyro
read cost **~1.4 ms of blocking CPU** (measured), which caps sampling at
~700 Hz and burns 70–98% of the core. Disqualifying for flight.

**Now:** after solder rework, the `imu_probe` firmware verified hardware SPI
at 10 MHz: **0 chip-ID errors and 0 comm failures over 1.4 million reads**.
The link is solid. (A residual ~0.04% of samples showed large accel deltas —
that is register *tearing* from the probe polling 8× faster than the ODR with
no data-ready gate, not a bus fault: chip-ID on the same wire was perfect, the
rate was flat across 1/4/8/10 MHz, and the rework didn't change it.)

`src/drivers/bmi323.cpp` now uses **SPI1 (PA5/6/7), CS PA4, ~5.25 MHz,
blocking** — a 28-byte burst is ~45 µs. Reads are **data-ready gated** so there
is no tearing. Config: 1600 Hz ODR both sensors, internal filter on, ±8 g accel,
2000 dps gyro, high-perf mode. Bit-bang code and `hal::gpio` bus shim are
deleted.

**Still to do:** FIFO + DMA (`hal::spi_xfer_async`) to take the read cost from
~45 µs to ~1 µs and decouple it from the ODR; a raw-sample spike filter.
Hardware-flash verification of this driver is pending (build is green).

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

1. Set **BOOT0 = 1** and press **reset** (keep BOOT0 high) — or send `REBOOT_BL`
   over the USB CDC console (no button needed; see below).
2. If `dfu-util -l` is empty, unplug and replug USB while BOOT0 stays high.
   Confirm the device shows `0483:df11`, not `0483:5740` (5740 = the running
   app, meaning BOOT0 is not actually high).
3. Flash:
   ```bash
   dfu-util -a 0 -s 0x08000000:leave -D .pio/build/black_f407ve/firmware.bin
   ```
4. Set **BOOT0 = 0** and press **reset** to run.

**Pi xHCI gets stuck on the live re-enumeration.** A *cold* plug into DFU
enumerates fine, but the CDC→DFU transition (`REBOOT_BL`, or any app→bootloader
jump) reliably wedges the Pi port with `error -110`, and it stays wedged across
replugs on that port. Unbinding/rebinding the `xhci-hcd.N` platform driver does
**not** clear it. What works: move the board to a physical port on the *other*
xHCI controller, or reboot the Pi. This is a Pi-host bug, not firmware.

**USB CDC console commands** (newline-terminated, into `/dev/ttyACM0` @ 115200):
`RESET_STATS` (zero the scheduler counters for a clean `SCHED` measurement),
`REBOOT_BL` (jump to the DFU bootloader), `SIM_FLYING 0|1` (bench: force the
`s_flying` latch to exercise the in-flight logging / integrator path).

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
   the app immediately after flashing. The Pi xHCI also wedges (`error -110`) on
   the CDC→DFU *live* transition and stays wedged on that port across replugs;
   fix is a different physical port (other controller) or a Pi reboot — a
   driver unbind/rebind does not clear it.
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
10. **SD writes are still blocking.** `disk_write` busy-waits on the card; the
    fitted card returns most 512 B writes in 1-3 ms but occasionally stalls
    ~14 ms on its flash-program cycle. This is the one thing keeping the 400 Hz
    loop from being hard-real-time (`worst_pass_us` ~15 ms, rare, dt-corrected,
    recovered by the scheduler's critical re-service). `task_log_flush` batches
    to 1 Hz to minimise how often it hits. Real fix: a non-blocking SD write
    path (`HAL_SD_GetCardState` polled per tick). A high-endurance / industrial
    card also shrinks the tail. See `docs/DECISIONS.md` 2026-09-08.

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
