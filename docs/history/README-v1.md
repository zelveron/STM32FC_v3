> Archived v1 document. Hardware instructions and flight claims do not apply to v2.2. See ../../README.md.

# STM32FC_v2 — Fixed-Wing Flight Controller

Firmware and host tools for a custom **STM32F407VET6** flight controller board,
built to fly a **1.5 m 3D-printed twin-EDF Boeing 787** model aircraft.

Repo: `github.com/zelveron/STM32FC_v2` (branch `main`).

> **This README is written for future AI coding agents as much as for humans.**
> It records every gotcha learned the hard way — marginal solder joints, DFU
> quirks, sensor protocol traps, vendored-library patches, host-USB bugs. Read
> [Known issues](#known-issues--lessons) before touching code.
> See `CLAUDE.md` for architecture rules and hard constraints, and
> `docs/DECISIONS.md` for the reasoning behind every design choice (append-only).

---

## Project status

The project started as a **sensor streamer** and has been converted, in the
CLAUDE.md-mandated order, into a working flight controller. MANUAL mode has been
flown / bench-verified on hardware; ASSIST and TKOFF are built and verified in
SITL but **not yet flight-tested**. Position estimation and AUTO are not started.

| Area | Status |
|---|---|
| BMI323 IMU | **hardware SPI1** ~5.25 MHz, DRDY-gated, ±8 g / 2000 dps, sampled 200 Hz |
| BMP581 baro | I²C1, non-blocking poll @ 50 Hz, ground-referenced AGL + climb rate |
| u-blox NEO-7M | NMEA @ 9600, 1 Hz, auto-baud (UBX/5 Hz is Phase 5) |
| CRSF RC input | ER8 on USART3, 420000 8N1, 16-ch + link stats |
| CRSF telemetry out | attitude / vario / GPS / flight-mode → TX16S |
| Servo / ESC PWM | 8 channels, TIM4 + TIM1, 333 Hz |
| Cooperative scheduler | fixed task table, critical-priority, per-task DWT timing, `worst_pass ≈ 15 ms` |
| Binary SD log | `FLTxxxxx.BIN`, 16 KB ring, CRC-16 framed, ~50 Hz |
| Estimation | quaternion gated-Mahony AHRS, gyro-bias cal, baro ground ref |
| Control | anti-windup PID → rate loop → angle loop → mixer, turn compensation |
| MANUAL mode | **flown / bench-verified** — passthrough, arming, failsafe, mode-req |
| ASSIST mode (FBWA) | built, SITL-verified (`--assist-check`), **not flight-tested** |
| TKOFF mode | built, SITL-verified (`--takeoff`), **not flight-tested** |
| SITL 6DOF sim | `[env:sitl]`, self-checking (`--check` / `--assist-check` / `--takeoff`) |
| Native unit tests | `[env:native]`, 80+ assertions |
| Host tools | `gui.py` monitor, `plot_log.py` flight-log plotter, `parse_bin_log.py` |
| — *not started* — | |
| Sensor axis → body-frame verification | **mandatory before an ASSIST/TKOFF flight** |
| `core/params` flash parameter store | blocks in-flight tuning + accel cal |
| 6-point accel calibration | needs `core/params` |
| UBX-NAV-PVT parser, nav filter, TECS, L1, AUTO | Phase 5+ |
| Non-blocking SD write path | the one thing keeping the loop from hard-real-time |
| Hard-fault handler → RTC backup regs | a crash is currently a silent reboot |

Removed since the streamer era: the **ALS31300 Hall sensor** and all sensor
**enable-pin** GPIO (no sensor has a power-gate line; everything is always on).

---

## Target aircraft and radio

- Airframe: 1.5 m 3D-printed Boeing 787, twin EDF
- 2× 120 A ESC
- 6 servos: 2 aileron, 2 elevator, 1 rudder, 1 nosewheel
- TX: RadioMaster TX16S MK3 (EdgeTX)
- RX: RadioMaster ER8 (ExpressLRS), CRSF @ 420000 baud

### Flight modes

Selected by one three-position transmitter channel (ch7): **low = MANUAL,
mid = ASSIST, high = TKOFF**.

1. **MANUAL** — direct passthrough. Must work even if the IMU has faulted. The
   fallback for anything that goes wrong.
2. **ASSIST** — FBWA: stick commands a clamped attitude *angle* (roll ±40°,
   pitch ±26°), angle loop → rate loop → mixer. Turn-compensation pitch-up FF
   while banked. Centre stick = wings level, hold level pitch.
3. **TKOFF** — roll wing-leveller only (stick → ±10° bank, for a crosswind
   wing-low), **pitch / yaw / throttle fully manual** so nothing fights the
   elevator during rotation and climb-out. Take off in TKOFF, then drop to
   ASSIST once settled.
4. **AUTO** — IMU + GPS + baro. Altitude and track hold first, then loiter,
   then waypoints and RTL. *Not built; no switch slot yet (TKOFF took the
   high position).*

The stabilised modes (ASSIST, TKOFF) engage only on the pilot's switch, only
after the gyro-bias cal completes, and their rate-loop integrators only run once
"flying" is latched (armed **and** throttle > 75 % **or** GPS speed > 8 m/s —
so they never wind against a stationary airframe). Sensor faults demote
downward (AUTO → ASSIST → MANUAL), never upward, latched for the flight, and
announced over CRSF telemetry (`FM` shows `!LOCK`).

---

## Hardware pin map (verified)

| Device | Signal | STM32 pin | Notes |
|---|---|---|---|
| BMI323 | SCK / MISO / MOSI / CS | PA5 / PA6 / PA7 / PA4 | SPI1, mode 0, ~5.25 MHz, blocking, DRDY-gated |
| BMP581 | SCL / SDA | PB6 / PB7 | I²C1, 7-bit addr **0x47** |
| u-blox | TX (MCU) / RX (MCU) | PA9 / PA10 | USART1 — PA9 is **TX-only**, PA10 **RX-only** (see #4) |
| CRSF RX | RX (MCU) / TX (MCU) | PB11 / PB10 | USART3, 420000 8N1, full-duplex (RC in + telem out) |
| SD card | D0..D3, CK, CMD | PC8..PC12, PD2 | SDIO, 1-bit @ 4 MHz (see #2) |
| USB CDC | D+ / D− | PA12 / PA11 | native USB, `SerialUSB` |

No device has an enable/power pin. All sensors are powered from the board
regulators and are always on.

### Reserved for flight I/O — do not reallocate

| Function | Pins |
|---|---|
| Servo/ESC 1–4 | TIM4 CH1–4 — PD12, PD13, PD14, PD15 |
| Servo/ESC 5–8 | TIM1 CH1–4 — PE9, PE11, PE13, PE14 |
| CRSF | USART3 (PB10/PB11) — **wired here**; USART2 (PA2/PA3) is the documented alt |

**Output channel map:** 1 aileron L · 2 aileron R · 3 elevator L ·
4 elevator R · 5 rudder · 6 nosewheel · 7 ESC L · 8 ESC R. Left/right surface
opposition is a per-channel `SrvChannel.reversed` flag; the mixer sends both
sides the same signed command.

The nosewheel gets its own channel (**not** Y-cabled to the rudder) — separate
gain, falls off with ground speed, centred once airborne (falloff/centring TBD).

Servos run at **333 Hz** from timer compare registers (`HardwareTimer`), not
`analogWrite`.

---

## Firmware architecture

Layered, per `CLAUDE.md`. Everything except `hal/stm32/` and the two bench
files compiles on the desktop (`[env:native]`), which is what lets an agent
verify its own changes.

```
src/
  hal/            hardware abstraction — the ONLY layer touching STM32/Arduino
    hal.hpp         interface implemented by both backends
    stm32/          real hardware (may call HAL_*/LL_* directly)
    native/         desktop backend for tests and SITL
  drivers/        bmi323, bmp581, ublox, crsf         (depend on hal only)
  estimation/     ahrs (gated Mahony), imu_prep (bias cal + LPF), baro_alt
  control/        pid, rate_ctrl, attitude_ctrl, mixer, rc_channel, srv_channel
  modes/          mode.hpp + mode_manual, mode_assist, mode_takeoff
  core/           scheduler, failsafe, arming, log_ring, log_frame, sd_bin_log
                  + usb_stream  (BENCH scaffolding, Arduino-coupled — deleted with USB)
  sitl/           aircraft (6DOF), sensors (synthetic)
  main_stm32.cpp  composition root: brings up hal+drivers+estimation, registers
                  the scheduler tasks, wires the mode manager, runs sched::run()
  main_native.cpp [env:native] entry — smoke test + all unit assertions
  main_sitl.cpp   [env:sitl] entry — real control chain vs the 6DOF model
```

### Build envs (`platformio.ini`)

| Env | Purpose |
|---|---|
| `black_f407ve` | the firmware (DFU upload) |
| `native` | portable-layer unit tests — proves `core/`/`estimation/`/`control/`/`modes/` stay Arduino-free |
| `sitl` | desktop 6DOF simulator |
| `crsf_probe`, `imu_probe` | diagnostic firmwares (not flight) |

```bash
pio run   -e black_f407ve            # cross build  -> .pio/build/black_f407ve/firmware.bin
pio run   -e native  && .pio/build/native/program        # unit tests + smoke
pio run   -e sitl                                         # build the simulator
.pio/build/sitl/program --check          # trim + control signs + no-departure
.pio/build/sitl/program --assist-check    # ASSIST holds/returns/no-oscillation/bumpless
.pio/build/sitl/program --takeoff         # TKOFF: roll levelled, pitch passes through
```

**Before claiming a change works: build it and run the checks, and paste the
real output.** For control changes, show the SITL trace.

---

## Scheduler

Single-threaded cooperative, no RTOS. Fixed task table; each pass runs every due
task once. `bmi` and `control` are **critical** — dispatched at the top of every
pass and re-serviced immediately after any non-critical task that ran > 500 µs,
so a slow logger/debug/SD write delays the rate loop by at most one such task,
not the sum. Per-task min/mean/max runtime and an overrun count via the DWT
cycle counter, streamed on a 1 Hz `SCHED,` line.

| Hz | Task |
|---|---|
| 400 | `control` — CRSF → arming/failsafe → mode manager → mixer → PWM |
| 200 | `bmi` — IMU read, `imu_prep` (bias-correct + LPF), AHRS update |
| 100 | `crsf` — drain USART3, parse |
| 50 | `gps`, `bmp`, `log` (pack a frame into the ring) |
| 20 | `stream` — the USB tagged-CSV echo |
| 10 | `crsf_tx` (telemetry), `cmd` (USB console) |
| 5 | `log_flush` — ring → SD (blocking lives here, isolated) |
| 2 | `debug` — low-rate GPS/SD/EST/MODE lines |
| 1 | `bmi_probe`, `sched` (the SCHED report) |

Measured `worst_pass_us ≈ 15 ms`, entirely one blocking SD sector write (see
#10). Everything else per task is < 1.1 ms. IWDG support exists
(`sched::set_watchdog_ms`) but is **not yet enabled** — it turns on once the
watchdog-reset path latches MANUAL.

---

## Estimation

- **AHRS** (`estimation/ahrs`) — quaternion **gated complementary filter**
  (Mahony explicit, `Kp = 1.0`, `Ki = 0.05`). The accel correction is weighted
  by a magnitude gate (`|a|` within 0.05 g of 1 g) **and** a body-rate gate
  (full trust < 5 dps, zero > 25 dps): a coordinated turn sits near 1 g so the
  magnitude gate alone won't reject it, and without centripetal compensation
  the accel drags the estimate toward level — so it coasts on the gyro whenever
  it's rotating. **Known limit:** sustained-turn bank still under-reads a few
  degrees; the fix is GPS-velocity centripetal compensation with the Phase-5
  nav filter.
- **No magnetometer.** Yaw is gyro-integrated (relative to power-on), drifts
  slowly, **is not a compass**. A GPS-aided heading comes with Phase 5.
- **Gyro-bias calibration** (`estimation/imu_prep`) — accumulates from boot,
  completes after ~4 s of the gyro span staying under a threshold. On the
  rising edge of `bias_ready`, while disarmed, a **control-surface sweep**
  (aileron → elevator → rudder) tells the pilot the cal is done without a
  screen.
- **Baro altitude** (`estimation/baro_alt`) — ground pressure latched while
  disarmed, frozen on arm; reports AGL. Climb rate is a 0.7 Hz-filtered dAGL/dt.

---

## Control

- **`control/pid`** — feedforward + P + I + filtered D-on-measurement.
  Anti-windup: integrator clamp `±i_max` plus a conditional freeze while the
  output is railed in the same direction. `preset_integrator()` for bumpless
  mode entry. `set_integrator_enabled()` freezes accumulation on the ground.
- **`control/rate_ctrl`** — three axis PIDs on the gyro; normalized surface
  command out.
- **`control/attitude_ctrl`** — angle error → desired body rate (P, rate-
  limited) + turn-compensation pitch-up FF `∝ tan(φ)·sin(φ)·g/V` while banked
  (banking loses vertical lift; without this the nose drops every turn and I
  fights it).
- **`control/mixer`** — `mix_manual` maps roll/pitch/yaw/throttle to the 8
  channels. Flaperon / differential-thrust / nosewheel-gain scaffolding present,
  most gains at defaults.
- **ASSIST** = stick → clamped angle → `attitude_ctrl` → `rate_ctrl` → mixer,
  with a bumpless `enter()` preload and the flying-latch integrator gate.
- **TKOFF** = the ASSIST roll loop only; pitch/yaw/throttle straight through the
  mixer as MANUAL.
- Airspeed for the turn-comp / (future) gain scheduling is a **GPS-ground-speed
  proxy** floored at 10 m/s — no pitot yet, unreliable in wind.

Gains are compile-time constants in `main_stm32.cpp setup()` (and the SITL
config), **rough SITL values, un-flight-tuned**. `core/params` will make them
adjustable without a reflash.

---

## Flash

### Recommended: ST-Link over SWD

An ST-Link V2 (clones ~€5) gives real breakpoints and live-variable watch via
**Cortex-Debug**, plus SWO/RTT printf that doesn't fight the USB CDC stream.
Single best time investment. *(not yet acquired)*

### Current: DFU (STM32 ROM bootloader)

Flashes over USB DFU (`0483:df11`). **Enumeration is flaky on the Raspberry Pi**
— see #5.

1. Set **BOOT0 = 1** and press **reset** (keep BOOT0 high) — or send `REBOOT_BL`
   over the USB CDC console (no button needed).
2. If `dfu-util -l` is empty, unplug/replug while BOOT0 stays high. Confirm
   `0483:df11`, not `0483:5740` (5740 = the running app).
3. `pio run -e black_f407ve -t upload`  (or `dfu-util -a 0 -s 0x08000000:leave -D .pio/build/black_f407ve/firmware.bin`)
4. Set **BOOT0 = 0** and reset to run.

**USB CDC console commands** (newline-terminated, into `/dev/ttyACM0` @ 115200):

| Command | Effect |
|---|---|
| `RESET_STATS` | zero the scheduler counters for a clean `SCHED` measurement window |
| `REBOOT_BL` | jump to the DFU bootloader (no BOOT0 button) |
| `SIM_FLYING 0\|1` | bench: force the `s_flying` latch to exercise the in-flight logging / integrator path |

---

## SD card logging

SDIO, FatFs. Firmware writes **binary** `FLTxxxxx.BIN` (next free index each
boot) — a 20-byte `LogFileHeader` (`STFC` + version + scales) then packed
86-byte `LogFrame` records, each with magic `0x5AA5` and a trailing
CRC-16/CCITT. Layout defined once in `src/core/log_frame.hpp`. ~50 Hz, via a
16 KB SPSC ring drained by `task_log_flush` at 5 Hz (few large writes beat many
small ones — see DECISIONS 2026-09-08). `f_sync` runs only on the ground. No
`sprintf` in the hot path.

Each frame carries: `t_ms`, accel[3] (g), gyro[3] (dps), attitude[3] (deg),
`press_pa`, `temp`, baro AGL, GPS lat/lon/alt/speed/sats/fix, all 8 `rc_us`
inputs, all 8 `out_us` outputs, `mode`, and armed / failsafe / requested-mode
flags.

Decode / plot on the PC:

```bash
python3 tools/parse_bin_log.py FLT00007.BIN            # -> FLT00007.csv (real units)
python3 tools/parse_bin_log.py FLT00007.BIN --summary  # frame count / Hz / CRC errors
python3 tools/plot_log.py       FLT00007.BIN           # interactive: tree of channels,
                                                      #   Plot / Overlay / Grid / normalize,
                                                      #   x = time (s), zoom-pan-save toolbar
```

`plot_log.py` needs `python3-matplotlib python3-numpy python3-tk` (apt) and
also auto-writes the `.csv` next to the `.BIN` on open.

Vendored `lib/STM32SD/` and `lib/FatFs/` contain critical fixes — **do not
regenerate them from upstream** (see #2).

---

## CRSF telemetry (FC → handset)

The flying aircraft has no USB; the pilot's picture of the FC comes back over
the CRSF uplink. `task_crsf_tx` @ 10 Hz sends, gated on link-up:

| CRSF frame | EdgeTX sensor(s) |
|---|---|
| `0x1E` attitude | `Ptch` `Roll` `Yaw` |
| `0x07` vario | `VSpd` (from the baro climb rate) |
| `0x02` GPS | `GPS` `GSpd` `Hdg`(0 for now) `GAlt` `Sats` |
| `0x21` flight mode | `FM` — `MANUAL` / `ASSIST` / `TKOFF`, `*` disarmed, `!FS` failsafe, `!LOCK` IMU-fault demotion |
| `0x08` battery | built but **not sent** — no pack-voltage sensor on the board yet |

On the TX16S: set **ExpressLRS → Telem Ratio** to `Std`/`1:8` (not Off), then
**Model → Telemetry → Discover new sensors**. Alarm on `RQly`. See the chat
history / `docs/` for the full sensor glossary and screen setup.

---

## Serial protocol (bench only)

USB CDC, 115200 baud, tagged CSV, parsed by leading tag. **Bench scaffolding —
deleted once the aircraft flies on SD + CRSF only.** Tags include `BMI` `ATT`
`BMP` `GPS`/`GPS_STAT` `EST` (bias/trust/AGL/climb) `MODE` (active/req/armed/
failsafe/lockout/flying) `MODE_CHANGE` `OUT` `RC` `LINK` `CRSF_STAT` `SCHED`
`CAL_DONE` `SD_STATUS`/`SD_DBG` `BMI_STATUS` `BMP_STATUS`. Exact field lists are
in `src/main_stm32.cpp` (search the `F("...")` prints).

`usb_log_drops` counts whole lines dropped because the host wasn't draining the
CDC queue — non-zero is the drop-on-full logger doing its job instead of
stalling the loop.

---

## Host tools (`tools/`)

| Tool | What |
|---|---|
| `gui.py` | Tkinter monitor: BMP / BMI / GPS / attitude + artificial horizon, **Flight control** panel (mode, armed, failsafe, assist-lockout, stab-integrator, gyro-cal), **RC in / servo out**. Auto-reconnect, data watchdog. |
| `plot_log.py` | interactive `.BIN` flight-log plotter (see SD logging). |
| `parse_bin_log.py` | `.BIN` → CSV decoder; also a library (`decode()` / `write_csv()` / `FIELDS`). |
| `capture.py` | raw CDC capture that survives USB re-enumeration. |
| `imureader.py` | early IMU bring-up helper. |

---

## Known issues & lessons

**Read this before coding. Do not "clean up" anything here without asking.**

1. **BMI323 marginal solder — RESOLVED.** Hardware SPI once failed (chip ID
   `0xFF`); after reflowing VDD/VDDIO/GND/SDO, `imu_probe` verified 10 MHz with
   **0 errors over 1.4 M reads**. The driver now runs SPI1 at ~5.25 MHz,
   blocking, DRDY-gated (a 28-byte burst is ~45 µs). A residual ~0.04 % of
   over-polled samples showed register *tearing*, not a bus fault — fixed by
   the data-ready gate. Still to do: FIFO + DMA to decouple the read from the
   ODR and hit the ≥1 kHz target.
2. **SD hang.** The stock STM32 HAL SD timeout was `100000000 ms` (≈27.8 h),
   which looked like an infinite hang. Fixed in vendored code:
   `lib/STM32SD/src/bsp_sd.h` — `SD_DATATIMEOUT` = 2000;
   `lib/STM32SD/src/bsp_sd.c` — `SD_CLK_DIV` = 10U (4 MHz), `SD_BUS_WIDE` = 1B.
   `lib/FatFs/` **must** keep its `ffsystem/` folder or the build fails.
   *1-bit @ 4 MHz costs ~8× bandwidth; revisit 4-bit + DMA later.*
3. **ALS31300 removed.** It lived on PB8=SCL / PB9=SDA (bit-bang I²C, addr
   `0x60`). Its range is hundreds of gauss vs Earth's ~0.5 G — useless as a
   magnetometer — and its bit-bang I²C blocked the loop. A mystery device at
   0x7E on that bus ACKed but ignored register commands — also gone. PB8/PB9 are
   free now. Do not re-add it.
4. **u-blox must be cross-wired** (module TX → PA10, module RX → PA9). USART1
   has no remap on the F407: PA9 is TX-only, PA10 is RX-only. Wired straight the
   MCU receives nothing. Without an antenna it still sends `$GPTXT` + 1 Hz NMEA
   with an empty fix; indoors you may see `sats=0` forever.
5. **DFU enumeration is flaky on the Pi.** `dfu-util -l` often shows nothing
   (`device descriptor read/64, error -110`). A **cold** plug into DFU works;
   the CDC→DFU **live** transition (`REBOOT_BL`, any app→bootloader jump)
   reliably wedges the Pi port with `error -110` and stays wedged across
   replugs on that port. Unbinding/rebinding `xhci-hcd.N` does **not** clear it.
   What works: a different physical port on the *other* xHCI controller, or a Pi
   reboot. Pi-host bug, not firmware. (`:leave` makes the board run the app
   straight after flashing.) A marginal USB *cable* also produces `error -110` —
   swap it if the problem follows the board across ports.
6. **Serial capture contention.** `gui.py` and any `cat /dev/ttyACM0` open the
   same CDC port and split the stream. Kill the GUI before a clean capture.
   `capture.py` survives USB re-enumeration.
7. **Boot prints are lost** unless the firmware waits for the USB host —
   `setup()` has a bounded `while (!SerialUSB)` for exactly this.
8. **GPS baud auto-detect** locks onto the first baud that yields a
   checksum-valid NMEA sentence (it does *not* wait for a fix), then stops
   cycling the 9 candidates.
9. **No sensor enable pins.** Every sensor is always powered. The old firmware
   drove PB11 / PE11 / PC14 as "enables"; removed (PB11/PE11 are reserved
   flight-I/O). Do not add sensor power-gate code.
10. **SD writes are still blocking.** `disk_write` busy-waits on the card; most
    512 B writes return in 1–3 ms but occasionally stall ~14 ms on the card's
    flash-program cycle. This is the one thing keeping the 400 Hz loop from
    hard-real-time (`worst_pass_us ≈ 15 ms`, rare, dt-corrected, recovered by
    the scheduler's critical re-service). `task_log_flush` batches to 5 Hz to
    minimise how often it hits. Real fix: a non-blocking SD write path
    (`HAL_SD_GetCardState` polled per tick). A high-endurance / industrial card
    also shrinks the tail. See DECISIONS 2026-09-08.

---

## Roadmap

Phases are ordered. Steps that get skipped and then cause months of confusion
are the vibration test and the axis-mapping verification.

| Phase | Status |
|---|---|
| **0 — unblock** (hardware SPI, split, `[env:native]`, CRSF driver, scheduler, binary log) | **done** (except: ST-Link, IMU FIFO+DMA, DFU-jump ✔, hard-fault handler, **axis-mapping verification**, **vibration test**) |
| **1 — estimation** (gyro-bias cal, gated AHRS, baro ground ref) | **done** (6-point accel cal + UBX parser deferred) |
| **2 — MANUAL** (`RC_Channel`/`SRV_Channel`, mixer, arming, failsafe) | **done, flown** |
| **3 — SITL** (6DOF, scripted sticks, self-checks) | **done** |
| **4 — ASSIST + TKOFF** (PID, rate/angle loops, turn comp, bumpless, mode manager, CRSF telem) | **built, SITL-verified — not flight-tested** |
| **4.5 — pre-flight** | axis-mapping verify · `core/params` · fly & tune ASSIST/TKOFF · hard-fault handler |
| **5 — position** (UBX-NAV-PVT, GPS-aided heading, nav filter, AHRS centripetal comp) | not started |
| **6 — AUTO cruise** (TECS, altitude + track hold) | not started |
| **7 — AUTO nav** (L1, loiter, waypoints, RTL, geofence, mission GUI, landing) | not started |

AUTO is gated on a **pitot** — see below.

---

## Recommended hardware additions

1. **Pitot-static airspeed sensor** (DLVR-L05D or SDP33; MS4525DO is cheaper but
   drifts). Biggest single improvement to AUTO viability — control gains scale
   with dynamic pressure and TECS needs a real minimum-airspeed floor. Mount the
   probe clear of the EDF inlets. **AUTO work is deliberately deferred until
   this is fitted.**
2. **ST-Link V2** — see Flash.
3. **Dedicated 6–8 A switching BEC for the servos.** Do **not** parallel the two
   ESC BECs — cut one (or both) red wires. Six servos on a jet this size pull
   5–8 A on a gust; a brownout in a stabilised mode is a crash.
4. **Soft-mount the FC** (gel / O-ring standoffs) and **balance the EDF rotors**.
5. **GPS antenna placement** — far from the two 120 A ESCs and battery leads,
   elevated, ground plane under it, twisted ESC power leads.
6. Optional: **M9N/M10** GNSS (10 Hz, multi-constellation); a magnetometer
   (**RM3100** > **IST8310**) only if a zero-speed heading reference is wanted —
   GPS ground-course heading is acceptable for fixed-wing in forward flight.
7. Optional: **high-endurance / industrial microSD** — shrinks the blocking
   write tail (#10) from ~14 ms to ~2 ms with no code change.

---

## Git

Remote: `https://github.com/zelveron/STM32FC_v2` (branch `main`). Author
identity `zelveron <zelveron@users.noreply.github.com>` is set repo-locally.
The older `github.com/zelveron/STM32FC` is the previous-generation reference
codebase, not this project.
