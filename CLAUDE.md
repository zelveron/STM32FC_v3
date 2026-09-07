# CLAUDE.md — STM32FC

Instructions for AI coding agents working in this repository.
Read this file and `README.md` in full before touching any code.

---

## What this project is

A fixed-wing autopilot for a 1.5 m 3D-printed twin-EDF Boeing 787 model
aircraft, running on a custom STM32F407VET6 board.

The repository currently contains a **working sensor streamer**, not a flight
controller. The job is to convert one into the other, incrementally, without
breaking what already works.

Architecture is modelled on ArduPilot's *structure* — hardware abstraction,
cooperative scheduler, mode classes, flash parameter store, layered failsafe.
The algorithms are reimplemented from scratch and kept deliberately simpler
(complementary filter, not EKF). **This is not a port of ArduPilot.** Do not
copy ArduPilot source into this tree.

> **This code flies a real aircraft over real ground.**
> A bug here destroys hardware or injures someone.
> Correctness beats cleverness beats brevity, always.
> When you are unsure, stop and ask. Do not guess and proceed.

---

## Hardware

| Item | Part | Interface |
|---|---|---|
| MCU | STM32F407VET6, 168 MHz Cortex-M4F | — |
| IMU | **BMI323** (chip ID `0x43`) | SPI1: PA4=CS PA5=SCK PA6=MISO PA7=MOSI |
| Baro | BMP581 (I2C addr `0x47`) | I2C1: PB6=SCL PB7=SDA |
| GNSS | u-blox NEO-7M | USART1: PA9=TX PA10=RX |
| RC link | RadioMaster ER8 (ExpressLRS), CRSF @ 420000 8N1 | *not yet wired — see plan* |
| ESCs | 2× 120 A, standard PWM | *not yet wired* |
| Servos | 6× (2 aileron, 2 elevator, 1 rudder, 1 nosewheel) | *not yet wired* |
| Storage | microSD | SDIO: PC8–PC12, PD2 |
| USB | native CDC | PA11/PA12 |

Transmitter: RadioMaster TX16S MK3 running EdgeTX.

**Note:** earlier project notes said "BMI270". That was wrong. The part on the
board is a **BMI323**. It does *not* need the BMI270 config-blob upload.

**No enable/power pins.** Every sensor is powered from the board's regulators
and is always on. There is no GPIO to gate any sensor's power. Do not add code
that toggles a sensor "enable" line — there is no such line.

**ALS31300 Hall sensor is removed.** It is not on the flight board, it cannot
serve as a magnetometer (its range is hundreds of gauss; Earth's field is
~0.5 G), and its bit-bang I2C blocked the loop. Do not re-add it. If a
pre-takeoff heading reference is wanted later, it will be a dedicated
magnetometer (RM3100 / IST8310) — see README "Recommended hardware additions".

### Free pins reserved for flight I/O

Do not allocate these to anything else.

| Function | Pins |
|---|---|
| Servo/ESC outputs 1–4 | TIM4 CH1–4 — PD12, PD13, PD14, PD15 |
| Servo/ESC outputs 5–8 | TIM1 CH1–4 — PE9, PE11, PE13, PE14 |
| CRSF RX/TX | USART2 (PA2/PA3) — fallback USART3 (PB10/PB11) |

### Output channel map

| Ch | Function |
|---|---|
| 1 | Aileron left |
| 2 | Aileron right |
| 3 | Elevator left |
| 4 | Elevator right |
| 5 | Rudder |
| 6 | Nosewheel steering (independent — **not** a mechanical Y with rudder) |
| 7 | ESC left |
| 8 | ESC right |

---

## Sensor axis → body frame mapping (define before any control code)

The BMI323 die axes are **not** guaranteed to line up with the aircraft body
frame. Nothing downstream — attitude, rate control, mixer — can be trusted
until this mapping is measured, written down here, and verified.

Body frame is **FRD**: X forward (nose), Y right (starboard wing), Z down.

Procedure, to be done on the bench once the board is in its final orientation
in the airframe:

1. Board level, nose north, wings level. Log raw accel. Identify which sensor
   axis reads ≈ **+1 g** (that axis, negated, is body **+Z**... i.e. the axis
   reading +1 g at rest is body −Z because gravity points down and accel
   measures specific force; confirm sign against the convention below).
2. Pitch the nose **up** ~30°. The sensor axis whose reading goes **negative**
   is body **+X** (nose-up tips gravity onto −X, so `a_x` reads negative for
   positive pitch — matches `pitch = atan2(-a_x, …)`).
3. Roll **right wing down** ~30°. The sensor axis whose reading goes
   **positive** is body **+Y** (`roll = atan2(a_y, a_z)` positive).
4. Rotate the board **clockwise seen from above** (nose swings right). The gyro
   axis that reads **positive** is body **+Z** (yaw-right = +r).
5. Cross-check the two remaining gyro axes: nose-up pitch rate must read
   **positive** on body +Y (+q), right-roll rate **positive** on body +X (+p).

Record the result as a fixed permutation + sign vector, e.g.:

```
// Example only — MEASURE, do not copy.
// body_x =  +sensor_y
// body_y =  -sensor_x
// body_z =  +sensor_z
```

Apply it in the driver, immediately after the raw read, once — never scatter
axis swaps through the estimator or mixer. Same treatment for any magnetometer
added later. Put the verified table in this section and add a paragraph to
`docs/DECISIONS.md`.

Sign conventions this mapping must satisfy:

- Positive roll = right wing down. Positive pitch = nose up. Positive yaw =
  clockwise seen from above.
- `roll  = atan2(a_y, a_z)`
- `pitch = atan2(-a_x, hypot(a_y, a_z))`
- Body rates `p, q, r` about `+X, +Y, +Z` respectively.

---

## Current blockers

Do not start work on estimation, control, or flight modes until these are
resolved. If asked to, say so and refuse.

1. **BMI323 is on bit-bang SPI at ~200 kHz** because of marginal solder joints
   (`bb_xfer()` in `src/drivers/bmi323.cpp`). The Bosch core reads 26 data bytes plus
   a dummy plus the address per sample = 28 byte-transfers; at bit-bang speed
   that is **~1.1–1.7 ms of blocking CPU per accel+gyro read**, not the ~500 µs
   the old README claimed. A flight controller needs 1 kHz sampling; one read
   already blows a 1 ms tick budget on its own. The joints must be reflowed and
   hardware SPI at 10 MHz verified. **This is a hardware fix, not a software
   one.** Do not replace the bit-bang code with hardware SPI until the reflow is
   verified (chip ID `0x43` at 10 MHz, then several million reads with zero
   mismatches).
2. **Sampling rates are datalogger rates, not flight rates.** Gyro at 200 Hz
   ODR / ~100 Hz sampled, baro at ~10 Hz, GPS at 1 Hz NMEA. See targets below.
3. ~~Everything is in one file.~~ **Done** — `src/main.cpp` is split into the
   layout below (`hal/`, `drivers/`, `estimation/`, `core/`, `main_stm32.cpp`),
   `[env:native]` added. The bit-bang IMU blocking (blocker 1) is unchanged.

---

## Sensor configuration targets

| Sensor | Current | Target | Reason |
|---|---|---|---|
| BMI323 gyro/accel | 200 Hz ODR, ~100 Hz sampled, bit-bang | **≥1.6 kHz ODR, internal filter ON, FIFO + DMA, decimate to 1 kHz** | Two EDFs at 30–45k RPM put energy above 1 kHz. Sampling slower aliases it into the control band as phantom motion the PIDs will chase. |
| BMI323 accel range | ±4 g | **±8 g or ±16 g** | Launch, gusts and landing clip ±4 g. Clipped accel corrupts attitude exactly when it matters. |
| BMP581 | ~10 Hz forced mode, I2C | **≥50 Hz** | Climb rate is the derivative of altitude. 10 Hz gives an unusably laggy derivative. |
| NEO-7M | NMEA, 9600, 1 Hz | **UBX binary, 115200, 5 Hz, `UBX-NAV-PVT`** | One message gives position, NED velocity, fix type, sat count and accuracy estimates. NED velocity is what the nav filter needs. |
| SD log | CSV text, 50 Hz, 1-bit @ 4 MHz | **Binary records, ring buffer, 512-byte aligned writes, 4-bit** | `sprintf` of 15 floats costs hundreds of µs. First flights need raw gyro at 500–1000 Hz for FFT analysis. |

---

## Telemetry / logging strategy

- **USB CDC (`SerialUSB`) tagged-CSV output is bench scaffolding only.** There
  is no ST-Link on this project; USB CDC is the only debug channel during
  bring-up. It will be **removed** once the restructure is validated. The flying
  aircraft has **no USB connection**.
- Flight telemetry to the pilot goes over **CRSF** back to the TX16S (EdgeTX
  announces mode / battery / GPS audibly).
- Flight data recording is **SD only**: binary records, ring buffer, non-blocking.
- **Until USB is removed, every write to `SerialUSB` must be non-blocking** —
  bounded, drop on full, never spin. A blocking log call stops the aircraft.

---

## Hard rules

These are not style preferences. Violating them is a bug.

1. **No `#include` of any STM32, CMSIS or Arduino header outside `src/hal/stm32/`.**
   Everything in `src/core/`, `src/estimation/`, `src/control/` and `src/modes/`
   must compile on the native desktop target. Need hardware? Add a method to the
   `hal::` interface and implement it in *both* backends.
   *Documented exception:* `src/core/usb_stream.*` and `src/core/sd_bin_log.*`
   are throwaway bench scaffolding (USB CDC telemetry + CSV SD log) that use
   Arduino directly and are excluded from `[env:native]`. They are deleted when
   the USB path goes. Do not add more exceptions; do not "fix" these.
2. **No dynamic allocation after `init()`.** No `malloc`, `new`, `String`,
   `std::vector`, `std::string`, `std::function`. Fixed-size arrays and static
   storage only. No exceptions, no RTTI.
3. **No blocking calls in any scheduler task.** No `delay()`, no
   `delayMicroseconds()`, no spin on a peripheral flag, no unbounded `while`.
   Drivers are state machines that make progress each call and return.
4. **No floating point in ISRs.** ISRs copy bytes and set flags. Math happens in
   scheduler tasks.
5. **Units go in the identifier name.** `roll_rate_dps`, `alt_m`,
   `airspeed_mps`, `pressure_pa`, `dt_s`. A bare `roll` or `alt` is rejected.
6. **Every function that can fail returns a status.** No silent failure, no error
   swallowed by a bare `return`.
7. **Never modify MANUAL mode or `core/failsafe` as a side effect of another
   change.** If a change requires touching them, stop and say so first.

---

## Conventions

- **Frames.** Body frame is X forward, Y right, Z down (FRD). World frame is
  North-East-Down (NED). Positive roll = right wing down. Positive pitch = nose
  up. Positive yaw = clockwise seen from above.
- **Angles.** Radians internally, everywhere. Degrees only at the parameter
  boundary and in logs; suffix those `_deg`.
- **Control outputs.** Normalized `float` in `[-1, +1]`. Throttle `[0, 1]`.
  Conversion to microseconds happens **only** in `SRV_Channel`.
- **Surface signs.** Positive aileron output = roll right. Positive elevator =
  pitch up. Positive rudder = yaw right.
- **Time.** `uint32_t` microseconds from `hal::micros()`. Compare by
  subtraction so wraparound is handled: `(now - then) > timeout`, never
  `now > then + timeout`.
- **Language.** C++17, restricted subset. `constexpr` over macros. Free
  functions where there is no state.
- **Naming.** `PascalCase` types, `snake_case` functions/variables, `_leading`
  underscore for private members, `SCREAMING_CASE` constants.

---

## Target layout

```
src/
  hal/              hardware abstraction — the ONLY layer touching STM32/Arduino
    hal.hpp         the interface both backends implement          [exists]
    stm32/          real hardware (may call HAL_*/LL_* directly)    [exists]
    native/         desktop backend for tests and SITL             [exists]
  drivers/          bmi323, bmp581, ublox [exist]; crsf, esc_out — depend on hal only
  core/             scheduler, failsafe, arming, log_ring, log_frame [exist];
                    params [none yet]; usb_stream, sd_bin_log [exist, BENCH — rule 1 exception]
  estimation/       ahrs (gated Mahony), imu_prep, baro_alt [exist]; ins, nav_filter
  control/          rc_channel, srv_channel, mixer [exist]; pid, rate_ctrl, attitude_ctrl, tecs, nav_l1
  modes/            mode.hpp + mode_manual [exist]; mode_assist, mode_auto
  sitl/             aircraft (6DOF), sensors [exist]; + main_sitl.cpp
  main_stm32.cpp    application entry (setup/loop)                  [exists]
  main_native.cpp   [env:native] entry                             [exists]
lib/                vendored: bmi323, bmp5, STM32SD, FatFs  (see README)
tests/              native unit tests
tools/              gui.py, capture.py, log parsers, plotters
docs/DECISIONS.md   one paragraph per design decision, append-only
```

We stay on **PlatformIO + Arduino framework** for now. The STM32 Arduino core is
built on STM32Cube HAL, so timing-critical drivers can call `HAL_*` / `LL_*`
directly without abandoning the working SD stack.

Build envs (`platformio.ini`):
- `black_f407ve` — the firmware. Excludes `main_native.cpp`, `hal/native/`.
- `native` — portable layers only (`hal.hpp` + `drivers/` + `estimation/` +
  `hal/native/` + `main_native.cpp`). Proves those layers stay Arduino-free.
  Excludes `main_stm32.cpp`, `hal/stm32/`, the bench `core/*` files.
- `sitl` — desktop 6DOF simulator: stick input through the real control chain
  (`control/` + `modes/`) into `sitl/`, then the AHRS; CSV state to stdout.
  `--check` self-tests trim + control signs + no-departure.
- `crsf_probe`, `imu_probe` — diagnostic firmwares (not flight).

---

## Scheduler

Single-threaded cooperative scheduler. No RTOS.

| Hz | Task |
|---|---|
| 1000 | IMU FIFO drain + filter |
| 400 | rate controllers, mixer, servo output |
| 200 | AHRS update |
| 100 | angle controllers, CRSF parse |
| 50 | TECS, baro, mode logic, failsafe |
| 25 | L1 navigation |
| 10 | GPS parse, CRSF telemetry TX, log flush |
| 1 | battery, health, param save |

Every task is instrumented with the DWT cycle counter (STM32) or a chrono clock
(native). Log min/max/mean runtime and an overrun count per task. **A task that
overruns its budget is a bug, not a tuning issue.**

Enable IWDG at ~200 ms, kicked only from the main loop. On watchdog reset, boot
straight into MANUAL passthrough and latch that.

---

## Flight modes

One transmitter channel, three positions.

- **MANUAL** — direct passthrough. This path must be as short and independent as
  physically possible and must work even if the IMU driver has faulted. This is
  where the aircraft falls back when anything goes wrong.
- **ASSIST** — stick commands an attitude *angle*, clamped (ArduPlane FBWA
  equivalent). Angle loop at 100 Hz feeding a rate loop at 400 Hz.
- **AUTO** — built in this order, never skipped: altitude + track hold →
  loiter → waypoints + RTL.

**Mode transitions must be bumpless.** On `enter()`, preload every PID
integrator so the initial output equals the current servo position. A servo snap
at 40 m/s loses the airframe. Test this in SITL. It is not optional.

Sensor faults demote **downward only**: AUTO → ASSIST → MANUAL. Never promote
automatically. Every demotion is latched, logged, and announced to the pilot
over CRSF telemetry. A silent mode change is worse than no failsafe.

---

## Control notes

- **Feedforward dominates on a fixed wing.** Surface deflection is roughly
  proportional to desired rate. Tune FF first, then P, then I.
- **Pitch compensation in turns.** Banking loses vertical lift. Add pitch-up
  feedforward proportional to `tan(bank)·sin(bank)·g/V` or the nose drops in
  every turn and the I-term fights it.
- **Gains scale with dynamic pressure** — roughly `(V_ref/V)²`, clamped 0.5–2.0.
  With no airspeed sensor this is approximated from GPS ground speed and is
  unreliable in wind. Flag this limitation in any auto-mode code.
- **Differential thrust** is free yaw authority (two independent EDFs). Make it a
  tunable gain, gated to zero below a throttle threshold.
- **Nosewheel** gets its own channel and gain, scaled down with ground speed and
  centred entirely once airborne.
- **TECS**: throttle controls total energy, pitch controls the balance between
  potential and kinetic. Never "pitch holds altitude, throttle holds speed" —
  the loops fight and you get phugoid oscillation. A minimum-airspeed floor
  overrides altitude demand, always.

---

## Verification

- `pio run -e black_f407ve` — cross build
- `pio test -e native` — native unit tests
- `pio run -e sitl` — desktop simulator

**Before claiming any change works, run the build and the tests and paste the
actual output.** For control changes, show the SITL trace. "This should work" is
not acceptable. If a change cannot be verified, say so plainly rather than
implying it was tested. There is no ST-Link on this project — hardware
verification is the human flashing over DFU and watching `tools/gui.py`.

---

## Working style

- Small commits, one logical change each, reasoning in the message.
- **Ask before adding any dependency.**
- **Ask before restructuring anything documented in the README's "Known issues"
  section.** That section records hardware faults and vendored-library patches
  discovered the hard way. A cleanup pass that "fixes" them will break the board.
- Sign errors cause more crashes here than algorithm errors. When touching the
  control or mixer path, state the sign convention you assumed in the commit
  message.
- Append a paragraph to `docs/DECISIONS.md` whenever a design question is
  settled, so the reasoning survives between sessions.
