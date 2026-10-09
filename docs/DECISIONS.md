> Historical decision log. For current v2.2 behavior, use README.md and the 2026-09-28 entry below; earlier hardware and flight-safety claims are superseded.

# Design decisions

Append-only. One paragraph per settled design question. Newest at the bottom.

---

**2026-09-03 — New project tree, cloned from the streamer.** Work starts in
`~/Desktop/STM32FC_Claude`, seeded by cloning `github.com/zelveron/STM32FC`
(commit `9b7855e`, the ALS/heading streamer). `CLAUDE.md` and `docs/` did not
exist in that repo; they are created here as the spec for the flight-controller
conversion. Phase 0 work happens on branch `phase0-cleanup`.

**2026-09-03 — Sequence: cleanup before restructure.** The briefing defines the
`main.cpp` split (Phase 0 step 3) as a *pure* refactor, verified by "the GUI
shows the same data". Several agreed fixes are deliberate behaviour changes
(remove ALS, remove enable pins, non-blocking USB, BMP timing, attitude filter
signature). Doing them inside the split would destroy that verification
checkpoint. So they are done first, as small behaviour-scoped commits on the
flat file, each build-checked; then the split is a mechanical move with no
behaviour change.

**2026-09-03 — ALS31300 removed entirely.** Not kept behind a compile flag. It
is physically off the board, it cannot work as a magnetometer (full-scale
hundreds of gauss vs Earth's ~0.5 G), and its software-bit-bang I2C read
(~1.2 ms) blocked the main loop. All `als_*` code, the `mag_heading`
computation, the `ALS_*` serial tags and the `als_heading` SD column are
deleted. PB8/PB9 are freed. A future heading reference, if wanted, will be a
dedicated magnetometer (RM3100 / IST8310), not this part.

**2026-09-03 — No sensor enable pins.** The board has no power-gate GPIO for any
sensor; all sensors are always on. The old firmware drove `PB11` and `PE11`
(both reserved flight-I/O pins — CRSF fallback UART and TIM1 servo output 6) and
`PC14` (an LSE oscillator pin) as sensor "enables". All of that GPIO setup is
removed. `BMP_EN` / `BMP_EN_ALT` macros deleted.

**2026-09-03 — USB CDC telemetry is temporary bench scaffolding.** There is no
ST-Link; USB CDC is the only bring-up debug channel. The tagged-CSV stream, the
GUI and the boot/debug prints exist only to validate the restructure. The flying
aircraft has no USB connection — flight recording is SD-only (binary, ring
buffer) and pilot telemetry is CRSF. Until USB is removed, every `SerialUSB`
write goes through a bounded, line-atomic, drop-on-full wrapper that never
spins (the stock core `USBSerial::write` busy-waits up to 3 ms per call on a
connected-but-stalled host and can emit half lines).

**2026-09-03 — BMP581 read is trigger-then-collect, not trigger-and-block.** The
old code did `set_power_mode(FORCED); delay(30); get_sensor_data()` every 100 ms
— a 30 ms hard block of the whole loop, and 30 ms is shorter than the worst-case
OSR-16 pressure conversion so it occasionally read a not-ready sample. Now the
forced measurement is triggered on one tick and the result collected on the
next; no `delay`. ODR configuration (meaningless in forced mode) is dropped.
BMP581 init moves entirely into `setup()` — it is no longer retried from inside
`loop()`.

**2026-09-03 — IMU scale factors derive from the configured range.** The
`* 4.0f / 32768.0f` (accel) and `* 2000.0f / 32768.0f` (gyro) literals appeared
both in the sensor config and again at the point of use. They are now a single
`constexpr` derived from the configured `BMI3_ACC_RANGE_*` / `BMI3_GYR_RANGE_*`,
so changing the range (e.g. to ±8 g per the Phase 0 target) cannot silently
leave the conversion at half scale.

**2026-09-03 — Attitude filter: dt injected, radians internal, yaw wrapped.**
`att_update()` took no `dt` and read `micros()` itself, so it could not be unit
tested on the native target; it also stored angles in degrees (against the
"radians internally" rule) and let yaw grow unbounded. It now takes `dt_s`,
keeps state in radians, wraps yaw to `[-pi, pi]`, and the caller owns the
clock. This is a mechanical fix only — the filter is still **not flight-grade**:
no accel gating, no gyro bias calibration, unreferenced drifting yaw. That work
is Phase 1.

**2026-09-03 — Sensor axis → body-frame mapping is a Phase 0 deliverable.** The
BMI323 die axes are not assumed to match body FRD. The mapping must be measured
on the bench with the board in its final airframe orientation, recorded in
`CLAUDE.md`, and applied once in the driver immediately after the raw read —
never scattered through the estimator or mixer. No control code lands before
this exists.

**2026-09-03 — `Wire` (BMP581 I2C) timeout shortened.** The STM32 core already
bounds every I2C transfer at `I2C_TIMEOUT_TICK` (default 100 ≈ 100 ms) — the
earlier claim of "no bounded timeout" was wrong — but there is no runtime API to
shorten it, so a wedged bus still stalls the loop for 100 ms. Set
`-D I2C_TIMEOUT_TICK=10` in `platformio.ini` (a 6-byte BMP581 read is <1 ms, so
10 ms is very generous). The real fix is a hal-backed non-blocking I2C state
machine, later.

**2026-09-03 — Dead bench diagnostics removed.** `pin_edge_count2()` (never
called), `pin_edge_count()` and the `PA_EDGES` block (two 50 ms busy-waits every
4 s; the crossed-wiring question it answered is settled — Known issues #4),
`i2c_scan()` and `als_debug_probe()` (ALS-era bus probes) are deleted. The GPS
`GPS_DBG` / `GPS_FIRST` / `GPS_RAW` prints stay for now — GPS still has no
indoor fix and they are cheap — and will be removed with the rest of the USB
path.

**2026-09-03 — main.cpp split into the layered layout (one commit).** `src/main.cpp`
(870 lines) is replaced by: `hal/hal.hpp` (full interface -- time, gpio, spi
blocking+DMA, i2c, uart, pwm, block device, reset cause, bootloader jump) with
`hal/stm32/` and `hal/native/` backends; `drivers/{bmi323,bmp581,ublox}`
(hal-only, portable, compiled by `[env:native]`); `estimation/ahrs`; the two
bench modules `core/{usb_stream,sd_csv_log}` (Arduino-coupled, native-excluded --
rule 1 documented exception); `main_stm32.cpp` (setup/loop composition +
bench prints) and `main_native.cpp` (smoke). Pure refactor: `black_f407ve`
emits the same tagged CSV at the same cadences. Known, accepted deltas: (a) if
two GGA sentences land in one `loop()` pass, GPS_STAT/GPS now print once with the
later values instead of twice -- cannot happen at 1 Hz NMEA; (b) UBX TX is now
non-blocking (drops a frame if the UART TX ring is full instead of busy-waiting)
-- frames are <=28 B into a 64 B ring every 2 s, so it never actually drops;
(c) `Wire.begin()` now runs inside `bmp581::begin()` after the USB-host wait
instead of before it -- nothing uses I2C in between. SPI/PWM/block-device hal
calls are declared but return `Status::unsupported` (no caller yet: SPI waits on
the BMI323 reflow, PWM on Phase 2, block device on the binary logger).
`reset_cause()` and `jump_to_bootloader()` are implemented for real (F407
`RCC->CSR` decode; system-memory `0x1FFF0000` entry) though nothing calls them
yet -- they are the two "debugging without a debugger" early tasks.

**2026-09-03 — Cooperative scheduler (`core/scheduler`).** Single-threaded, no
RTOS, as specified in CLAUDE.md. Fixed 16-slot task table; `add(name, rate_hz,
fn)` before `run()`. Each pass runs every due task once (bounded work),
profiling it with `hal::cycles()` (DWT on STM32, nanoseconds on native) into
per-task min/max/mean/last us + an overrun count (dispatched >half a period
late, or ran longer than its period). `run_once()` is exposed so a test/SITL
harness can step it; `run()` just loops it forever. Optional IWDG via
`set_watchdog_ms()` (default off -- opt-in, since once started it can't be
stopped and a spurious reset during bring-up is worse than no watchdog until
MANUAL exists to fall back into). New hal surface it needs: `cycles()`,
`cpu_hz()`, `watchdog_start()`, `watchdog_kick()`; `hal::init()` now enables
the DWT counter. Portable -- built and asserted by `[env:native]`
(`main_native.cpp`: 100 Hz task runs 50x / 20 Hz runs 10x over 500 ms).
NOT yet wired into `main_stm32.cpp` -- the streamer loop adopts it in the CRSF
session, when CRSF parse becomes the first real scheduled task.

**2026-09-03 — BMI323 moved to hardware SPI; bit-bang deleted.** The
`imu_probe` verified hardware SPI at 10 MHz: 0 chip-ID errors, 0 comm failures
over 1.4M reads (the ~0.04% large-accel-delta "glitches" were register tearing
from polling 8x above the ODR with no DRDY gate -- a probe artifact, not a link
fault; chip-ID on the same wire was perfect and the rate was flat across clock
speed). `hal::spi_config`/`spi_xfer` implemented on SPI1 via the core's polled
LL block transfer (`SPI.transfer(tx,rx,n)`), ~45 us for a 28-byte burst vs
1436 us bit-bang. `drivers/bmi323` rewritten: hardware-SPI Bosch callbacks,
`begin()` owns SPI setup, `read()` is DRDY-gated (`bmi3_get_sensor_status` ->
`Result::{ok,no_data,comm_error}`), flight config 1600 Hz ODR / +/-8 g / 2000
dps / high-perf / internal filter on. `bb_xfer` and the bit-bang read/write
callbacks removed; `hal::pins::imu_sck/miso/mosi` and `bmi323::config_pins()`
removed; `hal::gpio_*` kept only for CS. SPI clock: 8 MHz request -> ~5.25 MHz
actual (SPI1 APB2/16), inside the 10 MHz datasheet limit. `spi_xfer_async`
(DMA) still returns unsupported -- next optimisation, with a raw-sample spike
filter. Build green on both envs; hardware-flash verification pending.

**2026-09-07 — CRSF on USART3 (PB10/PB11).** The ER8 (ExpressLRS) receiver is
wired to USART3: PB11 <- RX TX, PB10 -> RX RX, 420000 8N1 not inverted. That is
CLAUDE.md's documented CRSF *fallback* port (USART2 PA2/PA3 is the primary, but
PB10/PB11 were free once the enable-pin GPIO was removed, and that is where the
user soldered). `hal::Uart::crsf` now maps to a dedicated `HardwareSerial(PB11,
PB10)` instance (selects USART3 from the pins); `uart_config` only forces
setTx/setRx for the GPS. New `drivers/crsf`: non-blocking byte state-machine
parser (sync 0xC8, CRC-8/DVB-S2 poly 0xD5 over type+payload, resync on bad
length/CRC), decodes RC_CHANNELS_PACKED (0x16 -> 16x 11-bit -> us via
(raw-992)*5/8+1500) and LINK_STATISTICS (0x14), counts frames/crc_err/resync.
Verified via `[env:crsf_probe]` streaming RC/LINK/CRSF_STAT over USB. Real
driver adds DMA circular RX + UART IDLE IRQ and telemetry TX later.

**2026-09-07 — CRSF reader verified on hardware.** ER8 on USART3 (PB11/PB10),
420000 8N1. After fixing the ELRS output baud (it was NOT 420000 -- a
mismatch gives a continuous byte stream at a plausible rate but ZERO valid
frames / zero CRC errors, because no byte lands on 0xC8): 30,000+ frames
decoded, 1 CRC error (an RF bit-flip, correctly rejected), 0 resyncs. RC
channels unpack correctly (throttle 989 at min, sticks 1500 centred, switches
at 1000/1503/2011); LINK_STATISTICS decode (up_lq=100, up_rssi -30 dBm,
up_snr 9 dB). Downlink stats read 0 -- no telemetry TX yet. `drivers/crsf`
(polled HardwareSerial RX) and `hal::Uart::crsf` are correct. Remaining CRSF
work: DMA circular RX + UART IDLE IRQ (CLAUDE.md spec; polled is fine at this
rate for now), telemetry TX (battery/GPS/attitude/mode uplink), CRSF-flag
failsafe. GOTCHA recorded: the ELRS Lua "output"/serial baud must be set to
420000 to match.

**2026-09-07 — Scheduler adopted in `main_stm32`.** The flat super-loop is
gone: each old `if (millis()-last >= period)` block is now a free function
registered with `sched::add()`, and `setup()` ends with `sched::run()`
(`loop()` is empty). Tasks + rates: gps 50, bmi_retry 1, bmi 100, att 20,
bmp 50, sd_log 50, debug 2, sched 1. Registration order = producers before
consumers (bmi writes g_*/ahrs before att/sd_log read them). New `SCHED,*`
line at 1 Hz reports per-task DWT min/mean/max us + overrun count + loop
pass count + worst pass -- the GUI ignores unknown tags so the stream is
otherwise identical. GPS drops from every-iteration to 50 Hz (20 ms between
UART drains, safe for the RX ring at 9600 baud; ublox's internal autobaud/UBX
timers are millis-based and unaffected). IWDG left off (commented) until
MANUAL exists. This flash also first-verifies the hardware-SPI BMI323 driver
(bit-bang deleted) on real hardware. Builds: black_f407ve RAM 9028 / Flash
94468 B.

**2026-09-07 — CRSF wired into the flight firmware.** `main_stm32` now calls
`crsf::begin(420000)` in setup and runs `task_crsf` at 100 Hz (CLAUDE.md
scheduler table: "100 Hz | CRSF parse"). It streams `RC,<16 us>` at 20 Hz and
`LINK,...` at 5 Hz over USB, and `task_debug` gains a `CRSF_STAT` line. Added
`-D SERIAL_RX_BUFFER_SIZE=256` to `black_f407ve`: CRSF is ~10 kB/s, so a
100 Hz poll buffers ~100 B between calls and the core's 64 B RX ring would
overflow. The proper fix is DMA circular RX + UART IDLE IRQ ("come back to A").
Nothing consumes the channels yet -- RC_Channel / mixer / mode_manual are next.

**2026-09-07 — MANUAL mode + the control chain.** Built as: hal PWM
(TIM4 PD12-15 / TIM1 PE9/11/13/14, HardwareTimer, 333 Hz, us-direct);
`control/rc_channel` (us -> normalized, deadzone + reverse), `control/srv_channel`
(normalized -> us -- the only place that happens -- + safe_us for
disarmed/failsafe), `control/mixer::mix_manual` (stick passthrough to the
8-output map; both ailerons/elevators get the same signed value, L/R
opposition is the SrvChannel reverse flag; diff-aileron / diff-thrust
scaffolded at 0); `core/failsafe` (RC-loss only for now, debounced 200 ms
engage / 300 ms recover, starts engaged); `core/arming` (rising-edge arm
switch + throttle <= 5% + no failsafe; disarm on switch-low or failsafe;
disarmed forces ESC outputs to min); `modes/mode.hpp` + `modes/mode_manual`.
`main_stm32` runs `task_control` at 400 Hz: CRSF channels (AETR: roll ch0,
pitch ch1, throttle ch2, yaw ch3; arm switch ch4 > 1700 us) -> RcChannel ->
Sticks -> failsafe/arming -> mode.update -> SrvChannel -> hal::pwm_write_us,
and streams `OUT,<8 us>` (20 Hz) + `MODE,<name>,armed=,failsafe=` (2 Hz).
34 native assertions pass. Servos not wired yet -- verified via the `OUT,`
stream responding to sticks; servo/scope check comes when the airframe is
wired. IWDG still off. Sign conventions stated in mixer.hpp.

**2026-09-07 — Binary ring-buffer logger replaces the CSV logger.** The CSV
logger blocked up to 74 ms on `f_sync` (and ~3 ms/record on `sprintf` of 17
floats), racking up ~590 `control`-task overruns per 16 s. Replaced with:
`core/log_ring` (8 KB SPSC byte ring, non-blocking push, drop+count on
overflow) + `core/log_frame` (86-byte packed record: magic, t_ms, IMU as
g*2048/dps*16 int16, attitude deg*100 int16, baro, GPS, 8 RC us, 8 output us,
mode+flags, CRC-16/CCITT) + `core/sd_bin_log` (STM32-only; opens FLTxxxxx.BIN,
writes a LogFileHeader, `flush_step()` drains 512-byte-aligned sectors,
`sync()` on a ~5 s cadence -- no f_sync in the hot path). `main_stm32`:
`task_log` @ 50 Hz packs a frame into the ring (~few us), `task_log_flush`
@ 25 Hz drains it (the SD blocking is now isolated in one low-rate task, not
the CSV logger inside a 50 Hz task). `tools/parse_bin_log.py` decodes to CSV.
`sd_csv_log.{hpp,cpp}` deleted (recoverable from git). RAM +8 KB for the ring.
The remaining ~1-3 ms/sector SD write latency goes to zero with DMA SDIO
(`hal::blk_write` async) + 4-bit mode -- a later item.

**2026-09-07 — SITL harness (`[env:sitl]`).** Desktop 6DOF simulator, per
CLAUDE.md Phase 3 pulled forward (servos are deferred, so SITL is how
control logic gets verified). `sitl/aircraft` -- rigid body + linear aero
(lift/drag/side + roll/pitch/yaw moments with control + rate-damping terms),
sub-stepped Euler, quaternion attitude, numeric trim solve in reset().
`sitl/sensors` -- synthesizes IMU/baro/GPS in the real driver conventions
(level -> az=+1 g, nose-up -> ax negative). `main_sitl` runs stick input
(built-in doublet maneuver or a t,roll,pitch,yaw,thr CSV script) through the
ACTUAL `control::RcChannel` -> `modes::ModeManual` -> `control::mixer` chain
into the model, then `ahrs::update`, and prints a state CSV. `--check` asserts
trim stability, aileron-right->roll-right sign, and no departure (7 checks,
pass). CAVEATS: aero coefficients are rough order-of-magnitude for a ~1.5 m
twin-EDF, NOT airframe-validated -- this is for "does the control logic behave
right", not performance prediction. The phugoid is lightly damped; the current
(non-flight-grade) AHRS shows large pitch error under longitudinal
acceleration (the throttle-bump climb) -- exactly the failure Phase 1 accel
gating must fix, and now measurable in sim.

**2026-09-07 — Phase 1 estimation: gated AHRS + gyro-bias cal + baro ground ref.**
`estimation/imu_prep` -- boot-time stationary gyro-bias calibration (400-sample
window, restarts if the board is disturbed) + a first-order LPF on gyro/accel
(placeholder anti-alias; becomes a biquad/notch with the 1 kHz FIFO). Wired
into `task_bmi` before the AHRS. `estimation/ahrs` rewritten as a quaternion
gated complementary (Mahony) filter with online gyro-bias estimation: the
accel correction is weighted by a trust factor that goes to 0 when `|accel|`
leaves a tight 0.10 g band around 1 g OR body rate exceeds ~120 dps -- so a
throttle surge / coordinated turn / gust no longer pulls the attitude toward
the specific-force vector. Kp 1.0, Ki 0.05, tunable via `set_gains`. Yaw still
unreferenced (no mag) but bias-corrected so drift is small. `estimation/baro_alt`
-- ground-referenced AGL, ground pressure latched while disarmed (frozen on
arm); `main_stm32` logs AGL as `alt_mm` and streams an `EST,` line
(bias_ready, gyro bias, acc_trust, agl, baro_ref). SITL regression: max AHRS
pitch error across the built-in maneuver INCLUDING the throttle bump dropped
from ~13 deg (old fixed-gain filter) to ~4.6 deg; `--check` asserts < 8 deg.
Deferred: 6-point accel calibration (needs `core/params`), UBX-NAV-PVT binary
parser (Phase 5). Builds green all envs; 11 new native + 2 new SITL assertions.

**2026-09-07 — "gyro cal complete" = a control-surface sweep.** So the pilot
at the aircraft knows the boot gyro-bias calibration finished without a
screen: on the rising edge of `bias_ready`, while disarmed, `control::SurfaceTest`
sweeps aileron -> elevator -> rudder in sequence, each centre -> +full ->
-full -> centre over 0.6 s (~1.8 s total). It overrides the mixer output for
surfaces during the sweep and never touches throttle; it is cancelled if the
aircraft arms. `main_stm32` also emits a one-shot `CAL_DONE,gyro_bias` line.
Header-only, 6 native assertions. NOTE on timing: the calibration accumulates
from the moment `task_bmi` starts (after setup(), ~1-3 s) and completes after
~4 s of the gyro span staying under 4 dps -- it is continuous, not timed;
just do not move the aircraft until the surfaces wiggle.

**2026-09-07 — ASSIST mode (FBWA): angle loop -> rate loop -> mixer.**
`modes::ModeAssist` (Phase 4). Stick commands a clamped attitude *angle*
(`_max_roll` 0.70 rad, `_max_pitch` 0.45 rad); yaw stick is a direct
rate demand (`_max_yaw_rate` 80 dps); throttle is passthrough. Chain:
`control::AttitudeController` (angle-P `110 dps/rad` + coordinated-turn pitch-up
FF `g*tan(phi)*sin(phi)/V`, clamped) -> `control::RateController` (per-axis
anti-windup PID, D-on-measurement + D-LPF) -> the SAME `mix_manual` /
output map as MANUAL. Bumpless entry: `enter()` stashes the current outputs and
`preset()`s the rate integrators on the first `update()` so the first command
equals the held servo position. Rough first gains (FF-dominant per CLAUDE.md):
roll kff/kp/ki 0.006/0.010/0.02, pitch 0.010/0.020/0.02, yaw kff/kp
0.004/0.006 (no I). Verified in SITL only (`--assist-check`): holds commanded
bank, no oscillation, bumpless, returns to level on release, no departure.
Tune in flight.

**2026-09-07 — AHRS accel-trust gate widened to reject turns; SITL surface
signs fixed.** Two coupled findings while bringing up ASSIST in SITL:
(1) `src/sitl/aircraft.cpp` used textbook aero surface signs (`Cm_de < 0`,
`Cn_dr < 0` with trailing-edge-positive deflection), i.e. +elevator command
gave nose *down* and +rudder gave yaw *left* -- opposite the project
convention ("+elevator = pitch up, +rudder = yaw right"). MANUAL passed only
because it is open-loop and the checks never tested pitch/yaw *direction*;
ASSIST closed the loop and departed (nose to -90 deg, state NaN). Fixed by
negating the elevator and rudder commands into the model's aero frame (aileron
already agreed) and flipping the sign of the trim-elevator solve. (2) The
gated complementary AHRS lost bank reference in a sustained turn: a
coordinated fixed-wing turn sits near 1 g (specific force stays ~body-down) so
the *magnitude* gate never closes, yet the accel still cannot see bank and,
without centripetal compensation, drags roll toward level. Body rate is the
reliable "maneuvering" signal, so the rate gate was tightened from full-trust
< 30 dps / zero-trust > 120 dps to **< 5 dps / > 25 dps** -- a 10-25 dps turn
now coasts on the gyro. `kMagBandG` also tightened 0.10 -> 0.05 g (throttle
surge is 0.3-0.5 g, still fully rejected). Built-in `--check` AHRS errors
unchanged (roll 0.9 deg, pitch 3.9 deg). KNOWN LIMITATION: sustained-turn
attitude still under-reads true bank by ~4-5 deg (pure gyro integration, no
kinematic/centripetal term) -- ASSIST bank authority is deliberately modest
until GPS-velocity centripetal compensation lands with the Phase 5 nav filter.

**2026-09-07 — Mode manager in `main_stm32`.** `task_control` now resolves the
ch7 request through `resolve_mode()`: FAILSAFE -> MANUAL (not latched, recovers
with the link); IMU not ready -> MANUAL and **latch `s_assist_lockout`**
(demotion is downward-only and, for a sensor fault, permanent until reboot --
CLAUDE.md); ASSIST also requires `bias_ready` (not latched, just waits); an
AUTO (high) request resolves to MANUAL for now but the `MODE` line still shows
`req=AUTO` so the mismatch is never silent. `set_mode()` calls `enter(s_out)`
on the incoming mode for bumpless hand-off and emits `MODE_CHANGE,<name>`.
`ModeInput` is built from `ahrs::` angles, the LPF'd bias-corrected body rates
(`g_g*`), and an airspeed proxy = GPS ground speed floored at 10 m/s (no
pitot; unreliable in wind). Log frame `mode` is now `s_mode_cur`. MANUAL and
`core/failsafe` paths untouched.

**2026-09-07 — ASSIST stabilizer integrators gated on a "flying" latch.**
Bench observation: board level and still, sticks centred, armed, in ASSIST --
the aileron output sat ~215 us off centre (1285 vs 1500). Cause is textbook
integrator windup against a stuck plant: a ~1 deg static AHRS roll (table not
level + accel zero bias) makes the angle loop demand a small constant body
rate; the airframe is clamped to the table so the rate error never clears and
the rate-loop I term winds to +/-i_max (0.4). Harmless in flight (the aircraft
rolls, the error clears) but a real off-centre surface at launch. Fix:
`Pid::set_integrator_enabled(bool)` (I still contributes to the output, just
stops accumulating) fanned out through `RateController` and driven from a new
`ModeInput::allow_integrators`. `main_stm32` sets it from an `s_flying` latch:
false until `armed && (throttle > 0.75 || GPS ground speed > 8 m/s)` -- set clear
of taxi -- then held
true until disarm. On the false->true edge `ModeAssist` re-presets the
integrators so they resume bumplessly from a clean state, not a wound rail.
MANUAL ignores the flag. `MODE` line gains `flying=`; GUI shows "Stab
integrators: frozen/active". 2 new native assertions; SITL `--assist-check`
unchanged (sim always flying). NOTE: the residual ~1 deg roll with I frozen is
an accel-cal / mounting error, not a control bug -- 6-point accel calibration
is still deferred (needs `core/params`).

**2026-09-07 — CRSF telemetry TX (FC -> handset).** The flying aircraft has no
USB; the pilot's picture of the FC comes back over the CRSF uplink to the
TX16S. `drivers/crsf` gained frame builders -- `send_attitude` (0x1E,
pitch/roll/yaw in rad*1e4), `send_gps` (0x02, lat/lon 1e7, ground speed,
heading, alt, sats), `send_vario` (0x07, cm/s), `send_battery` (0x08; built but
NOT sent -- no voltage/current sensor on the board yet, see below), and
`send_flight_mode` (0x21, ASCII: "MANUAL"/"ASSIST"/"AUTO", trailing `*` when
disarmed, `!FS` on failsafe, `!LOCK` when ASSIST is latched-out by an IMU
fault). All payloads big-endian per CRSF; CRC-8/DVB-S2 over type+payload, same
as the RX parser. Frames are written whole-or-not-at-all against
`hal::uart_write_space` (new hal call) so a full TX ring drops a frame instead
of desyncing the receiver. `task_crsf_tx` at 10 Hz: attitude + vario every
tick, GPS and flight-mode interleaved (~2.5 Hz). Gated on `crsf::receiving()`.
Climb rate is a 0.7 Hz-filtered dAGL/dt computed in `task_bmp` (`g_climb_mps`).
Debug stream gains `EST,...,climb_mps=` and `CRSF_STAT,...,telem_tx=`.
Verified: frame bytes/CRC checked against the CRSF spec for all five types.
NEXT for telemetry: a pack-voltage divider on a spare ADC pin (e.g. PC4 =
ADC1_IN14) -> `send_battery` -> EdgeTX low-battery callouts; optionally a
temperature frame (0x0D) for the ESCs/IMU. Course-over-ground for the GPS
heading field needs NMEA VTG (or the UBX-NAV-PVT switch in Phase 5).

**2026-09-08 — Scheduler priority + loop-timing hardening (flight-readiness
pass).** Live `SCHED` capture from the board showed `task_control` itself
healthy (43 us mean vs 2500 us budget) but `worst_pass_us = 104 ms`: the single
cooperative pass was being stalled by non-critical tasks that also sat *ahead*
of `bmi`/`control` in registration order -- `bmi_retry` calling the ~ms Bosch
re-init at 1 Hz (102 ms), `task_debug` doing a blocking `f_sync` + a 64-byte
hex dump + a print pile (43 ms), SD sector-write tails in `log_flush` (34 ms).
Changes:
- `sched::add(..., critical=false)`. Critical tasks (`bmi`, `control`) run at
  the top of every pass and are re-serviced after any non-critical task that
  ran > 500 us, so a slow logger / debug / SD write delays the rate loop by at
  most one such task, not their sum.
- `sched::reset_stats()` + a USB `RESET_STATS` command, so a clean measurement
  window can be taken after boot settles. Also added `REBOOT_BL` (jump to the
  DFU bootloader without the BOOT0 button) via the same tiny `task_cmd` reader.
- IMU brought up in `setup()` with a bounded retry (was: first scheduler pass).
  `task_bmi_probe` @ 2 Hz does a cheap chip-id read and only runs the full
  `begin()` if the part answers -- and never once `s_flying` is latched (a
  faulted IMU latches MANUAL for the flight anyway; a 100 ms stall in flight is
  worse than a dead AHRS we are not using).
- `f_sync()` moved from `task_debug` into the already-SD-isolated
  `task_log_flush` (~0.5 Hz there).
- All high-rate USB echo (`BMI`/`ATT`/`OUT`/`RC`) moved off the sample/parse/
  control tasks into one `task_stream` @ 20 Hz. `LINK` + GPS bring-up dumps
  moved to `task_debug`; the GPS dumps self-disable after 25 s.
- IMU sample rate 100 -> 200 Hz (== AHRS rate; half the SPI cost of matching
  the 400 Hz rate loop). `ImuPrep::configure` gained `cal_seconds` (default 4)
  so the bias window stays ~4 s regardless of rate.
Registration order is now: bmi*, control*, crsf, gps, crsf_tx, bmi_probe, bmp,
log, log_flush, stream, cmd, debug, sched.

**2026-09-08 — SD write jitter characterised; batched flush + 16 KB ring.**
Follow-on measurement (added `SIM_FLYING 0|1` to force the in-flight `s_flying`
latch on the bench). With the priority pass in place the entire control/estim
path is tiny -- `control` 36 us mean / <=68 us max (2500 us budget), `bmi`
85 us / <=117 us (5000 us budget), ~90 k loop passes/s. The one residual is
`worst_pass_us` ~= 14-16 ms, all of it a single blocking SD sector write: the
fitted card returns most 512 B writes in 1-3 ms but occasionally stalls ~14 ms
on its internal flash-program cycle. That is card-busy time, not transfer time,
so DMA would not remove it -- the real fix is a non-blocking SD write path
(issue one block, return, poll `HAL_SD_GetCardState` next tick). Tracked as the
next E-item; a high-endurance / industrial card also shrinks the tail to 1-3 ms
with no code change. Interim tuning, measured on hardware (control dispatched
>1.25 ms late, per second): 25 Hz/512 B = 4.5, 10 Hz/1 kB = 6.2, 5 Hz/2 kB =
3.4, 1 Hz/8 kB ~= ground-mixed 1.6. Few LARGE flushes win -- a long block trips
the critical re-service so `control`/`bmi` run the instant the write returns,
whereas a stream of ~2 ms blocks just makes `control` quietly late each time.
Settled: `log_flush` @ 1 Hz draining up to half the ring; `core/log_ring`
8 KB -> 16 KB (~3.5 s) so a multi-hundred-ms card hiccup never drops a frame
(RAM 14 % -> 21 %); `f_sync()` only while `!s_flying` (in flight the sectors
still land, the dir entry is refreshed on landing; a hard power loss mid-flight
leaves the data on the card with a stale dir size -> raw-sector recovery).
Net: `worst_pass_us` 104 ms -> ~15 ms, and the rate loop is on-time ~99 % with
the late dispatches bounded and dt-corrected. Flight-safe for MANUAL/ASSIST;
not yet hard-real-time.

**2026-09-09 — TKOFF mode: roll wing-leveller, pitch/yaw/throttle manual.**
Requested for the hardest phase of RC flying. Taking off in ASSIST is a trap:
ASSIST holds pitch *angle*, so centred sticks command 0 deg pitch -- it fights
rotation, then either mushes off flat or drops the nose right after liftoff.
`modes::ModeTakeoff` closes ONLY the roll loop (stick -> clamped bank -> the
ASSIST roll rate PID -> aileron), and passes pitch, yaw and throttle straight
through `mix_manual` exactly as MANUAL. So the pilot rotates and climbs with no
stabiliser on the elevator, but the wings stay level (or a small commanded
bank for crosswind). Bank authority is deliberately small: `max_roll` 0.35 rad
(~20 deg), `max_roll_rate` 120 dps (vs ASSIST's 200), reusing the ASSIST roll
gains (kff 0.006 / kp 0.010 / ki 0.02, i_max 0.4). Same bumpless entry and
ground-integrator-freeze (`allow_integrators`) as ASSIST. Selection: ch7 high
(was the unbuilt AUTO slot) -> low/mid/high = MANUAL / ASSIST / TKOFF; AUTO has
no slot until it exists. Same gates and IMU-fault lockout as ASSIST (the
`s_assist_lockout` latch now covers both stabilised modes). Verified: native
(pitch/yaw passthrough exact, roll centred -> ~0 aileron, full stick -> bank
holds at the 0.35 rad cap in a toy plant); SITL `--takeoff` (roll doublet
levelled and held to <0.1 deg, pitch doublet passes through to the elevator and
is NOT held). `--check` / `--assist-check` unchanged; builds green all envs.
Procedure: take off TKOFF, drop to ASSIST once settled, MANUAL is the bail-out.
A full launch/TKOFF-with-climb-pitch mode (fixed climb attitude + abort) is
still future work and needs the SITL ground model first.

**2026-09-09 — TKOFF bank cap 20 deg -> 10 deg.** You keep the stick centred on
a takeoff; the only reason to command any bank is a crosswind wing-low, which
never needs more than ~10 deg. 20 deg near the ground is a wingtip-strike /
over-bank hazard for no benefit and makes an accidental roll-stick bump matter.
`_max_roll` 0.35 -> 0.175 rad in `main_stm32`, SITL and the header default.

**2026-09-09 - repo renamed STM32FC_Claude -> STM32FC_v2.** GitHub repo and
git remotes (origin + claude both point at it now; main tracks origin/main).
The local working dir is still ~/Desktop/STM32FC_Claude. The old
github.com/zelveron/STM32FC is the prior-generation reference codebase, not
this project. README rewritten to current state in the same commit.

**2026-09-28 — v2.2 board migration and review remediation.**
The user selected the manufactured F407VG board with two BMI270s, BMP581,
BMM350 and SAM-M10Q. J2/UART4 is assigned to ER8; Y2 is confirmed 8 MHz.
Earlier hardware/pin/rate claims above are historical and superseded by
HARDWARE_V2.md. Both IMUs use filtered 400 Hz FIFOs and independent startup
calibration. One feeds the quaternion estimator, with calibrated backup
selection and latched ambiguity on persistent disagreement. No magnetic or
GNSS navigation fusion is claimed yet.

The user selected throttle-cut/healthy-level RC failsafe, with automatic
restoration at current throttle after stable reception if previously armed
and CH5 remains high. CH5 is arm; CH7 selects MANUAL/ASSIST/TKOFF. TKOFF remains
roll assistance with pilot pitch/yaw/throttle. Mode transitions use bounded
surface slew rather than integrator preload. GPS ground speed is no longer
used as airspeed for turn compensation.

SD writes are disabled by default and forbidden after first arming; earlier
claims that scheduler priority made blocking SD safe do not apply. A control
deadline latch and watchdog were added, but physical timing is unverified.
The default build inhibits motors because electrical rework and hardware
acceptance remain outstanding. See CHANGE_REVIEW.md, VALIDATION.md and
FEATURES.md for implementation, evidence and limitations. No firmware was
flashed and no flight-readiness claim is made.


**2026-09-28 - asynchronous recorder, control and telemetry improvements.**
This entry supersedes the interim ground-only SD policy immediately above.
The user authorized the proposed improvements. Runtime logging now writes
session/sequence/CRC sectors through a bounded command/SDIO-DMA state machine
to a FAT32 extent allocated and closed before the scheduler starts. Both IMUs
have raw/filtered 400 Hz diagnostic records; controller diagnostics are 100 Hz.
No filesystem operations or HAL command polling waits run in the flight logger.
Hardware/card timing and power-loss qualification remain pending.

ASSIST now transforms attitude-rate demands into body rates, uses measured
turn kinematics without substituting GPS for airspeed, and combines direct
pilot rudder with bounded transient yaw damping. Commands have acceleration
limits, smooth mode-entry blending, configurable slew and downstream anti-windup.
Integration eligibility clears on explicit disarm/corroborated landing and
persists through glides/failsafe. TKOFF remains roll assistance only.

Gravity innovation/rate rejection and residual-bias learning are more selective.
Measured six-face calibration can be fitted offline and configured per IMU;
the optional notch remains disabled until vibration spectra justify it.
The telemetry scheduler retains all groups, with per-type controls, a byte
budget, prioritized mode changes, fair scheduling and new-fix GPS updates.
TELEMETRY.md inventories both radio directions and receiver-generated statistics.

All three ARM profiles build; 274 C/C++ and 17 Python checks pass. These are
implementation/model results, not evidence of an actual flight or airframe tune.
No board was flashed; motor inhibition remains the default development gate.


**2026-10-08 — BMI270-only firmware and Windows GUI.** The connected card is
identified by the user as dual BMI270. Remove the former BMI323 drivers, vendor
library and build profile; historical entries above describe superseded work.
Default to `v2_bmi270` with magnetometer disabled and motors inhibited. Upload
Bosch's configuration image independently on each SPI bus, allow measurement
startup to settle, then flush the FIFO before accepting paired samples. Expose
per-device initialization errors and persistent GNSS communication/satellite
health in USB telemetry and the standalone Windows x64 dashboard.

**2026-10-09 — BMM350 magnetic yaw and ASSIST heading hold.** The user reports
BMM350 online and requests it as the heading reference with GUI source status.
Keep the existing Bosch acquisition path and selected-IMU AHRS; apply calibrated,
tilt-compensated world-yaw corrections after field/innovation/freshness checks.
Use pre-arm alignment and bounded airborne convergence, with gyro fallback.
Centered roll/rudder captures heading after settling in armed airborne ASSIST;
the heading loop requests at most 15 degrees bank. Pilot input, lost compass,
disarm and RC failsafe release hold. Existing throttle and fault behavior stay
intact. Add YAW_STATUS and GUI source/hold/rejection indicators.

No mounting or hard/soft-iron measurements were supplied. Add an offline
ellipsoid/axis-mapping tool and explicit configuration gate instead of inventing
coefficients. Default v2_bmi270 now enables BMM350 acquisition and retains motor
inhibition. See MAGNETIC_HEADING.md for calibration, gates and limits. No board
was flashed for this change.
