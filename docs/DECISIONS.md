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
