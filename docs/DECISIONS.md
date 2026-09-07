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
