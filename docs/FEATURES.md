# Flight-controller software feature catalog

This catalog describes the **implemented STM32FC_v3 development firmware for PCB v2.2**. It distinguishes usable software behavior from physical validation: the code has host tests and STM32 builds, but the manufactured board, servo directions, timing and flight gains remain unqualified. Default motor arming is inhibited. Detailed setup is in [TX16S_SETUP.md](TX16S_SETUP.md); fusion design is in [SENSOR_FUSION.md](SENSOR_FUSION.md).

## Power-up and startup checks

- **Defined startup outputs:** six surfaces are initialized to their configured centers and both ESC channels to minimum pulse before sensor initialization. There is no automatic servo sweep at calibration completion.
- **No USB-host wait in the flight application:** connecting a computer is not required for startup.
- **Board-specific power control:** enables the BMP581/GPS supplies and releases GPS reset using the actual v2.2 GPIOs.
- **Per-device initialization results:** USB reports BMI, BMP and magnetometer initialization. MANUAL does not require an IMU to initialize successfully.
- **Bosch BMI270 configuration upload:** each chip receives the official configuration image and independent accel/gyro/FIFO settings.
- **Startup FIFO cleanup:** samples that accumulated during other devices' initialization are discarded once before normal sampling.
- **Four-second stationary gyro calibration:** each IMU independently collects 1600 stationary samples at 400 Hz. The four seconds start with valid samples, not at the instant the battery is connected.
- **Calibration restart on movement:** the window restarts for excessive gyro rate, excessive per-axis spread, invalid acceleration magnitude or invalid data/timing. Current gates are less than 3 deg/s per axis, no more than 4 deg/s spread over the window, and 0.9-1.1 g acceleration magnitude.
- **No calibration during a flight session:** calibration is disabled after the first successful arming until restart. Bias is not relearned from normal flight motion.
- **Stabilization waits for calibration:** ASSIST/TKOFF remain unavailable until the selected IMU is calibrated and the attitude estimate is healthy. MANUAL remains available.
- **Explicit development gate:** default `FC_FLIGHT_ENABLED=0` permits sensor/surface testing but prevents motor arming. This must be changed deliberately for motor testing after board repairs.

## IMU processing and attitude estimation

- **Two independent BMI270 drivers:** U4 on SPI1 and U7 on SPI2, with individual device state, bias calibration, low-pass filters and sample freshness.
- **400 Hz paired accel/gyro FIFO processing:** every returned sample is processed in order; no newest-sample-only decimation.
- **Sensor-side filtering:** accel/gyro filtered FIFO data, +/-8 g acceleration and +/-2000 deg/s gyro ranges.
- **Software filtering:** configurable 30 Hz gyro and 15 Hz acceleration first-order LPFs; optional validated-frequency gyro notch, disabled until measured.
- **Explicit mounting rotation:** both IMUs are mapped to aircraft forward/right/down axes, with a proper-rotation check. Default orientation is components up, nose toward the Y2/left side of the unrotated PCB view.
- **Specific-force convention correction:** accelerometer sign is converted to the estimator's gravity-like convention; level stationary input is about +1 g on Z.
- **Quaternion attitude:** roll, pitch and relative yaw are derived from a normalized quaternion, avoiding Euler integration singularities.
- **Gravity correction with trust gating:** Mahony-style accelerometer correction weakens during acceleration or rotation; gyro integration continues when acceleration is not trusted.
- **Residual gyro-bias learning:** additional quiet-window, magnitude/rate/innovation gates and +/-3 deg/s bound protect learning from maneuvers. Gravity correction includes a direction-innovation gate and rejects body rates at/above 3 deg/s.
- **Primary/backup selection:** U4 is preferred; a healthy calibrated U7 can take over when U4 fails. Attitude is retained and old residual bias cleared. Selection does not automatically return to U4 during that boot.
- **Two-sensor disagreement detection:** gyro-vector difference over 20 deg/s or acceleration-vector difference over 0.5 g for 100 ms latches ambiguity. Two plausible disagreeing sensors cannot be voted on, so stabilization is inhibited.
- **Sensor health checks:** identity, error register, FIFO parsing/counts, excessive backlog, raw clipping, progressing sensor time and 20 ms sample freshness.
- **Estimator numeric checks:** invalid numbers or invalid time steps invalidate/reset the estimate instead of allowing a nonfinite quaternion to propagate.
- **Explicit limitations:** no synchronized sample clocks, independent dual EKFs, measured temperature compensation, absolute yaw, airspeed fusion or GNSS velocity fusion yet.

- **Six-face calibration support:** independent body-axis offset/scale for each IMU, compile-time bounds, and an offline stationary-face fitting tool. Actual calibration values still need measurement.

## Pilot controls and mixing

| Input | Behavior |
|---|---|
| CH1 roll | Right roll command is positive |
| CH2 pitch | Nose-up command is positive |
| CH3 throttle | 0-100% pilot demand |
| CH4 yaw | Right yaw command is positive |
| CH5 | Arm/disarm |
| CH7 | Low MANUAL, middle ASSIST, high TKOFF |
| CH6 / CH8 | Reserved, unused |

- **Normalized stick ranges and deadband:** configurable RC-channel conversion maps receiver pulse equivalents to normalized commands; centered axes have a small deadband.
- **Twin-aircraft mixer:** two ailerons, two elevators, rudder, independent nosewheel output and two ESCs.
- **Per-output endpoints/center/reversal:** mechanical installation can be corrected without changing the IMU frame or pilot control convention.
- **Motor range validation:** reversed ESC pulse ranges and invalid endpoint ordering fail a compile-time configuration check.
- **Differential-thrust scaffolding:** the mixer has parameters, but the current default differential-thrust gain is zero. There is no transmitter-assigned differential-thrust feature enabled by default.
- **No automatic flaps/elevon/VTOL mixing:** those airframes/functions need explicit additional implementation and validation.

## MANUAL

- Direct roll, pitch, yaw and throttle demand through the mixer.
- Available without GPS, barometer, magnetometer or IMU.
- Immediate return to pilot surface commands when selected or when stabilization is demoted.
- Arming, failsafe and motor-inhibit rules still apply; MANUAL does not bypass them.
- Servos operate while disarmed for checks; disarmed ESCs remain at their configured minimum pulse.

## ASSIST

- Roll stick commands bank angle, approximately **+/-40 degrees**.
- Pitch stick commands pitch angle, approximately **+/-26 degrees**.
- Centered roll/pitch sticks request level roll and configured pitch trim (initially zero).
- Pilot rudder remains direct, with bounded yaw damping that releases sustained natural turns; full rudder overrides the damping contribution. No heading hold or verified sideslip control.
- Throttle stays under direct pilot control.
- Wrapped attitude error generates Euler-rate requests transformed to body rates, including bank/pitch turn coupling. Roll/pitch demand acceleration limits are 600/400 deg/s²; rate controllers feed the common mixer.
- PID controllers include feedforward, output/integrator limits, anti-windup and filtered measurement derivative support. Default derivative gains are not an airframe-tuned feature.
- Mode entry uses a 0.25-second smooth blend from previous outputs. A configurable 12 normalized units/s command slew ceiling follows; applied-command feedback prevents integration farther into downstream limits. Throttle bypasses smoothing.
- Integration eligibility uses debounced throttle/GNSS/barometer evidence, persists through glides/failsafe, clears on explicit disarm or corroborated landing, and clears integrators while ineligible. See [control details](CONTROL_AND_TUNING.md); it remains a heuristic.
- Measured body rates provide turn geometry without a pitot. A bounded airspeed-scaling/coordination input exists but remains inactive on this hardware; GPS speed is never substituted for airspeed.
- No altitude, airspeed, position, track or heading hold; no envelope or stall protection.

## TKOFF takeoff assistance

- **Roll-only wing leveling**, with centered roll stick requesting level wings.
- Roll stick can request about **+/-10 degrees bank** for limited crosswind correction.
- Pilot retains pitch/elevator control for rotation and climb.
- Pilot retains yaw/rudder/nosewheel and throttle control.
- Only ailerons use the stabilized surface slew; pilot pitch/yaw/throttle remain direct.
- Uses the calibrated healthy attitude requirement and fault lockout policy.
- No launch detector, automatic throttle application, timed acceleration, pitch schedule, climb-altitude completion or automatic mode transition. The pilot chooses when to switch to ASSIST or MANUAL.

## Arming and motor authorization

- Disarmed at boot.
- Initial arming requires a usable link, CH5 observed low, then a rising transition with throttle at or below 5%.
- Booting with CH5 high cannot arm automatically.
- Raising CH5 with throttle high does not arm later just because throttle is lowered; cycle CH5 low/high again at low throttle.
- CH5 low immediately removes motor authorization.
- Both motors are forced to minimum during disarm, RC failsafe, output-configuration failure or latched control deadline/watchdog fault.
- Automatic RC recovery retains only authorization from an aircraft armed before that outage; it cannot authorize a previously unarmed aircraft.
- A new boot, disarm on recovery, or permission fault clears pending automatic recovery.

## Failsafe and fault handling

- **RC packet freshness:** only a CRC-valid complete channel frame refreshes pilot-control freshness; link-statistics/unknown traffic cannot keep stale sticks alive.
- **RC timeout:** 200 ms after the last valid channel frame, then minimum motor demand. Actual physical response includes receiver detection latency and PWM/actuator delay.
- **Healthy-sensor RC failsafe:** commands level roll/configured pitch trim with yaw damping and throttle cut.
- **Unhealthy-sensor RC failsafe:** centers surfaces with throttle cut.
- **Stable-link recovery:** requires 300 ms usable reception.
- **User-selected automatic throttle restoration:** previously armed + CH5 high restores the current throttle demand after the stable window, without requiring an idle-throttle rearm or applying a throttle ramp.
- **Attitude-fault lockout:** loss of stabilization eligibility while ASSIST/TKOFF is active demotes to MANUAL and latches lockout until restart; an ambiguous dual-IMU result cannot silently re-enable stabilization.
- **Control-gap detection:** a control interval greater than 20 ms after first arming latches motor inhibit until restart.
- **Independent watchdog:** approximately 100 ms nominal timeout; refreshed only after critical tasks have completed recently. It contains a stuck cooperative task, but hardware reset/ESC pulse-loss behavior must be tested physically.
- **Watchdog-recovery startup:** skips slow sensor/storage initialization and keeps motors inhibited, while permitting RC surface passthrough. It does not resume the previous flight automatically.
- **Numeric containment:** nonfinite surface commands map to center; nonfinite motor commands map to minimum. PID invalid input/time-step handling resets and returns zero.
- **Bounded parsers:** CRSF/GNSS/USB processing has byte/length limits; partial CRSF packets time out and overlong USB command lines are discarded as a whole.

## Barometer and GNSS

- BMP581 normal operation at **50 Hz**, with pressure/temperature oversampling and explicit data-ready enable/polling.
- Pressure/temperature finite/range checks; health expires after 200 ms without new valid pressure.
- Standard-atmosphere pressure altitude and relative altitude from pre-arm ground pressure.
- Ground pressure reference freezes at first arming and survives RC loss and subsequent disarming within that boot.
- Filtered climb/descent rate for telemetry.
- SAM-M10Q NMEA acquisition starting at **38400 baud**, with bounded passive baud discovery and no receiver-side baud rewrite.
- GGA position/fix/satellites/MSL altitude and RMC ground speed/course.
- Checksum, field, hemisphere and coordinate-range validation with double-precision geographic parsing.
- Explicit invalid-fix handling and two-second navigation freshness expiry.
- GNSS is telemetry plus the integrator-enable heuristic; it does not navigate the aircraft.

## Magnetometer

- BMM350 official Bosch initialization, compensation, 25 Hz acquisition and ready polling.
- Finite-value checks and 200 ms health expiry.
- Raw aircraft-uncalibrated magnetic axes over USB for future calibration/testing.
- Not fused into heading until the board supply is fixed and mounting, hard/soft-iron calibration and motor-interference rejection are validated.

## Outputs, timing and storage

- Eight logical PWM outputs on the board's actual timer-capable pins; ESCs are on physical SERVO8 and SERVO9.
- Physical SERVO7/PD11 is deliberately unused because it has no hardware timer PWM. SERVO10 is spare/inactive.
- Default **50 Hz**, 1000-2000 us servo/ESC pulse range; per-timer-group rate configuration.
- 400 Hz RC/IMU/control critical tasks; 50 Hz GNSS polling; 100 Hz barometer polling; 50 Hz magnetometer polling; 20 Hz high-rate USB stream.
- Cooperative scheduler with per-task minimum/mean/maximum execution times, overruns and worst-pass diagnostics. It is not a preemptive real-time kernel.
- 32 KiB RAM log ring with whole-record drop counting, CRC-protected records and session/sequence/CRC-protected 512-byte sectors.
- Asynchronous SDIO DMA logger enabled by default: boot-only contiguous FAT32 preallocation, bounded runtime command/transfer/card-ready state machine, no runtime filesystem access. Real-card qualification remains pending.
- Card errors/timeouts latch logging off; the file extent bounds every write. Decoder rejects unwritten/corrupt tails. Both IMUs log raw/filtered data at 400 Hz, controller diagnostics at 100 Hz, flight state at 50 Hz. See [logging](LOGGING.md).
- Nonblocking USB output queue with drop accounting; no requirement that a host keep up.

## Ground telemetry and tools

- ER8-to-TX16S CRSF telemetry: attitude, vario, valid GPS and actual mode/fault label. Per-group enable/rate controls, byte budget, fair scheduling, new-fix GPS updates, prioritized mode changes and queue diagnostics. [Complete inventory](TELEMETRY.md).
- Mode labels distinguish bench builds, disarmed/armed modes, failsafe, stabilization lockout and timing faults.
- Invalid/stale sensors do not generate new normal telemetry for that sensor; the radio must still handle stale previously received values.
- USB sensor, receiver, actuator, calibration, dual-IMU health, active/requested mode, logging and timing diagnostics.
- Ground commands: reset scheduler statistics and enter ROM DFU using `dfu` (legacy `REBOOT_BL` alias). Armed/flight-state entry is rejected; after arming, require fresh RC, completed failsafe recovery, CH5 off and idle throttle. The GUI releases the serial port and verifies the DFU USB identity. Reset-first entry works with the watchdog; see [DFU guide](DFU.md). The old simulated-flight override remains removed.
- Python GUI updated for dual-IMU status and stale attitude/barometer/GPS expiry; log decoding/plotting tools retained. Additional tags can be viewed with a serial terminal.
- Repeatable host regression suite plus nominal and assisted-flight simulation; actual Bosch BMI270 API exercised against an SPI/FIFO emulator.

## Planned or still missing

Persistent parameters; measured six-face acceleration calibration and magnetic calibration; temperature compensation; independent navigation estimator instances; exact sensor timestamp alignment; GNSS velocity fusion; airspeed measurement; battery/current monitoring; real-card logging qualification; black-box fault breadcrumbs; dynamic tuning; automatic launch; altitude/track/airspeed hold; loiter; waypoints; return-to-home; landing assistance; MAVLink/QGroundControl support; independent flight-control redundancy.

Passing host tests does not establish real sensor orientation, receiver failsafe behavior, servo direction, aerodynamic stability or stall margin. Those require the [hardware and airframe acceptance work](VALIDATION.md).
