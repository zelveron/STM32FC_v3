# Roll, pitch, assists and tuning

These are implemented control improvements with host/SITL coverage. The initial gains are not tuned to the physical aircraft, and the simulator is not an identified model of its aerodynamics or EMAX servos.

## ASSIST

Roll stick requests bank within +/-0.70 rad (40 deg). Pitch requests pitch within +/-0.45 rad (26 deg), with a configurable pitch-trim offset and the same final limits. Neutral pitch means configured trim, initially zero; it does not automatically mean level flight. Throttle remains direct pilot demand under the existing arming/failsafe rules.

The outer loop converts Euler roll/pitch error into Euler angle-rate requests, wraps roll error across +/-pi, then transforms requests into body rates. Holding constant Euler pitch during a turn requires nonzero body pitch rate. With no pitot, measured heading rate supplies this kinematic coupling. Near the Euler singularity (cos(pitch) below 0.25), heading-rate compensation is suppressed. ASSIST is not an aerobatic/inverted-flight controller.

Rudder is direct pilot input plus bounded yaw damping. The no-pitot damper follows a 0.5-second low-pass heading-rate baseline, damping transient departures while releasing sustained natural turns. Its maximum addition is +/-0.25 normalized command, reduced toward zero as pilot rudder reaches full stick. This is turn-compatible damping, not proven zero-sideslip control. No sideslip sensor is fitted, and yaw is not held to an absolute heading.

An explicitly valid measured airspeed input enables a coordinated-turn reference and bounded speed scaling, with reference 18 m/s and accepted range 8-50 m/s. The normalized output scale is clamped to 0.5-2.0. Production firmware sets `airspeed_valid=false` because this board has no fitted/validated pitot input; **GPS ground speed never enables this feature**. These are development bounds, not measured stall/cruise speeds. A future pitot integration needs calibration, freshness checks, failure behavior and retuning across the envelope.

## Command shaping and outputs

Desired roll and pitch body rates are acceleration-limited to initial values of 600 and 400 deg/s². Controllers reset on mode entry and the shaping state starts at measured body rate. Stabilized outputs blend from the preceding mode with a 0.25-second smooth transition. A separate configurable command slew ceiling, initially 12 normalized units/s, bounds remaining changes. It is a software ceiling, not a claim about physical servo speed.

After mixing, blending and slew limiting, rate PIDs receive the applied command. An integral update that would push farther into that downstream limitation is undone; integration that unwinds the limit remains possible. Internal output-rail limiting still applies. This feedback represents commanded surfaces, not measured servo position. The 50 Hz PWM hold and mechanical servo lag still need actual measurement.

## Integration eligibility

The old reboot-long 75%-throttle latch is removed. While armed, integration becomes eligible after 500 ms of continuous evidence: throttle above 25%, valid GNSS groundspeed above 8 m/s, or healthy barometric climb/descent magnitude above 1.5 m/s. Eligibility persists through gliding and RC failsafe. Explicit disarm clears it. A landing can clear it after 3 seconds with throttle below 10%, fresh GNSS speed below 2 m/s, healthy vario magnitude below 0.3 m/s and total body rate below 3 deg/s. When ineligible, integrators are cleared and held at zero.

This remains a heuristic. It can misclassify taxi, strong headwind hover relative to ground, or unusual launches. GNSS is not mandatory for takeoff eligibility, but automatic landing recognition needs it; explicit CH5 disarm remains the reliable ground reset. Flight-session pressure reference and fault latches remain independent and are not cleared by this heuristic.

## TKOFF

TKOFF stays a roll-only wing leveler with approximately +/-10 deg bank authority. It now uses wrapped roll error, bank/pitch body-rate coupling, acceleration-limited roll demand, smooth entry and applied-command anti-windup. Pitch, rudder and throttle remain directly controlled by the pilot. There is no automatic launch detection, throttle ramp, climb schedule or automatic transition to ASSIST. Those would require a separately specified launch/abort policy and verified airspeed/energy behavior.

## Estimator and filter changes

Gravity correction now considers acceleration magnitude, body rate and the direction disagreement between measured acceleration and predicted gravity. Rate trust is full below 1 deg/s and zero at 3 deg/s; innovation trust tapers from full at 10 deg to zero at 25 deg. Gyro propagation continues while gravity is rejected. Bias learning requires at least one second of quiet, sufficiently trusted data, less than 3 deg/s total body rate, less than 5 deg innovation and acceleration within 0.02 g of gravity. Learned residual bias is capped at +/-3 deg/s per axis.

The stricter maneuver rejection reduces turn-induced gravity errors but spends more time integrating gyros without correction. Gyro drift and slow accelerations remain limitations without navigation/airspeed aiding. It is still one Mahony attitude filter, not an EKF or two independent estimators.

Each IMU has configurable accelerometer offset/scale, software LPFs and an optional gyro notch. Current LPFs remain 30 Hz gyro / 15 Hz accel, first order. The notch is disabled (`gyro_notch_hz=0`) until measured; enabled settings are checked against the 400 Hz sample rate and supported Q range. No arbitrary notch frequency or stronger low-pass filter has been imposed.

## Measure before tuning

1. Verify electrical corrections, mounting signs, surface directions and PWM timing first. Read [VALIDATION.md](VALIDATION.md).
2. Capture stationary six-face means independently for each IMU. Exported raw acceleration precedes stored calibration. `tools/calibrate_accel.py` requires at least 100 samples per face and rejects movement/gross misalignment. Put the resulting body-frame offset/scale into `config::accel_offset_g` and `accel_scale`; rebuild and verify all six faces. No automatic flash-parameter store exists yet.
3. Qualify [asynchronous logging](LOGGING.md), then collect stationary and motor-running raw/filtered traces for both IMUs. Preserve contiguous frame sequences. Use `export_diagnostics.py` and `analyze_imu.py`; compare spectra, clipping and actuator demand. The 400 Hz stream cannot resolve vibration above 200 Hz.
4. Tune roll and pitch separately. Establish trim and feedforward, then proportional tracking, then modest integral correction. Add derivative only if measured damping needs justify its noise and delay. Default D remains zero.
5. Check 50 Hz output hold, real servo delay, saturation recovery, slow/fast flight, long turns, gusts, mode changes, RC recovery and IMU failover. No specific physical-flight gain recommendation has been inferred from the toy model.

See [TELEMETRY.md](TELEMETRY.md) for the complete radio inventory. GNSS/airspeed fusion, thermal bias characterization, magnetic calibration/fusion, dynamic notches, autonomous launch and navigation remain future work requiring new measurements and validation.
