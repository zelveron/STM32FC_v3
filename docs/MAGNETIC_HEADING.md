# BMM350 heading and ASSIST heading hold

BMM350 is the absolute **magnetic north** reference when its installation calibration, field checks and sample freshness pass. The selected BMI270 still propagates the attitude quaternion at 400 Hz. BMM350 provides tilt-compensated yaw corrections from its 25 Hz measurements. This is magnetic heading, not true north, GPS course, position hold or wind correction.

The user reports BMM350 online as of 2026-10-09. This change implements the software path and tests it on the host. No measured compass calibration or axis mapping was supplied, so `src/config/magnetometer.hpp` starts unconfigured. Sensor diagnostics remain enabled, but magnetic correction and heading hold stay unavailable until a measured configuration is installed. Online communication and Bosch factory compensation alone do not establish aircraft heading.

## Acquisition and fallback

1. The existing Bosch driver initializes and compensates measurements in sensor axes. Subtract the measured hard-iron offset, apply the symmetric soft-iron matrix, then rotate sensor axes into body forward/right/down.
2. Remove roll/pitch using the shared AHRS attitude. Compute magnetic yaw with `atan2(-east, north)` in the north/east/down convention. Only world yaw is corrected; the operation preserves roll and pitch.
3. Require a healthy selected IMU, completed gyro calibration, healthy BMM350 driver, finite values, proper mounting rotation and positive-definite calibration matrix. Reject total-field departures beyond 30% of the calibration radius, horizontal field below 8 µT, or pitch where `abs(cos(pitch)) < 0.25`.
4. Qualify observations for at least 500 ms and ten samples, with gaps no larger than 120 ms and adjacent yaw-innovation changes no larger than 15 degrees. Samples older than 200 ms are ineligible.
5. Before the first arming, qualified magnetic yaw can initialize the absolute yaw reference. After first arming, acquisition and recovery use a two-second complementary correction capped at 5 deg/s. Heading hold requires innovation below 5 degrees. Once aligned, innovations beyond 45 degrees are rejected. Such a large disagreement must be investigated; the firmware does not silently accept it as a new reference.

Invalid field, driver failure, stale data or attitude loss immediately remove magnetic eligibility. Gyro propagation continues whenever attitude remains healthy. ASSIST retains its existing roll/pitch control and rudder damping. Recovery requalifies samples and captures a new hold target after settling; it does not return to an old target. Existing IMU-fault, RC-failsafe and motor gates remain in force.

These checks reject gross disturbances, not every magnetic error. A slowly changing, plausible-strength disturbance can remain undetected. Roll/pitch errors also contaminate tilt compensation. The calibration radius is normalized to 50 µT by default; it is not an estimate of the local Earth's field or an independent interference measurement.

## ASSIST behavior

With ASSIST active, fresh aligned magnetic heading, CH5 armed, the existing airborne heuristic active and no RC failsafe, centered roll and rudder sticks (within 5%) request heading hold. Capture occurs after 0.5 seconds with bank below 10 degrees and heading rate below 8 deg/s.

The shortest wrapped heading error requests bank at 0.5 radians of bank per radian of heading error, limited to ±15 degrees. Existing attitude/rate controllers, command shaping and output slew limits apply. This turns the aircraft with its ailerons; pilot rudder remains direct with the existing damper. Pitch and throttle retain their existing pilot controls. No altitude or airspeed hold is added.

Roll or rudder input releases hold immediately. After the turn settles with centered sticks, the current heading is captured. Disarm, loss of airborne eligibility, failsafe, invalid heading or leaving ASSIST clears hold. TKOFF remains a roll-only wing leveler. Defaults live in `control::AssistTuning`; these initial gains have host/model coverage, not measured airframe tuning.

## Measure and install calibration

The supplied `tools/calibrate_mag.py` fits the full hard/soft-iron ellipsoid and selects among 24 proper signed-axis rotations by comparing magnetic fields against gyro-only attitude. It rejects insufficient 3D coverage, degenerate fits, strong inconsistency, ambiguous axis mapping and uncalibrated gyro data. It neither contacts the board nor flashes it.

1. Use a gyro-only yaw build/capture. An older firmware without `YAW_STATUS` is supported; on the new firmware leave `magnetic_calibration` unconfigured. For recalibration, first set its `calibrated` flag false and use that rebuilt image so the compass cannot validate its own axis mapping through fused yaw.
2. Keep propulsion off and the assembly still through the four-second gyro calibration. In the GUI, verify **Gyro calibration: ready**, then select **Record USB**. Use a single uninterrupted boot/session with both `MAG` and `ATT` present.
3. Slowly rotate the installed assembly through all orientations: nose/tail, left/right and top/bottom. Include rotations around all three axes; a flat horizontal circle is insufficient. Keep the capture reasonably short to limit gyro drift, with at least 300 paired samples (usually 45–90 seconds gives better coverage). Keep nearby steel/magnets and moving cables away. Slow motion matters because USB magnetic/attitude records are paired approximately, not by exact sensor timestamps.
4. Stop capture and run from the repository root, using Python with NumPy. NumPy is already available in the Raspberry Pi system Python. For another Python environment, install `tools/calibration-requirements.txt` first.

   ```sh
   python3 tools/calibrate_mag.py /path/to/capture.csv --output build/compass-calibration
   ```

   The output directory must be new. The tool writes `report.json` (fit errors, axis mapping, sample count and capture SHA-256) and `magnetometer.hpp` (firmware configuration). A rejected capture creates no calibration; repeat a better rotation capture rather than bypassing the checks.

5. Review the report and copy the generated header over `src/config/magnetometer.hpp`. Build `pio run -e v2_bmi270` for motor-inhibited bench checks, or deliberately choose the existing `v2_flight` profile for motor-enabled firmware. Installing a header or building does not flash the board. Follow [DFU instructions](DFU.md) when a flash is explicitly intended.
6. Verify magnetic cardinal headings and direction of change, then compare heading while tilting at a fixed direction. Confirm source recovery and field rejection in telemetry. Installed-aircraft interference under representative electrical loads needs measurement before relying on heading hold. Recalibrate after changing mounting or magnetic surroundings.

Calibration is compiled into the firmware. There is no persistent parameter store or in-flight calibration command. The tool does not identify an arbitrary non-orthogonal chip installation: its mounting model is one of the PCB's orthogonal signed-axis rotations.

## GUI and telemetry

The Flight deck labels yaw **MAGNETIC HEADING** with **Source: BMM350 + gyro**, or **GYRO YAW / COASTING** with the fallback reason. It shows heading hold and the captured target only when the firmware reports it active with eligible mode/arming state. The Sensors page shows installation status, corrected field strength, innovation and capture instructions. Older firmware lacking `YAW_STATUS` displays an unavailable source instead of assuming a healthy magnetometer is fused.

The one-second USB status is:

```text
YAW_STATUS,source=BMM350,valid=1,reason=tracking,heading=90.0,mag_heading=90.0,field=50.00,innovation=0.0,configured=1,hold=1,target=90.0
```

`source` is `NONE`, `GYRO` or `BMM350`. `valid` means a fresh aligned magnetic reference suitable for hold, not general IMU validity. Reasons are `disabled`, `setup_required`, `waiting`, `no_attitude`, `field_rejected`, `innovation_rejected`, `qualifying`, `aligning`, `tracking`, `stale`, or `driver_unavailable`. Angles are degrees; headings and target use 0–360, innovation uses ±180. `MAG` still carries uncompensated-for-installation sensor axes for calibration. `MAG_HEALTH` still describes the physical driver.

`ATT`, CRSF attitude yaw and attitude log frames all use the same corrected AHRS quaternion. CRSF GPS course remains ground track; no new radio fields or channel assignments are added. GUI status may lag by the one-second reporting interval and expires after three seconds, while firmware enforces the 200 ms compass timeout on every control tick.

The Sensors page uses `MAG_HEALTH` to distinguish initialization failure, I²C
timeout/busy/NACK, no fresh sample, invalid/out-of-range values and streaming.
Driver failure takes precedence over the heading installation reminder;
`configured` still reports calibration independently. A previously accepted
`MAG` value is not displayed as valid while current driver health is bad.

`MAG_HEALTH` includes `communicating` (a completed measurement read within
200 ms), `reads`, `samples` (accepted measurements), `invalid`, `consecutive`
read failures, `recoveries`, `otp_error` and `bus_status`. The last failed
transport status is retained for diagnosis: 0 none, −1 generic error,
−2 timeout, −3 busy, −4 unsupported, −5 NACK. It is historical after recovery;
use current `stage`, `healthy` and `communicating` to interpret it. Startup
fails if any OTP transfer/status failure occurs, even if Bosch's final OTP
word succeeds. Stage codes are listed in [FEATURES.md](FEATURES.md).

`MAG_DATA,x=…,y=…,z=…,temp=…,raw=x/y/z/t` reports the last measurement read,
including rejected values. Magnetic values are µT, temperature is °C, and
raw values are signed 24-bit counts from that same coherent burst after
removing Bosch's two dummy bytes. This record is diagnostic, not proof of
freshness or valid heading. Only accepted values are published on `MAG`.

`MAG_CHECK,x=…,y=…,err=…,pmu=…,aggr=…,axes=…,st=…` retains the **boot-time**
positive-minus-negative X/Y self-test responses in µT and the configuration
register snapshot. The full self-test threshold is 300 µT per axis, as in
[Bosch's pinned out-of-range API](https://github.com/boschsensortec/BMM350_SensorAPI/blob/3daf377ccaf589319c0d41af192105e9275987a2/bmm350_oor.h).
Expected registers are `err=0,pmu=40,aggr=54,axes=7,st=0` (decimal): normal
mode, 25 Hz with eight-sample averaging, XYZ enabled and user self-test off.
The Bosch self-test includes a magnetic reset and runs only before scheduler
startup. A failed numerical response allows diagnostic reads but rejects
measurements; transfer/cleanup/configuration failures prevent acquisition.
The GUI shows the startup X/Y responses separately from the current field.

## Software verification

Run `python3 tools/test_host.py` on Linux or `tools/test_host.ps1` on Windows, plus `python3 tests/mag_calibration.py` and the GUI tests. Coverage includes tilt compensation, calibration/axis mapping, angle wrap, field/innovation rejection, stale/failed sensors, bounded airborne acquisition, shared application/AHRS integration, gyro drift correction, pilot override and closed-loop heading recovery in the simplified aircraft model. These checks do not constitute hardware or flight qualification.

Reference conventions: [Bosch BMM350 mounting and integration guidance](https://www.bosch-sensortec.com/media/boschsensortec/downloads/handling_soldering_mounting_instructions/bst-bmm350-hs000.pdf) and [NXP tilt-compensated compass derivation](https://www.nxp.com/docs/en/application-note/AN4248.pdf).
