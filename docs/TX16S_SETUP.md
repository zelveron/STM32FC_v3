# TX16S MK3 MAX + ER8 setup

This guide configures the transmitter for **this firmware**, using the ER8 as a serial CRSF receiver. The FC drives the aircraft's servos and ESCs. Version-specific menu labels may differ: record your EdgeTX, transmitter-module ELRS and ER8 ELRS versions before starting. No transmitter model file has been generated because those installed versions and switch preferences were not supplied.

First complete the [board corrections and wiring](HARDWARE_V2.md). For all bench procedures remove propellers or disconnect the EDF motors from the ESCs. The user's chosen RC-recovery policy can automatically restore throttle.

## 1. Receiver and radio link

1. Use the ER8's **dedicated serial connector**. With power off, connect ER8 **TX → `MCU_RX_PI_TX` / J2 pin 2 / PA1**, ER8 **RX → `MCU_TX_PI_RX` / J2 pin 3 / PA0**, and ground to J2 pin 1. `PI` is the PCB's old Raspberry Pi naming; follow the **MCU_RX/MCU_TX** direction. Both signal wires are needed for channels into the FC and FC telemetry back to the handset. Supply receiver power separately from a suitable regulated source. J2 has no power pin.
2. Set the ER8 serial protocol to **CRSF**, uninverted, full duplex, **420000 baud / 8N1**. Use the receiver Web UI or available ELRS Lua receiver settings. Do not select SBUS, inverted CRSF, or MAVLink. If a firmware version exposes baud separately, verify it matches.
3. The ER8 has its own serial port: do not follow instructions for other PWM receivers that convert PWM outputs 2/3 to a serial port. See the [official PWM receiver guide](https://www.expresslrs.org/hardware/pwm-receivers/).
4. Use compatible ELRS major versions and the same binding phrase on the active TX module and ER8, or their supported binding procedure. Select the correct **2.4 GHz ER8** receiver target when updating. This does not change EdgeTX itself.
5. In the EdgeTX model's RF settings, enable the ELRS module you actually use with **CRSF** and turn the other RF module off. Internal ELRS is the normal choice if fitted. Receiver-to-FC baud is separate from handset-to-TX-module baud; a 400k handset setting is not a request to set the ER8-FC connection to 400000.
6. A reasonable starting radio profile is **100 Hz Full Res, 8 channels**, if offered by your matched module/receiver firmware. Alternatively use a supported 150 Hz Hybrid profile; CH5 remains two-position and CH7 still has enough positions. Keep telemetry enabled; start with 1:4 if available, then measure reception and telemetry performance. These are radio packet rates, not servo rates. Follow the [ELRS switch-mode guide](https://www.expresslrs.org/software/switch-config/) for version-specific choices.

This firmware detects loss from missing valid CRSF channel frames. It cannot detect an ER8 configured to endlessly transmit held channel values as if they were live. Verify actual RF-loss behavior with the receiver and firmware version you use; the ER8 PWM-output failsafe settings alone do not prove serial CRSF behavior.

## 2. Create a dedicated EdgeTX model

Create a new model named, for example, `STM32FC-v3`. Start with simple AETR control channels. Avoid airplane-wizard mixes that already split two ailerons/elevators: the flight controller handles the physical outputs.

Receiver CH1/2/3/4 carry roll/pitch/throttle/yaw. On this aircraft, plug the corresponding devices into **SERVO1/2/3/4**; the opposite aileron goes to **SERVO6**, which the FC reverses. CH5 still arms the motor and CH7 selects mode. Physical SERVO5 stays centered; SERVO8/9 stay at idle.

Use the following mapping. SF and SC are example physical switches; choose an accessible two-position switch for arm and a three-position switch for modes, then verify the channel monitor. Do not assume a switch's physical up/down position equals a particular channel sign.

| Channel | Mix source | Name | Required behavior at the FC |
|---|---|---|---|
| CH1 | Aileron input | Roll | Center 1500; right roll command increases value |
| CH2 | Elevator input | Pitch | Center 1500; **pull back / nose-up command increases value** |
| CH3 | Throttle input | Throttle | Low stick about 1000; high stick about 2000 |
| CH4 | Rudder input | Yaw | Center 1500; right yaw command increases value |
| CH5 | Two-position switch, e.g. SF | Arm | OFF about 1000; ON about 2000 |
| CH6 | None | Spare | Unused by firmware |
| CH7 | Three-position switch, e.g. SC | Mode | About 1000 = MANUAL; 1500 = ASSIST; 2000 = TKOFF |
| CH8 | None | Spare | Unused by firmware |

In **Mixes**, edit each channel, select its input/switch source and set weight to +100 as a starting point. Verify direction, especially elevator: use the sign needed to make back stick increase **CH2**, rather than assuming the radio's raw elevator sign. Use centered trim/subtrim and ordinary -100/+100 limits initially. ELRS may show approximately 988/1500/2012 us at full transmitter travel; the FC clamps its control range to 1000-2000 us.

The FC arm threshold is **CH5 >1700 us**. Mode thresholds are **CH7 <1333 MANUAL**, **1333-1666 ASSIST**, **>=1667 TKOFF**. Mode is selected by the received value, not a named EdgeTX flight mode.

Use output reversal in the FC configuration to correct an individual servo's mechanical direction. Transmitter stick direction must remain consistent with the table in all modes. Set the radio's throttle and switch warnings to low throttle, arm OFF and MANUAL at startup. Keep trim at zero during stabilization checks; roll/elevator trim becomes an attitude command in ASSIST, not merely a servo trim.

### ExpressLRS 4 arming setting

If the RF page exposes **Arm using**, choose **CH5** for this setup. In ELRS 4 with newer EdgeTX, `Arm using Switch` can replace the CH5 mix in non-Full-Res modes; a correct-looking Mixes page alone may therefore be insufficient. If using that alternate setting intentionally, the selected physical switch and received CH5 must agree. The [official ELRS radio preparation guide](https://www.expresslrs.org/quick-start/transmitters/tx-prep/) explains this behavior.

Verify the **received** `RC` values over USB after configuring the radio, including after changing packet/switch modes. Confirm CH5 really goes low/high and CH7 has all three positions.

## 3. Discover telemetry

The complete [radio inventory](TELEMETRY.md) distinguishes transmitter controls, FC return telemetry and ELRS-generated link statistics. Current maximum generation targets are attitude/vario 10 Hz, GPS at most 2.5 Hz on new fixes, mode 2.5 Hz plus prioritized changes. The FC uses a configurable byte budget; actual handset refresh depends on your ELRS profile and reception. No telemetry groups were removed by this update.

With the FC and receiver running and the RF link established, open the model's **Telemetry** page, select **Discover new sensors**, wait for the available values, then stop discovery. Create widgets for flight mode, attitude, receiver link quality, GNSS status and vario. See the [EdgeTX telemetry manual](https://manual.edgetx.org/color-radios/model-settings/telemetry).

| Data | Meaning / limitation |
|---|---|
| FM | Actual permitted mode or fault label, not simply switch position |
| Pitch / Roll / Yaw | Radians in the CRSF packet, displayed according to EdgeTX; yaw is unreferenced and drifts |
| Vario / VSpd | Filtered derivative of relative pressure altitude; no altitude-hold controller |
| GPS position, satellites, altitude | Fresh valid GNSS fix; altitude is GNSS MSL altitude |
| Ground speed / course | GNSS motion over ground, **not airspeed or body heading** |
| Receiver link sensors | Supplied by ER8/ELRS, useful for RF alarms |
| RxBt, if provided by ER8 | Verify its configured source; do not assume this is the 3-6S propulsion pack voltage |

Names vary with firmware. The FC does not currently measure pack voltage/current or remaining capacity. No battery telemetry is fabricated. Set your pack monitoring separately until a validated measurement path exists.

Mode text:

- `B-MANUAL`, `B-ASSIST`, `B-TKOFF`: bench build, motors inhibited.
- `MANUAL*`, `ASSIST*`, `TKOFF*`: disarmed in a build that permits arming. Without `*`, armed.
- `!FS-LVL`: RC failsafe requesting level stabilization.
- `!FS-CTR`: RC failsafe with centered surfaces.
- `!LOCK`: stabilization locked out after an attitude fault.
- `!TIMING`: control deadline fault or watchdog-recovery motor inhibit.

The `!` prefix is an application label; it does not automatically create an EdgeTX audible alarm. Configure radio telemetry-loss / RF-quality alarms explicitly. During RF loss, the radio may retain the last received FM text; it cannot display a new FC failsafe message until telemetry returns. Actual RF telemetry refresh can be slower than the FC's serial transmission cadence.

## 4. Startup and mode use

1. Power on with CH5 OFF, low throttle, CH7 MANUAL. Leave the aircraft still for **at least four seconds of valid stationary IMU samples after sensor initialization**. Movement restarts the calibration window. USB `EST,bias_ready=1` confirms calibration of the selected sensor; `IMU_HEALTH` reports the two devices.
2. In the default bench build, arming is inhibited but surface/mode tests work. Check radio directions in USB `RC`, output mapping in `OUT`, and attitude signs in `ATT`.
3. For the separately enabled `v2_motor_test` build, leave a stable link established with CH5 low, then switch CH5 high while throttle is at or below 5%. A high arm switch at boot cannot arm the aircraft; switching high with throttle raised is rejected and requires another low/high cycle.
4. **MANUAL**: direct surface commands; no IMU required. Motors still require arming. SERVO1/6 receive opposite aileron pulses; SERVO2 is elevator and SERVO4 is rudder. SERVO3 is throttle.
5. **ASSIST**: roll stick commands approximately +/-40 degrees bank; elevator commands approximately +/-26 degrees pitch. Center requests level roll and configured pitch trim (initially zero). Rudder remains direct pilot input with bounded transient yaw damping; full rudder overrides damping. Throttle remains manual. With calibrated BMM350, armed/airborne ASSIST captures magnetic heading after centered roll/rudder settles; either stick releases hold. Compass loss restores ordinary roll/pitch ASSIST. This does not hold altitude, speed or position. See [magnetic heading](MAGNETIC_HEADING.md).
6. **TKOFF**: roll wing leveling with about +/-10 degrees bank command. You still control rotation, climb pitch, rudder and throttle. There is no automatic launch detection or throttle ramp.
7. CH5 OFF cuts motor commands immediately in software. Disarmed surfaces still operate in the permitted mode for checks. Return to MANUAL for direct stick control.

Do not arm before calibration if you intend to use ASSIST/TKOFF. Calibration is disabled after the first arming for that boot; early manual arming can leave stabilization unavailable until restart. A normal power cycle is required to reset the flight session, ground reference and latched faults; do this only on the ground.

## 5. RC loss and automatic recovery

After **200 ms without valid channel frames**, the FC commands minimum throttle. If the selected calibrated attitude estimate is healthy and stabilization has not been locked out, it requests level roll/pitch. Otherwise it centers surfaces. This cannot return the aircraft home, guarantee a glide, or prevent a stall.

The user chose **automatic throttle recovery**. After **300 ms of continuously usable reception**, an aircraft that was armed before the outage resumes throttle at the current stick position if CH5 is high. There is no throttle ramp or mandatory low-throttle rearm on that recovery path. CH5 low when reception recovers cancels that authorization. A timing/output fault or a cold boot also prevents automatic rearming.

The firmware timeout starts at the last valid serial channel packet. Add the receiver's own RF-loss detection latency and up to a PWM frame of actuator delay when measuring end-to-end behavior. Perform the full loss/recovery acceptance test with motors mechanically disabled; do not infer success solely from an RF icon or the transmitter's channel monitor.

## 6. Troubleshooting

| Symptom | Check |
|---|---|
| No RC | ER8 serial protocol, RX/TX crossover, common ground, separate receiver power; `CRSF_STAT,receiving=1` |
| Radio is bound, but FC has zero valid frames | Check ER8 TX is on `MCU_RX_PI_TX`, not `MCU_TX_PI_RX`; binding only establishes the RF link |
| RC channels work, but FC sensors are absent on the handset | Connect ER8 RX to `MCU_TX_PI_RX`; check outgoing `CRSF_STAT,telem_tx` increases, telemetry is enabled, and run Discover new sensors on EdgeTX |
| Sticks work but modes do not | CH7 received values and calibration; look at `MODE,req=` versus `MODE,active=` |
| CH5 mix appears correct but arming differs | ELRS `Arm using` setting, actual received CH5, stable link and low throttle |
| ESCs always at 1000 | Default motor-inhibit build, disarm/failsafe, output or timing fault; inspect boot `FLIGHT_GATE` and FM |
| ASSIST gives wrong correction | Stop. Check board attitude signs first, then individual servo reversal; do not try to tune around a sign error |
| No heading accuracy | Check USB YAW_STATUS for calibration/qualification and gyro fallback; GNSS course differs from magnetic heading in wind |
| Telemetry lost/stale | Receiver-to-FC TX wiring, enabled telemetry ratio and RF reception; old sensor values are not proof of current health |
