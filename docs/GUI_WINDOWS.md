# STM32FC Ground Station

Modern desktop revision **2026.10.08-aetr6**, for **Windows 10/11 x64 (Intel/AMD)**.

This revision labels the current aircraft outputs: SERVO1 aileron, SERVO2 elevator, SERVO3 throttle, SERVO4 rudder and SERVO6 reversed aileron. Use it with the matching AETR6 firmware; the earlier `modern` release used a different mixer.

Double-click **STM32FC-GUI.exe**. Python, Qt 6, PySide6, serial support and the
DFU utility are bundled. No Python installation, browser, internet connection
or administrator launch is needed. First startup takes a few seconds while
the application unpacks into Windows' temporary directory. Keep the portable
ZIP's `licenses/` and `sources/` when redistributing it.

This build is unsigned. macOS, Linux, 32-bit Windows and Windows ARM are not
qualified by this Windows package. The source GUI requires PySide6-Essentials
6.8.3 and pySerial 3.5 on a platform supported by those packages.

## Connect

1. Close other dashboards or serial monitors using the controller.
2. Open the EXE. **Auto-detect STM32** connects when exactly one normal USB
   controller (`0483:5740`) is present, including after it is plugged in later.
3. For multiple boards, use **↻**, select the correct COM port, then
   **Reconnect**. The old reader must close before a new connection starts.
4. Choose a page in the sidebar. **Disconnect** stops the reader and preserves
   the last diagnostic values with stale labels. Reconnection starts a fresh
   session so data from two boards cannot be mixed.

**Preview** opens a clearly marked simulation, releases the real serial port,
and disables recording and DFU. **Exit preview** clears all simulated data and
attempts a real connection. Preview never sends commands to hardware.

## Pages and displayed data

| Page | Contents |
| --- | --- |
| Flight deck | Artificial horizon, shaded 3D aircraft, roll/pitch/relative yaw, active/requested mode, GPS ground speed, relative barometric altitude, climb, satellites used, position, GPS altitude/UTC, offline north-up ground track, eight component indicators, arming/failsafe/motor authorization/assist lockout/integrator state |
| Receiver | All 16 CRSF input channels, eight commanded PWM outputs, uplink LQ/RSSI/SNR, valid-frame count, CRC errors/resyncs, telemetry queue count and raw RF-mode enumeration |
| Sensors | Each BMI270's health/driver error/register snapshot, selected source, dual-IMU disagreement, BMP581 error/pressure/temperature/pressure altitude, GNSS communication/satellite state, filtered selected-IMU acceleration/angular rate, gyro calibration/bias/gravity trust, BMM350 field values, SD status/drop counters and rolling attitude history |
| Diagnostics | Every received tagged message and scheduler task, searchable fields, update age, recent/stale state and local JSON snapshot export |
| System | Connection/capture state, message and drop counts, recent session/mode events and ROM DFU maintenance |

Status colors are green for current healthy data, amber for degraded/missing
fresh samples, red for a reported failure or lost link, and gray for offline,
waiting or firmware-disabled devices. Hover over an indicator for details.
Inactive SD logging does not prove the SD card is missing. Individual IMU
indicators do not imply both IMUs are feeding the estimator simultaneously.

## Interpreting the instruments

- Body axes are forward/right/down. Positive roll is right wing down; positive
  pitch is nose up. The aircraft uses the same roll/pitch/yaw rotation as the
  attitude values. An invalid estimator blanks both attitude instruments.
- **Yaw is relative, not a magnetic heading.** The current firmware does not
  fuse BMM350 into heading, even when magnetic samples are available.
- **GPS ground speed is not airspeed.** The track is a local offline plot,
  without map tiles, terrain or waypoint navigation. Last track points remain
  gray when there is no current position fix.
- Relative barometric altitude uses the firmware's reference pressure. It is
  not measured clearance over terrain. GPS altitude is separately labeled MSL.
- Satellites used come from fresh GGA. Visibility is only unknown / none
  reported / satellites in view; firmware does not expose a total in-view count.
- RC bars require a receiving CRSF status as well as recent channels. The
  firmware may keep publishing cached channels after receiver-link loss.
- PWM values are **commands**, not measured servo position or confirmation
  that the ESC/servo accepted them. Radio telemetry counters are not handset
  acknowledgements. RF-mode names depend on ELRS radio firmware, so the raw
  enumeration is shown.
- USB provides one selected IMU signal set; it does not provide two independent
  raw IMU streams. The GUI shows each IMU's health separately.

High-rate attitude/IMU/barometer values expire after 0.35 seconds, RC/PWM/mag
after 0.5 seconds, mode/link after 1.5 seconds, GPS after 2.5 seconds, and most
health messages after 3 seconds. SD status has a 12-second allowance for its
5-second reporting interval. Raw diagnostics retain last values and their ages.
The GUI refreshes at up to 20 Hz; this does not change firmware or RF rates.

The GUI expects the v3 tagged-CSV protocol. Earlier 0.3.1 JSON sensor firmware
needs its matching dashboard or a deliberate firmware update.

## Recording and diagnostics

**Record USB** saves incoming tagged CSV locally. Changing recording restarts
the serial session. Disk errors stop recording, report a session event and
leave live telemetry running. **Export diagnostic snapshot** saves the latest
raw lines, ages, connection state and whether the data was simulated.

BMI270 code 103 means chip ID mismatch (expected `0x24`); code 109 means the
configuration image failed to load. BMP581 reset-write failure identifies
the failed driver stage, not its underlying electrical cause. GNSS byte
reception, valid NMEA, satellite visibility and position fix are distinct.

Unexpected errors are shown and logged to
`%LOCALAPPDATA%\STM32FC-GUI\error.log` when writable.

## DFU and drivers

In **System → Enter ROM DFU**, confirm ground use with propulsion disconnected,
CH5 disarmed and throttle low. ROM DFU stops the flight application,
stabilization and PWM. The GUI releases its serial reader, sends the existing
maintenance command and verifies `0483:df11` with bundled dfu-util. It does not
select, erase or flash a firmware image. A serial disconnect alone is never
reported as successful DFU. After a board restart, use **Reconnect**.

The EXE bundles application dependencies, not operating-system USB drivers.
Normal monitoring uses Windows USB serial support; ROM DFU needs an appropriate
WinUSB/libusb driver. The GUI does not install drivers or change system settings.

## Optional commands

```powershell
.\STM32FC-GUI.exe --demo
.\STM32FC-GUI.exe --port COM6
.\STM32FC-GUI.exe --capture "C:\Logs\bench-session.csv"
```

Create the capture directory first. There is no cloud upload.

## Rebuild and verify

Use Python 3.12 x64 on Windows. Only the build needs internet for dependencies
and the initial verified vendor downloads:

```powershell
py -3.12 -m venv build\gui-venv
.\build\gui-venv\Scripts\python.exe -m pip install -r tools\gui-requirements.txt
.\build\gui-venv\Scripts\python.exe tests\gui_parser.py
.\build\gui-venv\Scripts\python.exe tests\gui_connection.py
.\build\gui-venv\Scripts\python.exe tests\gui_ui.py
.\build\gui-venv\Scripts\python.exe tests\dfu_helper.py
.\build\gui-venv\Scripts\python.exe tools\build_gui.py --onedir
.\build\gui-venv\Scripts\python.exe tools\build_gui.py
```

Outputs are `dist/STM32FC-GUI.exe`, its SHA256 file and
`dist/STM32FC-GUI-2026.10.08-aetr6-Windows-x64.zip`. The optional folder build is
`build/gui-onedir/STM32FC-GUI/`. The ZIP also includes editable GUI source,
Qt/PySide/DFU/libusb source archives, notices and a build manifest with the base
commit, dirty-worktree flag and hashes of all GUI Python modules. Version
fields live in `tools/build_gui.py` and `tools/gui-version.txt`.

The package smoke test renders all five Qt pages, validates preview/fault/
disconnect behavior, checks bundled imports and enumerates USB using dfu-util:

```powershell
$p = Start-Process .\dist\STM32FC-GUI.exe -ArgumentList '--self-test "C:\Temp\gui-check.json"' -PassThru -Wait
$p.ExitCode
Get-Content C:\Temp\gui-check.json
```

It does not open a serial port, send commands or flash. The Windows build
workflow runs the source regressions and packaged smoke test. These checks
validate the desktop software, not flight behavior or every Windows machine.
