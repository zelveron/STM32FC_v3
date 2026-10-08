# STM32FC GUI for Windows

Release **2026.10.08**, for **Windows 10/11 on x64 (Intel/AMD)**.

Double-click **STM32FC-GUI.exe**. Python, Tcl/Tk, pySerial, dfu-util and libusb
are included. No Python, pip, PlatformIO, internet connection or administrator
launch is needed to use the GUI. The EXE can run by itself from any writable
local folder; first startup can take a few seconds while it unpacks its runtime
into Windows' temporary folder. Keep the portable ZIP's licenses and source
archives when sharing the distribution.

This Windows build does not run on macOS, Linux or 32-bit Windows. Windows ARM
emulation and older Windows versions have not been qualified. The current
release is unsigned; no signing certificate is configured.

## Connect and view data

1. Close other dashboards/serial monitors using the board's COM port.
2. Run the EXE and plug the controller into USB. **Auto-detect STM32** connects
   when exactly one `0483:5740` board is present, including if plugged in later.
3. For several boards, click **Refresh ports**, select the correct COM port,
   then **Reconnect**. Reconnect releases the previous serial handle first.
4. Scroll to see flight mode, arming/failsafe, calibration, both IMU health,
   roll/pitch/yaw, raw IMU data, barometer, GPS, RC, outputs, SD/USB counters and
   the **All live parameters** table. The table retains the last raw values;
   check connection/freshness before interpreting them.

The monitor uses our v3 tagged-CSV firmware protocol. The earlier 0.3.1 JSON
sensor firmware requires the matching v3 firmware before this GUI can display
its data. A missing sensor/GPS fix/receiver is a board or signal status, not a
missing PC dependency. Yaw is relative on the assembled board without a
magnetometer.

## Sensor diagnostics

- BMI270 #1 and #2 each show a driver stage. Code 103 means the chip ID did not
  match 0x24; code 109 means the 8192-byte configuration image failed to load.
  The header identifies healthy sensors and the selected source. Raw accel/gyro
  values are from the selected IMU. One usable IMU is shown in amber because
  redundancy is unavailable.
- BMP581 reports whether initialization, register access or sample reads failed.
  A failed reset write means communication has not been established; it does
  not determine whether the underlying cause is a NACK or timeout.
- GNSS **Communication OK** requires recent checksum-valid NMEA sentences.
  **No GNSS data received** and **Bytes received · no valid NMEA** are distinct.
  With communication working, the fix row distinguishes no satellites reported
  in view, satellites in view without a fix, and unknown satellite visibility.
  **Satellites used** is the fresh GGA count, including zero without a fix.
- Lost controller telemetry expires health/status displays instead of leaving
  old green statuses visible. The raw diagnostic table remains a last-value log.

## DFU and USB drivers

**Enter DFU** requests the existing ground-maintenance command and verifies
`0483:df11` using the bundled dfu-util. The flight application and PWM stop in
ROM DFU. Use it only on the ground with propulsion disconnected, CH5 disarmed
and throttle low. Firmware enforces its disarm/idle gates. The GUI does not
select, erase or flash a firmware image. After programming/restarting the
board, choose **Auto-detect STM32** and **Reconnect** (or select its new COM
number with Refresh ports).

The EXE includes all application dependencies, **not Windows USB drivers**.
Normal CDC monitoring uses Windows' USB serial driver. DFU enumeration needs
a compatible WinUSB/libusb driver for the ROM `0483:df11` interface. An existing
ST driver association may need adjustment on a new PC. Installing a driver is
a separate administrator operation; the GUI never changes drivers or system
settings. A serial disconnect alone is never reported as successful DFU.

## Optional commands

```powershell
.\STM32FC-GUI.exe --port COM6
.\STM32FC-GUI.exe --capture "C:\Logs\bench-session.txt"
```

Create the capture directory first. Unexpected GUI errors are displayed and
logged to `%LOCALAPPDATA%\STM32FC-GUI\error.log` where possible. Connection
errors appear in the footer; close any other app holding that COM port.

## Rebuild from source

Only the build computer needs Python **3.12 x64 with Tkinter**, pip and an
internet connection for initial downloads. Run from the repository root:

```powershell
py -3.12 -m venv build\gui-venv
.\build\gui-venv\Scripts\python.exe -m pip install -r tools\gui-requirements.txt
.\build\gui-venv\Scripts\python.exe tools\build_gui.py --onedir
.\build\gui-venv\Scripts\python.exe tools\build_gui.py
```

`tools/STM32FC-GUI.spec` bundles the runtime and verified vendor binaries. The
build script fetches official archives, checks pinned SHA256 hashes, includes
licenses and corresponding DFU/libusb source, and writes:

- `dist/STM32FC-GUI.exe` — standalone, windowed application.
- `dist/STM32FC-GUI-2026.10.08-Windows-x64.zip` — EXE, quick start, checksums,
  build metadata, third-party notices and source archives.
- `dist/STM32FC-GUI.exe.sha256` — integrity checksum.

The optional folder build is under `build/gui-onedir/STM32FC-GUI/`. Downloads
are cached under `build/gui-downloads/`. Release version fields live in
`tools/build_gui.py` and `tools/gui-version.txt`. The build recipe is repeatable;
byte-identical EXEs across different build machines are not promised.

The repository also includes `.github/workflows/build-gui.yml`, which builds on
Windows x64, tests the source and packaged application, and uploads the EXE,
portable ZIP, checksums and smoke-test report as a workflow artifact.

## Validation

Run `tests/gui_parser.py`, `tests/gui_connection.py` and `tests/dfu_helper.py`
with the build venv's Python. The packaged application also has a read-only
smoke test:

```powershell
$p = Start-Process .\dist\STM32FC-GUI.exe -ArgumentList '--self-test "C:\Temp\gui-check.json"' -PassThru -Wait
$p.ExitCode
Get-Content C:\Temp\gui-check.json
```

It creates real Tk widgets, checks sample parsing, imports bundled pySerial,
executes the bundled DFU binary and enumerates USB. It **does not open a serial
port, send DFU commands or flash**. The report records bundle paths, versions
and success/failure. Local package verification is not testing on every
Windows configuration or flight qualification.
