#!/usr/bin/env python3
"""Enter ROM DFU over USB CDC and verify 0483:df11; never erase or flash.

Close the dashboard first. Usage: python tools/enter_dfu.py --port COM5
Requires pySerial and dfu-util (PATH or PlatformIO's tool-dfuutil package).
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess
import time

import serial
from serial.tools import list_ports


def find_dfu_util(explicit=None):
    if explicit:
        return str(Path(explicit).resolve(strict=True))
    found = shutil.which("dfu-util")
    if found:
        return found
    for name in ("dfu-util.exe", "dfu-util"):
        p = Path.home() / ".platformio/packages/tool-dfuutil/bin" / name
        if p.is_file():
            return str(p)
    raise RuntimeError("dfu-util not found; install it or provide --dfu-util PATH")


def dfu_devices(output):
    """One entry per ROM device, using only its internal-flash alternate."""
    return [line for line in output.splitlines()
            if re.search(r"Found DFU:\s*\[0483:df11\]", line, re.I)
            and re.search(r"\balt=0(?:,|\s)", line)]


def dfu_list(executable):
    result = subprocess.run([executable, "-l"], capture_output=True, text=True,
                            timeout=5, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    return result.stdout + result.stderr


def request_dfu(port=None, executable=None, timeout=10):
    executable = find_dfu_util(executable)
    before = dfu_list(executable)
    if dfu_devices(before):
        raise RuntimeError("STM32 DFU already present; no serial command sent. Check the selected board with dfu-util -l.")
    if port is None:
        candidates = [p.device for p in list_ports.comports() if p.vid == 0x0483 and p.pid == 0x5740]
        if len(candidates) != 1:
            raise RuntimeError("Expected one STM32 CDC port; connect the board or specify --port COMx")
        port = candidates[0]
    with serial.Serial(port, 115200, timeout=0.1, write_timeout=1) as device:
        device.write(b"\ndfu\n")
        try:
            device.flush()
        except (serial.SerialException, OSError):
            pass  # The MCU may reset immediately. This does NOT confirm DFU.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            try:
                line = device.readline().decode("ascii", errors="replace").strip()
            except (serial.SerialException, OSError):
                break
            if line.startswith("NAK,DFU"):
                raise RuntimeError("Firmware refused DFU: land, disarm CH5 and lower throttle; after flight a stable RC link is required.")
    deadline = time.monotonic() + timeout
    last_output = ""
    while time.monotonic() < deadline:
        last_output = dfu_list(executable)
        devices = dfu_devices(last_output)
        if len(devices) == 1:
            return devices[0]
        if len(devices) > 1:
            raise RuntimeError("Multiple STM32 DFU boards appeared; select the correct board before programming.")
        time.sleep(0.25)
    raise RuntimeError("DFU not verified. Serial disconnect alone is insufficient. Check USB/DFU driver and BOOT/RESET if the application is unresponsive.\n" + last_output.strip())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="CDC port, e.g. COM5 or /dev/ttyACM0 (default: detect one STM32)")
    parser.add_argument("--dfu-util", help="path to dfu-util executable")
    args = parser.parse_args()
    try:
        print("Verified ROM DFU:", request_dfu(args.port, args.dfu_util))
    except (RuntimeError, OSError, serial.SerialException, subprocess.SubprocessError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
