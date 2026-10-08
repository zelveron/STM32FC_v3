"""Read-only package smoke test; never sends serial commands or changes firmware."""
import json
from pathlib import Path
import platform
import subprocess
import sys
import traceback
import tkinter as tk

import serial
from enter_dfu import find_dfu_util, dfu_list


def run(report_path):
    report = {"ok": False, "frozen": bool(getattr(sys, "frozen", False)),
              "python": platform.python_version(), "architecture": platform.machine(),
              "pyserial": serial.__version__, "serial_module": serial.__file__}
    root = None
    try:
        from gui import MonitorApp
        root = tk.Tk()
        root.withdraw()
        app = MonitorApp(root, start_reader=False)
        app._handle_line("IMU_CONFIG,model=BMI270,mag=0,error0=0,error1=0")
        app._handle_line("IMU_HEALTH,1,1,0,0,1")
        app._handle_line("ATT,10,5,90")
        app._handle_line("BMP,1013.25,20,100")
        app._handle_line("GPS_HEALTH,rx=1,nmea=1,fix=0,used=0,visible=0,baud=9600")
        root.update_idletasks()
        assert "BMI270 · #1 OK · #2 OK" in app.status_var.get()
        assert "+10.0" in app.roll_var.get()
        assert "1013.250" in app.pressure_var.get()
        assert "Communication OK" in app.gps_link_var.get()
        assert "no satellites" in app.fix_var.get()
        app._handle_line("IMU_CONFIG,model=BMI270,mag=0,error0=0,error1=103,regs0=240100,regs1=0")
        app._handle_line("IMU_HEALTH,1,0,0,0,1")
        app._handle_line("BMP_HEALTH,valid=0,error=2")
        root.update_idletasks()
        assert "Chip ID mismatch" in app.imu2_diag_var.get()
        assert "#2 unavailable" in app.status_var.get()
        assert "communication not established" in app.bmp_health_var.get()
        assert app.diag_tree.get_children()
        assert not app.reader.is_alive()
        report["tk"] = root.tk.call("info", "patchlevel")
        report["widgets_and_parser"] = "passed; no serial port opened"
        utility = find_dfu_util()
        report["dfu_util"] = utility
        if report["frozen"]:
            assert Path(utility).is_relative_to(Path(sys._MEIPASS))
            assert Path(serial.__file__).is_relative_to(Path(sys._MEIPASS))
        version = subprocess.run([utility, "--version"], capture_output=True,
                                 text=True, timeout=10,
                                 creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        assert version.returncode == 0, version.stderr
        assert "dfu-util 0.11" in version.stdout
        report["dfu_version"] = version.stdout.splitlines()[0]
        report["dfu_listing"] = dfu_list(utility)
        report["ports"] = [p.device for p in serial.tools.list_ports.comports()]
        report["ok"] = True
    except Exception:
        report["error"] = traceback.format_exc()
    finally:
        if root is not None:
            root.destroy()
    Path(report_path).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if report["ok"] else 1
