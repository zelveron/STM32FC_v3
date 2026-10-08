"""Read-only package smoke test; never opens a serial port or changes firmware."""
import json
from pathlib import Path
import platform
import subprocess
import sys
import traceback
from unittest.mock import patch
import serial
from PySide6 import __version__ as pyside_version
from PySide6.QtCore import qVersion
from PySide6.QtWidgets import QApplication
from enter_dfu import find_dfu_util, dfu_list


def run(report_path):
    report = {"ok": False, "frozen": bool(getattr(sys, "frozen", False)),
              "python": platform.python_version(), "architecture": platform.machine(),
              "pyserial": serial.__version__, "serial_module": serial.__file__,
              "pyside": pyside_version, "qt": qVersion()}
    window = None
    application = QApplication.instance() or QApplication([])
    try:
        from gui_controller import MonitorApp
        with patch("gui_transport.serial.Serial", side_effect=AssertionError("Self-test must not open serial")):
            window = MonitorApp(start_reader=False, demo=True)
            window.timer.stop()
            window.poll()
            assert window.metrics["mode"].value.text() == "ASSIST"
            assert window.model.attitude() is not None
            assert window.model.position() is not None
            assert not window.dfu_button.isEnabled()
            assert not window.session.running()
            assert "SIMULATED" in window.banner.text()
            window.show()
            for page in range(5):
                window.select_page(page)
                application.processEvents()
                assert not window.grab().isNull()
            window.model.feed("IMU_CONFIG,model=BMI270,mag=0,error0=0,error1=103,regs0=240100,regs1=0")
            window.model.feed("IMU_HEALTH,1,0,0,0,1")
            window.model.feed("BMP_HEALTH,valid=0,error=2")
            window.render()
            assert "Chip ID mismatch" in window.sensor_details["imu2"].text()
            assert "communication not established" in window.sensor_details["baro"].text()
            window.request_transition("disconnect")
            assert window.model.received == 0
            assert window.metrics["speed"].value.text() == "—"
        report["widgets_and_parser"] = "Five Qt pages rendered; preview/fault/disconnect checks passed; no serial port opened"
        utility = find_dfu_util()
        report["dfu_util"] = utility
        if report["frozen"]:
            bundle = Path(sys._MEIPASS).resolve()
            assert Path(utility).resolve().is_relative_to(bundle)
            assert Path(serial.__file__).resolve().is_relative_to(bundle)
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
        if window is not None:
            window.close()
    Path(report_path).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if report["ok"] else 1
