"""Serial discovery and ownership regression tests; no physical hardware access."""
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import gui


def device(name, vid=0x0483, pid=0x5740):
    return SimpleNamespace(device=name, vid=vid, pid=pid)


class ConnectionTests(unittest.TestCase):
    def test_discovery_requires_one_matching_board(self):
        cases = [([], None), ([device("COM8")], "COM8"),
                 ([device("COM8"), device("COM9")], None),
                 ([device("COM1", 0x1234)], None),
                 ([device("COM8"), device("COM1", 0x1234)], "COM8")]
        for ports, expected in cases:
            with self.subTest(ports=ports), patch.object(gui.serial.tools.list_ports, "comports", return_value=ports):
                self.assertEqual(gui.detect_port(), expected)

    def test_hotplug_is_rediscovered_before_opening(self):
        app = gui.MonitorApp.__new__(gui.MonitorApp)
        app.capture_path = None
        app.q = gui.queue.Queue()
        app._reader_stop = gui.threading.Event()
        # First pass: no device. Second pass: renamed USB COM port appears.
        def stop_on_open(port, *args, **kwargs):
            self.assertEqual(port, "COM19")
            app._reader_stop.set()
            raise gui.serial.SerialException("test complete")
        with patch.object(gui, "detect_port", side_effect=[None, "COM19"]) as detect, \
             patch.object(app._reader_stop, "wait"), \
             patch.object(gui.serial, "Serial", side_effect=stop_on_open) as opened:
            app._read_loop(None)
        self.assertEqual(detect.call_count, 2)
        self.assertEqual(opened.call_count, 1)


if __name__ == "__main__":
    unittest.main()
