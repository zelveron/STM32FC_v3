"""Qt lifecycle and instrument regression tests. No physical serial access."""
import sys
from pathlib import Path
from unittest.mock import patch, MagicMock
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from PySide6.QtWidgets import QApplication
from gui_controller import MonitorApp
from gui_demo import frame
from gui_telemetry import Telemetry
from gui_widgets import rotate_body


class DashboardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.qt = QApplication.instance() or QApplication([])

    def setUp(self):
        self.serial = patch("gui_transport.serial.Serial", side_effect=AssertionError("Unexpected serial open"))
        self.serial.start()
        self.window = MonitorApp(start_reader=False)
        self.window.timer.stop()

    def tearDown(self):
        self.window.session.thread = None
        self.window.close()
        self.serial.stop()

    def test_preview_is_isolated_and_cleared_on_exit(self):
        w = self.window
        w.request_transition("demo"); w.poll()
        self.assertIn("SIMULATED", w.banner.text())
        self.assertFalse(w.dfu_button.isEnabled())
        self.assertFalse(w.record_button.isEnabled())
        self.assertFalse(w.session.running())
        with patch.object(w.session, "start") as start:
            w.request_transition("connect")
        start.assert_called_once()
        self.assertEqual(w.model.received, 0)
        self.assertFalse(w.demo)
        self.assertEqual(w.metrics["speed"].value.text(), "—")

    def test_preview_waits_for_existing_reader_to_close(self):
        w = self.window
        w.session.thread = MagicMock()
        w.session.thread.is_alive.return_value = True
        w.request_transition("demo")
        self.assertTrue(w.session.stop.is_set())
        self.assertFalse(w.demo)
        self.assertFalse(w.connect_button.isEnabled())
        w.session.thread.is_alive.return_value = False
        w.finish_transition()
        self.assertTrue(w.demo)

    def test_stale_and_disconnected_values_are_not_live(self):
        w = self.window; clock = [10.0]
        w.model = Telemetry(lambda: clock[0]); w.link_open = True
        for line in frame(0): w.model.feed(line)
        w.render()
        self.assertIsNotNone(w.horizon.attitude)
        clock[0] += 13; w.render()
        self.assertIsNone(w.horizon.attitude)
        self.assertEqual(w.metrics["mode"].value.text(), "—")
        self.assertIsNone(w.rc_bars.values)
        self.assertIsNone(w.out_bars.values)
        w.request_transition("disconnect")
        self.assertGreater(w.model.received, 0)
        self.assertIn("Offline", w.health_tiles["imu1"].state.text())

    def test_all_pages_render_at_supported_sizes(self):
        w = self.window; w.start_demo(); w.poll(); w.show()
        for size in ((1080, 700), (1440, 910)):
            w.resize(*size)
            for page in range(5):
                w.select_page(page); self.qt.processEvents()
                self.assertFalse(w.grab().isNull())
        w.filter.setText("GPS_HEALTH"); w.update_table(True)
        self.assertEqual(w.table.rowCount(), 1)
        self.assertEqual(w.table.item(0, 0).text(), "GPS_HEALTH")

    def test_body_axis_rotation_convention(self):
        # FRD body -> NED: right wing down; nose up; clockwise yaw.
        for point,angles,expected in (((0,1,0),(90,0,0),(0,0,1)),
                                      ((1,0,0),(0,90,0),(0,0,-1)),
                                      ((1,0,0),(0,0,90),(0,1,0))):
            actual = rotate_body(point, *angles)
            for a,b in zip(actual,expected): self.assertAlmostEqual(a,b,places=6)

    def test_closed_window_does_not_restart_a_pending_connection(self):
        w = self.window; w.pending = "connect"; w.close()
        with patch.object(w.session, "start") as start: w.finish_transition()
        start.assert_not_called()


if __name__ == "__main__": unittest.main()
