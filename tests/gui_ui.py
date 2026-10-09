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

    def test_heading_source_is_explicit_and_expires(self):
        w=self.window; clock=[10.0]; w.model=Telemetry(lambda:clock[0]); w.link_open=True
        for line in frame(0): w.model.feed(line)
        w.model.feed("YAW_STATUS,source=BMM350,valid=1,reason=tracking,heading=90,mag_heading=90,field=50,innovation=0,configured=1,hold=0,target=0")
        w.render(); self.assertIn("BMM350 + gyro",w.yaw_source.text())
        self.assertEqual(w.yaw_title.text(),"MAGNETIC HEADING")
        w.model.feed("MODE,active=ASSIST,req=ASSIST,armed=1,failsafe=0,flying=1")
        w.model.feed("YAW_STATUS,source=BMM350,valid=1,reason=tracking,heading=90,mag_heading=90,field=50,innovation=0,configured=1,hold=1,target=90")
        w.render();self.assertIn("Holding 90.0°",w.heading_hold_status.text())
        w.model.feed("MODE,active=ASSIST,req=ASSIST,armed=1,failsafe=1,flying=1")
        w.render();self.assertNotIn("Holding",w.heading_hold_status.text())
        w.model.feed("YAW_STATUS,source=GYRO,valid=0,reason=field_rejected,heading=90,mag_heading=0,field=100,innovation=90,configured=1,hold=0,target=0")
        w.render(); self.assertIn("gyro fallback",w.yaw_source.text())
        self.assertIn("rejected",w.yaw_source.text())
        self.assertIn("unavailable",w.heading_hold_status.text())
        clock[0]+=3.1; w.render(); self.assertIn("stale",w.yaw_source.text())

    def test_online_magnetometer_does_not_invent_heading_source(self):
        w=self.window; w.link_open=True
        for line in frame(0): w.model.feed(line)
        del w.model.records["YAW_STATUS"]
        w.model.feed("MAG,30,0,40")
        w.render(); self.assertIn("unavailable",w.yaw_source.text())

    def test_mag_fault_is_displayed_even_with_recent_sample(self):
        w=self.window;w.link_open=True
        for line in frame(0):w.model.feed(line)
        w.model.feed("MAG,30,0,40")
        w.model.feed("MAG_HEALTH,enabled=1,initialized=1,healthy=0,communicating=1,stage=11,result=0,samples=19,reads=22,bus_errors=0,invalid=3")
        w.model.feed("MAG_DATA,x=-3895,y=-3358,z=-41,temp=30,raw=-550000/-450000/-6000/55000")
        w.model.feed("MAG_CHECK,x=0.5,y=0.1,err=0,pmu=40,aggr=54,axes=7,st=0")
        w.render()
        self.assertIn("Out of range",w.storage_details.text())
        self.assertIn("Rejected field: -3895.00",w.storage_details.text())
        self.assertNotIn("+30.00 / +0.00",w.storage_details.text())
        self.assertIn("Startup self-test X/Y: 0.50 / 0.10",w.storage_details.text())
        w.model.feed("MAG_HEALTH,enabled=1,initialized=0,healthy=0,stage=13,result=-2,reads=0")
        w.render();self.assertIn("incomplete (communication error)",w.storage_details.text())
        self.assertNotIn("Rejected field",w.storage_details.text())


if __name__ == "__main__": unittest.main()
