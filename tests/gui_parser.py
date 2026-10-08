"""Protocol/freshness regressions; no Qt, board or serial connection required."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
from gui_telemetry import Telemetry
from gui_demo import frame


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.now=10.0
        self.m=Telemetry(lambda:self.now)

    def populate(self):
        for line in frame(0): self.assertTrue(self.m.feed(line),line)

    def test_complete_flight_sample(self):
        self.populate()
        self.assertEqual(self.m.get("MODE")["active"],"ASSIST")
        self.assertIsNotNone(self.m.attitude()); self.assertIsNotNone(self.m.position())
        self.assertEqual(self.m.get("GPS")["sats"],14)
        self.assertEqual(len(self.m.get("RC")),16); self.assertEqual(len(self.m.get("OUT")),8)
        self.assertEqual(self.m.get("OUT")[-2:],(1000,1000))

    def test_attitude_requires_eligible_estimator(self):
        self.m.feed("ATT,10,5,90"); self.assertIsNone(self.m.attitude())
        self.m.feed("IMU_HEALTH,1,1,0,0,1"); self.assertEqual(self.m.attitude(),(10,5,90))
        self.m.feed("IMU_HEALTH,1,1,0,1,0"); self.assertIsNone(self.m.attitude())

    def test_high_rate_data_expires(self):
        self.populate(); self.now+=.6
        for tag in ("ATT","BMI","BMP","RC","OUT"): self.assertIsNone(self.m.get(tag),tag)
        self.assertIsNotNone(self.m.get("MODE"))

    def test_mode_arming_and_failsafe_expire(self):
        self.populate(); self.now+=1.6; self.assertIsNone(self.m.get("MODE"))

    def test_all_health_becomes_stale(self):
        self.populate(); self.now+=13
        for k in ("imu1","imu2","baro","gps","rc","sd","mag"):
            self.assertEqual(self.m.health(k).state,"stale",k)

    def test_disconnected_health_is_never_green(self):
        self.populate()
        for k in ("usb","imu1","imu2","baro","gps","rc","sd","mag"):
            self.assertEqual(self.m.health(k,False).state,"off")

    def test_second_imu_failure_is_separate(self):
        self.populate(); self.m.feed("IMU_CONFIG,model=BMI270,mag=0,error0=0,error1=103,regs0=240100,regs1=0")
        self.m.feed("IMU_HEALTH,1,0,0,0,1")
        self.assertEqual(self.m.health("imu1").state,"ok")
        self.assertEqual(self.m.health("imu2").state,"bad")
        self.assertIn("Chip ID mismatch",self.m.health("imu2").detail)
        self.assertIsNotNone(self.m.attitude())

    def test_imu_disagreement(self):
        self.populate(); self.m.feed("IMU_HEALTH,1,1,0,1,0")
        self.assertEqual(self.m.health("imu1").label,"Disagree")
        self.assertIsNone(self.m.attitude())

    def test_barometer_fault_vetoes_recent_sample(self):
        self.populate(); self.m.feed("BMP_HEALTH,valid=0,error=2")
        self.assertIsNone(self.m.barometer())
        self.assertIn("communication not established",self.m.health("baro").detail)

    def test_disabled_magnetometer_is_not_failed(self):
        self.populate(); self.assertEqual(self.m.health("mag").label,"Disabled")

    def test_enabled_magnetometer_needs_sample(self):
        self.m.feed("IMU_CONFIG,model=BMI270,mag=1,error0=0,error1=0")
        self.assertEqual(self.m.health("mag").state,"stale")
        self.m.feed("MAG,1,2,3"); self.assertEqual(self.m.health("mag").state,"ok")

    def test_sd_cadence_is_five_seconds(self):
        self.populate(); self.now+=6; self.assertIsNotNone(self.m.get("SD_DBG"))
        self.m.feed("SD_DBG,0,,bytes=0,log_drops=0,usb_drops=1")
        self.assertEqual(self.m.health("sd").label,"Inactive")

    def test_rc_echo_does_not_prove_link(self):
        self.populate(); self.m.feed("CRSF_STAT,receiving=0,frames_ok=1,crc_err=0,resync=0,telem_tx=0")
        self.assertFalse(self.m.receiver_live())
        self.assertEqual(self.m.health("rc").state,"bad")

    def test_gnss_communication_states(self):
        for rx,nmea,label in ((0,0,"No data"),(1,0,"No NMEA"),(1,1,"No fix")):
            self.m.feed(f"GPS_HEALTH,rx={rx},nmea={nmea},fix=0,used=0,visible=-1,baud=9600")
            self.assertEqual(self.m.health("gps").label,label)

    def test_lost_fix_clears_position_but_keeps_last_track(self):
        self.populate(); self.m.feed("GPS_HEALTH,rx=1,nmea=1,fix=0,used=0,visible=1,baud=9600")
        self.assertIsNone(self.m.position()); self.assertTrue(self.m.track)

    def test_lost_gnss_data_vetoes_cached_fix(self):
        self.populate(); self.m.feed("GPS_HEALTH,rx=0,nmea=0,fix=1,used=14,visible=1,baud=9600")
        self.assertIsNone(self.m.position())

    def test_old_no_fix_does_not_hide_new_position(self):
        self.m.feed("GPS_HEALTH,rx=1,nmea=1,fix=0,used=0,visible=1,baud=9600")
        self.now+=.1; self.m.feed("GPS,40,29,100,9,1,12:00:00,30")
        self.assertIsNotNone(self.m.position())

    def test_legacy_gps_status_can_revoke_fix(self):
        self.populate(); self.m.feed("GPS_STAT,0,0,12:00:00,0")
        self.assertIsNone(self.m.position())

    def test_nonfinite_and_malformed_samples_do_not_mutate_state(self):
        self.populate()
        for line in ("ATT,nan,0,0","ATT,1,2","BMI,1,2,3,4,5,inf","BMP,1,nan,3",
                     "GPS,91,29,100,9,1,12:00:00,30","GPS,40,181,100,9,1,12:00:00,30",
                     "IMU_HEALTH,1,0,bad,0,1","BMP_HEALTH,valid=7,error=0",
                     "MODE,active=ASSIST,req=ASSIST,armed=bad","RC,1500",
                     "GPS_HEALTH,rx=1,nmea=bad,fix=0,used=0,visible=0,baud=9600",
                     "EST,bias_ready=1,acc_trust=1,agl_m=nan,climb_mps=0,baro_ref=1"):
            before=dict(self.m.records)
            self.assertFalse(self.m.feed(line),line); self.assertEqual(self.m.records,before)

    def test_raw_fields_and_scheduler_remain_available(self):
        self.m.feed("SCHED,control*,hz=400,max_us=99,overruns=0")
        self.m.feed("TELEM,1,2,3,4,5,6")
        self.assertIn("SCHED:control*",self.m.raw); self.assertEqual(self.m.get("TELEM"),(1,2,3,4,5,6))

    def test_storage_is_bounded(self):
        for n in range(1500):
            self.m.feed(f"ATT,{n%90},0,0"); self.m.feed(f"GPS,40,{29+n/100000},100,9,1,12:00:00,30")
            self.m.feed(f"UNKNOWNTAG{n},x=1")
        self.assertLessEqual(len(self.m.raw),150); self.assertLessEqual(len(self.m.history),600)
        self.assertLessEqual(len(self.m.track),1200)
        self.assertLessEqual(len(self.m.records),20)


if __name__=="__main__": unittest.main()
