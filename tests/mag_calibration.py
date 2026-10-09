"""Recover known installation errors from independent synthetic 3D rotations."""
import sys
from pathlib import Path
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
from calibrate_mag import fit, header, read_capture, world_rotations


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        rng=np.random.default_rng(350)
        self.attitudes=np.column_stack((rng.uniform(-180,180,1500),rng.uniform(-80,80,1500),rng.uniform(-180,180,1500)))
        body=np.einsum('nji,j->ni',world_rotations(self.attitudes),np.array([30,0,40]))
        self.rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        self.offset=np.array([18,-12,7])
        self.correction=np.array([[1.1,.08,-.03],[.08,.85,.04],[-.03,.04,1.04]])
        sensor=body @ self.rotation
        self.fields=sensor @ np.linalg.inv(self.correction).T+self.offset+rng.normal(0,.03,body.shape)

    def test_recovers_offsets_soft_iron_and_mounting(self):
        result=fit(self.fields,self.attitudes)
        self.assertEqual(result["rotation"],[-2,1,3])
        np.testing.assert_allclose(result["offset_ut"],self.offset,atol=.02)
        np.testing.assert_allclose(result["correction"],self.correction,atol=.002)
        self.assertLess(result["alignment_rms"],.01)
        text=header(result)
        self.assertIn("true, true, true",text)
        self.assertIn("{-2,1,3}",text)

    def test_planar_or_stationary_capture_rejected(self):
        for fields in (np.tile([30,0,40],(1500,1)), np.column_stack((30*np.cos(np.arange(1500)*.02),30*np.sin(np.arange(1500)*.02),np.full(1500,40)))):
            with self.assertRaises((ValueError,np.linalg.LinAlgError)): fit(fields,self.attitudes)

    def test_bad_attitude_pairing_rejected(self):
        with self.assertRaises(ValueError): fit(self.fields,self.attitudes[::-1])

    def test_magnetic_disturbance_and_nonfinite_capture_rejected(self):
        fields=self.fields.copy(); fields[::7]+=150
        with self.assertRaises((ValueError,np.linalg.LinAlgError)): fit(fields,self.attitudes)
        fields[0,0]=np.nan
        with self.assertRaises(ValueError): fit(fields,self.attitudes)

    def test_capture_requires_eligible_recent_gyro_attitude(self):
        lines=["IMU_HEALTH,1,1,0,0,1","ATT,1,2,3","MAG,29,0,40", # no calibrated gyro yet
               "EST,bias_ready=1","ATT,1,2,3","MAG,30,0,40","MAG,31,0,40",
               "MAG,32,0,40", # too many magnetic samples since last attitude
               "YAW_STATUS,source=BMM350","ATT,1,2,3","MAG,33,0,40",
               "YAW_STATUS,source=GYRO","IMU_HEALTH,1,1,0,1,0","ATT,1,2,3","MAG,34,0,40"]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"capture.txt"; path.write_text("\n".join(lines))
            fields,attitudes=read_capture(path)
        self.assertEqual(fields.shape,(2,3)); self.assertEqual(attitudes.shape,(2,3))

    def test_expired_health_cannot_calibrate(self):
        lines=["IMU_HEALTH,1,1,0,0,1","EST,bias_ready=1"]+["ATT,1,2,3"]*61+["MAG,30,0,40"]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"capture.txt";path.write_text("\n".join(lines))
            fields,_=read_capture(path)
        self.assertEqual(len(fields),0)


if __name__=="__main__": unittest.main()
