import csv
import importlib.util
import struct
import sys
import tempfile
import unittest
import math
from types import SimpleNamespace
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from log_container import crc16, read_payload, records, IMU_FMT, CONTROL_FMT
from export_diagnostics import export
from calibrate_accel import fit
from parse_bin_log import decode, HDR_FMT, FRAME_FMT
from analyze_imu import spectrum

def record(fmt, values):
    raw=struct.pack(fmt, *values)
    return raw[:-2]+struct.pack('<H',crc16(raw[:-2]))

def sector(payload, seq, session=100):
    raw=struct.pack('<4sQIH',b'FCS2',session,seq,len(payload))+payload.ljust(492,b'\0')
    return raw+struct.pack('<H',crc16(raw))

class LogTests(unittest.TestCase):
    def setUp(self):
        directory=ROOT/'build'/'log-tool-tests'/self._testMethodName
        directory.mkdir(parents=True,exist_ok=True)
        self.tmp=SimpleNamespace(name=str(directory))
        self.path=Path(self.tmp.name)/'flight.bin'
        self.header=struct.pack(HDR_FMT,b'STFC',1,86,0x5aa5,2048,16,100)
        self.imu=record(IMU_FMT,[0xa16d,2500,1,64]+[16,32,48,0,0,2048]*2+[0,1,0])
        self.control=record(CONTROL_FMT,[0xc17d,10000,16,32,16,32,48,1000,2000,3000,7000,1,5,0])
        self.normal=record(FRAME_FMT,[0x5aa5,100]+[0]*9+[101325.,200,0,0,0,0,0,0,0]+[1500]*16+[1,1,0])
    def tearDown(self): pass # fixtures stay in ignored build/ for inspection
    def test_mixed_recovery_across_sectors(self):
        body=self.header+self.imu*15+self.control+self.normal
        blocks=[sector(body[i:i+492],i//492) for i in range(0,len(body),492)]
        self.path.write_bytes(b''.join(blocks)+b'\xff'*512)
        recovered,stats=read_payload(self.path)
        self.assertEqual(recovered,body)
        self.assertEqual(stats['stop'],'unwritten-or-corrupt-sector')
        self.assertEqual(len(list(records(self.path,{}))),17)
        meta,rows=decode(self.path)
        self.assertEqual(len(rows),1)
        self.assertEqual(meta['n_resync'],0)
    def test_torn_write_stops_at_durable_prefix(self):
        self.path.write_bytes(sector(self.header,0)+sector(self.imu,1)[:200])
        data,stats=read_payload(self.path)
        self.assertEqual(data,self.header)
        self.assertEqual(stats['stop'],'partial-sector')
    def test_old_session_not_accepted(self):
        self.path.write_bytes(sector(self.header,0)+sector(self.imu,1,99))
        self.assertEqual(read_payload(self.path)[0],self.header)
    def test_missing_sector_not_spliced(self):
        self.path.write_bytes(sector(self.header,0)+sector(self.imu,2))
        self.assertEqual(read_payload(self.path)[0],self.header)
    def test_bad_sector_crc_rejected(self):
        raw=bytearray(sector(self.imu,1)); raw[24]^=1
        self.path.write_bytes(sector(self.header,0)+raw)
        self.assertEqual(read_payload(self.path)[0],self.header)
    def test_legacy_and_csv_units(self):
        self.path.write_bytes(self.header+self.imu+self.control+self.normal)
        prefix=str(Path(self.tmp.name)/'test')
        stats=export(self.path,prefix)
        self.assertEqual(stats['imu0_records'],1)
        with open(prefix+'_imu0.csv') as f: row=next(csv.DictReader(f))
        self.assertEqual(float(row['raw_gx_dps']),1)
        self.assertEqual(float(row['filtered_az_g']),1)
        with open(prefix+'_control.csv') as f: row=next(csv.DictReader(f))
        self.assertEqual(float(row['throttle']),.7)
        self.assertEqual(float(row['elevator']),.2)
        self.assertEqual(int(row['integrators']),1)
        self.assertEqual(len(decode(self.path)[1]),1)
    def test_six_face_fit(self):
        path=Path(self.tmp.name)/'faces.csv'
        with open(path,'w',newline='') as f:
            w=csv.writer(f);w.writerow(['face','ax_g','ay_g','az_g'])
            for i,axis in enumerate('XYZ'):
                for sign in '+-':
                    values=[.01,-.02,.03]; values[i]+=(1 if sign=='+' else -1)/1.02
                    for _ in range(100): w.writerow([sign+axis]+values)
        cal=fit(path)
        for v,target in zip(cal['offset_g'],[.01,-.02,.03]):self.assertAlmostEqual(v,target)
        for v in cal['scale']:self.assertAlmostEqual(v,1.02)
    def test_incomplete_calibration_rejected(self):
        path=Path(self.tmp.name)/'faces.csv';path.write_text('face,ax_g,ay_g,az_g\n+X,1,0,0\n')
        with self.assertRaises(ValueError):fit(path)
    def vibration_csv(self, gap=False):
        path=Path(self.tmp.name)/'vibration.csv'
        with open(path,'w',newline='') as f:
            w=csv.writer(f)
            w.writerow(['sequence']+[f'{kind}_g{axis}_dps' for kind in ('raw','filtered') for axis in 'xyz'])
            for n in range(2048):
                v=math.sin(2*math.pi*80*n/400)
                w.writerow([n+(1 if gap and n>=1000 else 0)]+[v]*3+[v*.1]*3)
        return path
    def test_spectrum_recovers_known_peak(self):
        out=Path(self.tmp.name)/'spectrum.csv';spectrum(self.vibration_csv(),out)
        with open(out) as f:rows=list(csv.DictReader(f))
        peak=max(rows,key=lambda r:float(r['raw_gx_dps_psd']))
        self.assertLess(abs(float(peak['frequency_hz'])-80),.4)
        self.assertAlmostEqual(float(peak['filtered_gx_dps_psd'])/float(peak['raw_gx_dps_psd']),.01,places=6)
    def test_spectrum_rejects_sample_gap(self):
        with self.assertRaises(ValueError):spectrum(self.vibration_csv(True),Path(self.tmp.name)/'spectrum.csv')

if __name__=='__main__':unittest.main()
