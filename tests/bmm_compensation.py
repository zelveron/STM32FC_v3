"""Independent compensation checker versus the exact unmodified Bosch C API."""
import ctypes
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import check_bmm350_compensation as audit


class CompensationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.lib_path = Path(cls.tmp.name)/'bosch.so'
        subprocess.run(['gcc', '-shared', '-fPIC', '-O2', str(ROOT/'tests/bmm350_compensation_reference.c'),
                        '-o', str(cls.lib_path)], check=True)
        cls.lib = ctypes.CDLL(str(cls.lib_path))
        cls.lib.reference_compensation.argtypes = [ctypes.POINTER(t) for t in
                                                  (ctypes.c_uint16,ctypes.c_int32,ctypes.c_float,ctypes.c_float)]

    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()

    def test_signed_boundaries(self):
        for bits in (8,12,16,24):
            self.assertEqual(audit.signed((1<<bits)-1,bits),-1)
            self.assertEqual(audit.signed(1<<(bits-1),bits),-(1<<(bits-1)))
            self.assertEqual(audit.signed((1<<(bits-1))-1,bits),(1<<(bits-1))-1)

    def test_decoder_and_arithmetic_match_bosch_across_1000_fixtures(self):
        rng=random.Random(350)
        for _ in range(1000):
            words=[rng.randrange(65536) for _ in range(32)]
            raw=[rng.randrange(-550000,550001) for _ in range(3)]+[rng.randrange(40000,70000)]
            out=(ctypes.c_float*4)();trim=(ctypes.c_float*19)()
            self.assertEqual(self.lib.reference_compensation((ctypes.c_uint16*32)(*words),
                             (ctypes.c_int32*4)(*raw),out,trim),0)
            c=audit.decode_otp(words)
            for i,name in enumerate(audit.TRIM_NAMES):
                self.assertEqual(audit.float_bits(c[name]),audit.float_bits(trim[i]),name)
            expected=audit.compensate(raw,c)['compensated']
            for i,name in enumerate(('x','y','z','temp')):
                self.assertAlmostEqual(expected[name],out[i],delta=0.01+abs(out[i])*2e-6)

    def snapshot(self):
        words=[0]*32;words[0]=0x1234;words[13]=0xff01
        c=audit.decode_otp(words);lines=['MAG_OTP_STATUS,passes=3,mismatch=0']
        for p in range(3):
            for offset in range(0,32,8):
                lines.append(f'MAG_OTP,pass={p},offset={offset},words='+ '/'.join(f'{w:X}' for w in words[offset:offset+8]))
        for offset in (0,8,16):
            lines.append(f'MAG_TRIM,offset={offset},bits='+ '/'.join(f'{audit.float_bits(c[k]):X}' for k in audit.TRIM_NAMES[offset:offset+8]))
        raw=[-509024,-446675,-959,50558];out=audit.compensate(raw,c)['compensated']
        lines.append('MAG_DATA,'+','.join(f'{k}={v:.2f}' for k,v in out.items())+',raw='+ '/'.join(map(str,raw)))
        return lines

    def run_audit(self,lines):
        p=Path(self.tmp.name)/'capture.txt';p.write_text('\n'.join(lines)+'\n');return audit.audit(p)

    def test_complete_snapshot_rounding_and_rejected_sample(self):
        result=self.run_audit(self.snapshot())
        self.assertTrue(result['calculation_matches']);self.assertLess(result['first_sample_stages']['board']['x'],-2000)

    def test_incomplete_or_unstable_otp_is_not_accepted(self):
        lines=self.snapshot()
        with self.assertRaises(ValueError): self.run_audit(lines[1:])
        with self.assertRaises(ValueError): self.run_audit(lines[:1]+lines[2:])
        lines[5]=lines[5].replace('1234','1235')
        with self.assertRaises(ValueError): self.run_audit(lines)

    def test_conflicting_repeat_is_not_silently_overwritten(self):
        lines=self.snapshot();lines.append(lines[1].replace('1234','1235'))
        with self.assertRaises(ValueError): self.run_audit(lines)

    def test_coefficient_or_output_disagreement_is_reported(self):
        lines=self.snapshot();lines[13]=lines[13].replace('3E4CCCCD','0')
        self.assertFalse(self.run_audit(lines)['calculation_matches'])
        lines=self.snapshot();lines[-1]=lines[-1].replace('temp=', 'ignored=0,temp=100')
        self.assertFalse(self.run_audit(lines)['calculation_matches'])


if __name__=='__main__': unittest.main()
