"""Serial ownership, partial-line, discovery and overload tests; no hardware."""
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch, MagicMock
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
import gui_transport as transport


class ConnectionTests(unittest.TestCase):
    def test_discovery_requires_exactly_one_matching_board(self):
        def dev(name,vid=0x0483):return SimpleNamespace(device=name,vid=vid,pid=0x5740)
        for items,expected in (([],None),([dev("COM8")],"COM8"),([dev("COM8"),dev("COM9")],None),([dev("COM1",0x1234)],None)):
            with patch.object(transport,"ports",return_value=items):self.assertEqual(transport.detect_port(),expected)

    def test_auto_detect_is_retried_after_hotplug(self):
        s=transport.SerialSession()
        def opening(port,*args,**kwargs):
            self.assertEqual(port,"COM19");s.stop.set();raise transport.serial.SerialException("done")
        with patch.object(transport,"detect_port",side_effect=[None,"COM19"]),patch.object(s.stop,"wait"),patch.object(transport.serial,"Serial",side_effect=opening) as opened:
            s.read_loop(None)
        self.assertEqual(opened.call_count,1)

    def test_serial_handle_closes_and_partial_lines_are_preserved(self):
        s=transport.SerialSession();device=MagicMock();device.__enter__.return_value=device;device.in_waiting=128
        chunks=iter([b"ATT,10,",b"",b"5,90\r\nBMP,1000,20,12\n"])
        def read(n):
            try:return next(chunks)
            except StopIteration:s.stop.set();return b""
        device.read.side_effect=read
        with patch.object(transport.serial,"Serial",return_value=device):s.read_loop("COM8")
        events=list(s.events.queue)
        self.assertIn(("line","ATT,10,5,90"),events)
        self.assertIn(("line","BMP,1000,20,12"),events)
        device.__exit__.assert_called_once();self.assertIsNone(s.current_port)

    def test_cannot_start_a_second_owner(self):
        s=transport.SerialSession();s.thread=MagicMock();s.thread.is_alive.return_value=True
        with self.assertRaisesRegex(RuntimeError,"Previous serial"):s.start("COM8")

    def test_capture_failure_does_not_stop_telemetry(self):
        s=transport.SerialSession("capture.csv");device=MagicMock();device.__enter__.return_value=device;device.in_waiting=128
        count=[0]
        def read(n):
            count[0]+=1
            if count[0]>1:s.stop.set();return b""
            return b"ATT,10,5,90\n"
        device.read.side_effect=read
        capture=MagicMock();capture.write.side_effect=OSError("Disk full")
        with patch("builtins.open",return_value=capture),patch.object(transport.serial,"Serial",return_value=device):s.read_loop("COM8")
        self.assertIn(("line","ATT,10,5,90"),list(s.events.queue))
        self.assertIsNone(s.capture_path)
        self.assertTrue(any(k=="capture_error" for k,v in s.events.queue))

    def test_queue_is_bounded_and_keeps_latest_status(self):
        s=transport.SerialSession()
        for i in range(3500):s.emit("line",str(i))
        s.emit("disconnected",None)
        self.assertEqual(s.events.qsize(),3000);self.assertEqual(s.dropped,501)
        self.assertEqual(list(s.events.queue)[-1],("disconnected",None))


if __name__=="__main__":unittest.main()
