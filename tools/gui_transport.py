"""One serial owner, bounded queue and optional capture; no UI dependencies."""
import queue
import threading
import serial
from serial.tools import list_ports


def ports():
    return list(list_ports.comports())


def detect_port():
    candidates = [p.device for p in ports() if p.vid == 0x0483 and p.pid == 0x5740]
    return candidates[0] if len(candidates) == 1 else None


class SerialSession:
    def __init__(self, capture_path=None):
        self.events = queue.Queue(maxsize=3000)
        self.stop = threading.Event()
        self.thread = None
        self.current_port = None
        self.capture_path = capture_path
        self.dropped = 0

    def emit(self, kind, value):
        try:
            self.events.put_nowait((kind, value))
        except queue.Full:
            # Keep the latest status/data; never block the serial reader on UI.
            try:
                self.events.get_nowait()
            except queue.Empty:
                pass
            self.dropped += 1
            self.events.put_nowait((kind, value))

    def running(self):
        return bool(self.thread and self.thread.is_alive())

    def start(self, port=None):
        if self.running():
            raise RuntimeError("Previous serial connection is still closing")
        self.stop.clear()
        self.thread = threading.Thread(target=self.read_loop, args=(port,), daemon=True)
        self.thread.start()

    def read_loop(self, selected):
        capture = None
        try:
            if self.capture_path:
                try:
                    capture = open(self.capture_path, "a", encoding="utf-8", buffering=1)
                except OSError as exc:
                    self.capture_path = None
                    self.emit("capture_error", f"USB capture stopped: {exc}")
            while not self.stop.is_set():
                try:
                    port = selected or detect_port()
                    if port is None:
                        self.emit("status", "Connect one STM32, or select a COM port if several boards are attached.")
                        self.stop.wait(1)
                        continue
                    with serial.Serial(port, 115200, timeout=.2) as device:
                        self.current_port = port
                        device.reset_input_buffer()
                        self.emit("connected", port)
                        pending = bytearray()
                        while not self.stop.is_set():
                            # Preserve lines split by a USB timeout; cap malformed input.
                            chunk = device.read(min(max(device.in_waiting, 1), 4096))
                            pending.extend(chunk)
                            while b"\n" in pending:
                                raw, _, pending = pending.partition(b"\n")
                                line = raw.decode("utf-8", errors="replace").strip()
                                if line and len(line) <= 4096:
                                    if capture:
                                        try:
                                            capture.write(line + "\n")
                                        except OSError as exc:
                                            try: capture.close()
                                            except OSError: pass
                                            capture = None
                                            self.capture_path = None
                                            self.emit("capture_error", f"USB capture stopped: {exc}")
                                    self.emit("line", line)
                            if len(pending) > 4096:
                                pending.clear()
                                self.emit("status", "Discarded an oversized USB line")
                except (serial.SerialException, OSError) as exc:
                    self.emit("status", f"Serial unavailable: {exc}")
                finally:
                    self.current_port = None
                    self.emit("disconnected", None)
                self.stop.wait(1)
        except OSError as exc:
            self.emit("status", f"Capture file unavailable: {exc}")
        finally:
            if capture:
                try: capture.close()
                except OSError as exc: self.emit("capture_error", f"USB capture close failed: {exc}")
