"""Desktop session lifecycle; single serial owner and explicit preview isolation."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import sys
import threading
import time
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication, QFileDialog, QMessageBox
from gui_telemetry import Telemetry
from gui_transport import SerialSession, ports
from gui_view import DashboardView
from gui_presenter import DashboardPresenter
from enter_dfu import request_dfu


class MonitorApp(DashboardPresenter, DashboardView):
    def __init__(self, port=None, capture_path=None, start_reader=True, demo=False):
        super().__init__()
        self.model=Telemetry(); self.session=SerialSession(capture_path)
        self.link_open=False; self.demo=False; self.dfu_mode=False; self.pending=None
        self.dfu_thread=None; self.dfu_results=queue.Queue(); self._last_table=0; self._last_demo=0; self.closing=False
        self.setup_view(port)
        self.timer=QTimer(self); self.timer.timeout.connect(self.poll); self.timer.start(50)
        if demo: self.start_demo()
        elif start_reader: self.session.start(port)
        self.render()

    def refresh_ports(self, selected=None):
        selected=selected or self.port_combo.currentData()
        self.port_combo.clear(); self.port_combo.addItem("Auto-detect STM32",None)
        names=sorted(p.device for p in ports())
        if selected and selected not in names: names.append(selected)
        for name in names: self.port_combo.addItem(name,name)
        self.port_combo.setCurrentIndex(max(0,self.port_combo.findData(selected)))

    def request_transition(self, operation):
        if self.pending or (self.dfu_thread and self.dfu_thread.is_alive()): return
        self.pending=operation; self.session.stop.set(); self.link_open=False
        self.set_controls(False); self.finish_transition()

    def set_controls(self, enabled):
        for widget in (self.port_combo,self.refresh_button,self.connect_button,self.disconnect_button,self.preview_button,self.record_button): widget.setEnabled(enabled)

    def finish_transition(self):
        if self.closing: return
        if self.session.running():
            QTimer.singleShot(50,self.finish_transition)
            return
        while not self.session.events.empty(): self.session.events.get_nowait()
        operation=self.pending; self.pending=None
        if operation != "disconnect" or self.demo: self.model=Telemetry()
        self.demo=False; self.dfu_mode=False; self.link_open=False; self._last_table=0
        self.set_controls(True)
        if operation=="demo": self.start_demo()
        elif operation=="connect": self.session.start(self.port_combo.currentData())
        elif operation=="dfu":
            self.set_controls(False); self.dfu_status.setText("Serial port released. Requesting and verifying ROM DFU…")
            self.dfu_thread=threading.Thread(target=self.dfu_worker,args=(self._dfu_port,),daemon=True); self.dfu_thread.start()
        else: self.footer.setText("Disconnected. Connect the controller or open Preview.")
        self.render()

    def start_demo(self):
        from gui_demo import position
        if self.session.running(): raise RuntimeError("Preview requires the serial reader to stop first")
        self.demo=True; self._demo_start=time.monotonic(); self._last_demo=0
        self.model.track.extend(position(t) for t in range(-110,1))
        self.model.event("Preview started — simulated data; serial connection closed")

    def toggle_record(self):
        if self.demo: return
        if self.session.capture_path: self.session.capture_path=None
        else:
            filename,_=QFileDialog.getSaveFileName(self,"Save USB capture",f"stm32fc-{datetime.now():%Y%m%d-%H%M%S}.csv","CSV files (*.csv);;All files (*)")
            if not filename: return
            self.session.capture_path=filename
        self.record_button.setText("Stop capture" if self.session.capture_path else "Record USB")
        self.request_transition("connect")

    def enter_dfu(self):
        if self.demo or not self.link_open or self.pending or self.dfu_mode: return
        choice=QMessageBox.question(self,"Enter ROM DFU?","This stops stabilization and PWM. Confirm the aircraft is on the ground, propulsion disconnected, CH5 disarmed and throttle low.",QMessageBox.Yes|QMessageBox.Cancel,QMessageBox.Cancel)
        if choice!=QMessageBox.Yes: return
        self._dfu_port=self.session.current_port
        if self._dfu_port: self.request_transition("dfu")

    def dfu_worker(self, port):
        try: self.dfu_results.put((True,request_dfu(port)))
        except Exception as exc: self.dfu_results.put((False,str(exc)))

    def poll(self):
        if self.demo:
            from gui_demo import frame
            now=time.monotonic()
            if now-self._last_demo>=.05:
                self._last_demo=now
                for line in frame(now-self._demo_start): self.model.feed(line)
        else:
            # Bound each drain so a diagnostic burst cannot starve the UI.
            for _ in range(500):
                try: kind,value=self.session.events.get_nowait()
                except queue.Empty: break
                if kind=="line": self.model.feed(value)
                elif kind=="connected":
                    self.link_open=True; self.model=Telemetry(); self.model.event("USB connected: "+value); self.footer.setText("Streaming from "+value)
                elif kind=="disconnected": self.link_open=False
                elif kind=="status": self.footer.setText(value)
                elif kind=="capture_error":
                    self.model.event(value); self.record_button.setText("Record USB")
        try:
            success,detail=self.dfu_results.get_nowait(); self.dfu_mode=success; self.set_controls(True)
            self.dfu_status.setText("DFU VERIFIED · 0483:df11. After flashing/restarting, use Reconnect." if success else "DFU NOT VERIFIED: "+detail)
            self.model.event("ROM DFU verified" if success else "DFU failed: "+detail)
        except queue.Empty: pass
        self.render()

    def export_snapshot(self):
        path,_=QFileDialog.getSaveFileName(self,"Export local diagnostic snapshot","stm32fc-snapshot.json","JSON (*.json)")
        if not path: return
        now=self.model.clock()
        data={"source":"SIMULATED" if self.demo else "USB", "connected":self.link_open,
              "captured_at":datetime.now().isoformat(),"messages":{k:{"line":v,"age_seconds":now-t} for k,(v,t) in self.model.raw.items()}}
        try: Path(path).write_text(json.dumps(data,indent=2)+"\n",encoding="utf-8")
        except OSError as exc: QMessageBox.warning(self,"Snapshot could not be saved",str(exc))

    def closeEvent(self,event):
        self.closing=True
        self.timer.stop(); self.session.stop.set()
        if self.session.thread: self.session.thread.join(timeout=1)
        event.accept()


def main():
    parser=argparse.ArgumentParser(description="STM32FC Ground Station")
    parser.add_argument("--port","-p"); parser.add_argument("--capture")
    parser.add_argument("--demo",action="store_true",help="simulated preview; never opens the serial port")
    parser.add_argument("--self-test",metavar="REPORT",help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.self_test:
        from gui_selftest import run
        return run(args.self_test)
    app=QApplication(sys.argv); app.setStyle("Fusion")
    def error_hook(kind,error,tb):
        import traceback,tempfile
        log=Path(os.environ.get("LOCALAPPDATA",tempfile.gettempdir()))/"STM32FC-GUI/error.log"
        try:
            log.parent.mkdir(parents=True,exist_ok=True)
            with log.open("a",encoding="utf-8") as f: traceback.print_exception(kind,error,tb,file=f)
        except OSError: pass
        QMessageBox.critical(None,"STM32FC GUI error",str(error)+f"\n\nDetails: {log}")
    sys.excepthook=error_hook
    window=MonitorApp(args.port,args.capture,demo=args.demo); window.show()
    return app.exec()
