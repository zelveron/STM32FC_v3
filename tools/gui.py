#!/usr/bin/env python3
"""
Live monitor for the STM32F407 BMP581 + dual IMU + SAM-M10Q controller.

Reads tagged CSV over USB CDC and shows:
  - dual IMU connection status
  - BMP581: pressure / temperature / altitude
  - dual IMU: accel (g) and gyro (deg/s)
  - Attitude: roll / pitch / yaw (complementary filter) + artificial horizon
  - uBlox GNSS: position / altitude / satellites / fix

Usage:
    python3 tools/gui.py                 # auto-detect port
    python3 tools/gui.py --port /dev/ttyACM0
"""

import argparse
import math
import os
from pathlib import Path
import queue
import sys
import threading
import time

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox
from enter_dfu import request_dfu


def detect_port():
    stm_ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == 0x0483 and p.pid == 0x5740]
    # Never choose an arbitrary serial device or guess between multiple boards.
    return stm_ports[0] if len(stm_ports) == 1 else None


class MonitorApp:
    def __init__(self, root, port=None, capture_path=None, start_reader=True):
        self.root = root
        self.port = port
        self.capture_path = capture_path
        self.imu_model = "IMU"
        self.q = queue.Queue()
        self._reader_stop = threading.Event()
        self._dfu_busy = False
        self._dfu_mode = False
        self._reconnecting = False
        self._link_open = False
        self._last_data = time.time()
        self._last_att = self._last_bmp = self._last_gps = 0.0
        root.title("STM32FC — Flight Controller Monitor")
        root.geometry("920x960")
        root.minsize(720, 500)
        root.configure(bg="#1e1e1e")

        bg = "#1e1e1e"
        fg = "#e0e0e0"
        green = "#2ecc71"
        red = "#e74c3c"

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TLabel", background=bg, foreground=fg, font=("Helvetica", 12))
        style.configure("Header.TLabel", font=("Helvetica", 14, "bold"))
        style.configure("Value.TLabel", font=("Helvetica", 16, "bold"), foreground="#ffffff")
        style.configure("TFrame", background=bg)
        style.configure("TLabelframe", background=bg, foreground=fg)
        style.configure("TLabelframe.Label", background=bg, foreground=fg)

        # Keep every panel reachable on laptop screens and at Windows DPI scaling.
        shell = ttk.Frame(root)
        shell.pack(fill="both", expand=True)
        canvas = tk.Canvas(shell, bg=bg, highlightthickness=0)
        scroll = ttk.Scrollbar(shell, orient="vertical", command=canvas.yview)
        scroll.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        canvas.configure(yscrollcommand=scroll.set)
        body = ttk.Frame(canvas)
        body_id = canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind("<Configure>", lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(body_id, width=event.width))
        root.bind_all("<MouseWheel>", lambda event: canvas.yview_scroll(-int(event.delta / 120), "units"))
        root = body

        self.amber = "#fbbf24"
        self.blue = "#3b82f6"

        # --- BMI status header ---
        self.status_var = tk.StringVar(value="dual IMU: waiting...")
        status_lbl = tk.Label(root, textvariable=self.status_var, font=("Helvetica", 14, "bold"),
                              bg=bg, fg=red)
        status_lbl.pack(pady=(12, 8))

        maintenance = ttk.Frame(root)
        maintenance.pack(fill="x", padx=16, pady=4)
        ttk.Label(maintenance, text="Port:").pack(side="left", padx=(0, 6))
        self.port_var = tk.StringVar(value=port or "Auto-detect STM32")
        self.port_combo = ttk.Combobox(maintenance, textvariable=self.port_var, state="readonly", width=22)
        self.port_combo.pack(side="left", padx=(0, 8))
        self.refresh_button = ttk.Button(maintenance, text="Refresh ports", command=self._refresh_ports)
        self.refresh_button.pack(side="left", padx=(0, 8))
        self._refresh_ports()
        self.dfu_button = ttk.Button(maintenance, text="Enter DFU", command=self._enter_dfu)
        self.dfu_button.pack(side="right")
        self.dfu_button.configure(state="disabled")
        self.resume_button = ttk.Button(maintenance, text="Reconnect", command=self._resume_reader)
        self.resume_button.pack(side="left", padx=8)
        self.maintenance_var = tk.StringVar(value="DFU: ground use only; disarm CH5 and lower throttle.")
        ttk.Label(root, textvariable=self.maintenance_var, wraplength=850).pack(padx=16, pady=4)

        # --- Flight control section (mode manager / arming / failsafe) ---
        fc_frame = ttk.LabelFrame(root, text="Flight control")
        fc_frame.pack(fill="x", padx=16, pady=6)

        self.mode_var = tk.StringVar(value="--")
        self.mode_lbl = tk.Label(fc_frame, textvariable=self.mode_var,
                                 font=("Helvetica", 20, "bold"), bg=bg, fg=fg)
        self.mode_lbl.pack(pady=(6, 2))

        self.mode_sub_var = tk.StringVar(value="requested --   ·   last change --")
        tk.Label(fc_frame, textvariable=self.mode_sub_var, bg=bg, fg="#888888",
                 font=("Helvetica", 10)).pack(pady=(0, 4))

        self.armed_var = tk.StringVar(value="--")
        self.failsafe_var = tk.StringVar(value="--")
        self.lockout_var = tk.StringVar(value="--")
        self.integ_var = tk.StringVar(value="--")
        self.cal_var = tk.StringVar(value="--")
        self._row(fc_frame, "Armed", self.armed_var)
        self._row(fc_frame, "Failsafe", self.failsafe_var)
        self._row(fc_frame, "Assist lockout", self.lockout_var)
        self._row(fc_frame, "Stab integrators", self.integ_var)
        self._row(fc_frame, "Gyro cal", self.cal_var)
        self.imu_diag_var = tk.StringVar(value="--")
        self.bmp_health_var = tk.StringVar(value="--")
        self.sd_var = tk.StringVar(value="--")
        self._row(fc_frame, "IMU diagnostics", self.imu_diag_var)
        self._row(fc_frame, "Barometer health", self.bmp_health_var)
        self._row(fc_frame, "SD / USB", self.sd_var)

        # --- RC in / servo out ---
        io_frame = ttk.LabelFrame(root, text="RC in  ·  servo out (µs)")
        io_frame.pack(fill="x", padx=16, pady=6)
        self.rc_var = tk.StringVar(value="A --  E --  T --  R --   arm --  mode --")
        self.out_var = tk.StringVar(value="ail --/--  elev --/--  rud --  nose --  esc --/--")
        self._row(io_frame, "RC", self.rc_var)
        self._row(io_frame, "Out", self.out_var)

        # --- BMP section ---
        bmp_frame = ttk.LabelFrame(root, text="BMP581 (pressure)")
        bmp_frame.pack(fill="x", padx=16, pady=6)

        self.pressure_var = tk.StringVar(value="-- hPa")
        self.temp_var = tk.StringVar(value="-- °C")
        self.alt_var = tk.StringVar(value="-- m")

        self._row(bmp_frame, "Pressure", self.pressure_var)
        self._row(bmp_frame, "Temperature", self.temp_var)
        self._row(bmp_frame, "Altitude", self.alt_var)

        # --- BMI section ---
        bmi_frame = ttk.LabelFrame(root, text="dual IMU (IMU)")
        bmi_frame.pack(fill="x", padx=16, pady=6)

        self.acc_var = tk.StringVar(value="--, --, -- g")
        self.gyr_var = tk.StringVar(value="--, --, -- dps")

        self._row(bmi_frame, "Accel", self.acc_var)
        self._row(bmi_frame, "Gyro", self.gyr_var)

        # --- GPS section ---
        gps_frame = ttk.LabelFrame(root, text="uBlox GNSS (NMEA)")
        gps_frame.pack(fill="x", padx=16, pady=6)

        self.pos_var = tk.StringVar(value="--, --")
        self.gps_alt_var = tk.StringVar(value="-- m")
        self.sats_var = tk.StringVar(value="--")
        self.fix_var = tk.StringVar(value="--")
        self.gps_time_var = tk.StringVar(value="--")
        self.gps_speed_var = tk.StringVar(value="--")

        self._row(gps_frame, "Position", self.pos_var)
        self._row(gps_frame, "GPS Altitude", self.gps_alt_var)
        self._row(gps_frame, "Satellites", self.sats_var)
        self._row(gps_frame, "Fix", self.fix_var)
        self._row(gps_frame, "Time (UTC)", self.gps_time_var)
        self._row(gps_frame, "Speed", self.gps_speed_var)

        # --- Attitude section ---
        att_frame = ttk.LabelFrame(root, text="Attitude (quaternion fusion; yaw is relative)")
        att_frame.pack(fill="x", padx=16, pady=6)

        self.roll_var = tk.StringVar(value="-- °")
        self.pitch_var = tk.StringVar(value="-- °")
        self.yaw_var = tk.StringVar(value="-- °")

        self._row(att_frame, "Roll", self.roll_var)
        self._row(att_frame, "Pitch", self.pitch_var)
        self._row(att_frame, "Yaw", self.yaw_var)

        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        self._yaw_deg = 0.0
        self._last_mode_change = "--"

        self.horizon = tk.Canvas(att_frame, width=240, height=150,
                                 bg="#000000", highlightthickness=0)
        self.horizon.pack(pady=6)
        self._draw_horizon()

        diagnostics = ttk.LabelFrame(root, text="All live parameters — latest message per tag / task")
        diagnostics.pack(fill="both", padx=16, pady=8)
        self.diag_tree = ttk.Treeview(diagnostics, columns=("tag", "value"), show="headings", height=16)
        self.diag_tree.heading("tag", text="Message / task")
        self.diag_tree.heading("value", text="Latest values (raw tagged CSV)")
        self.diag_tree.column("tag", width=170, stretch=False)
        self.diag_tree.column("value", width=1050, stretch=False)
        diag_x = ttk.Scrollbar(diagnostics, orient="horizontal", command=self.diag_tree.xview)
        diag_y = ttk.Scrollbar(diagnostics, orient="vertical", command=self.diag_tree.yview)
        self.diag_tree.configure(xscrollcommand=diag_x.set, yscrollcommand=diag_y.set)
        diag_y.pack(side="right", fill="y")
        diag_x.pack(side="bottom", fill="x")
        self.diag_tree.pack(fill="both", expand=True)
        ttk.Label(diagnostics, text="Includes scheduler, CRSF, estimator and logging diagnostics. Last values may be stale after disconnect.",
                  wraplength=800).pack(padx=6, pady=4)

        # --- footer ---
        self.footer_var = tk.StringVar(value=f"Connecting to {port or 'STM32 (auto-detect)'} ...")
        ttk.Label(self.root, textvariable=self.footer_var, foreground="#888888").pack(side="bottom", pady=6)

        self._connected = False
        self.status_lbl = status_lbl
        self.red = red
        self.green = green

        self.reader = threading.Thread(target=self._read_loop, args=(port,), daemon=True)
        if start_reader:
            self.reader.start()
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        root.after(50, self._poll)

    def _close(self):
        self._reader_stop.set()
        self.root.destroy()

    def _enter_dfu(self):
        if self._dfu_busy or self._dfu_mode or self._reconnecting or not self._link_open:
            return
        self._dfu_busy = True
        self.dfu_button.configure(state="disabled")
        self.resume_button.configure(state="disabled")
        self.port_combo.configure(state="disabled")
        self.maintenance_var.set("Releasing serial port, requesting DFU, then verifying 0483:df11...")
        self._reader_stop.set()
        threading.Thread(target=self._dfu_worker, daemon=True).start()

    def _dfu_worker(self):
        # Only this worker opens the command port, after the reader has closed it.
        self.reader.join(timeout=3)
        if self.reader.is_alive():
            self.q.put(("dfu_error", "Serial reader did not stop; no DFU command sent."))
            return
        try:
            result = request_dfu(self.port)
            self.q.put(("dfu_ok", result))
        except Exception as exc:
            self.q.put(("dfu_error", str(exc)))

    def _resume_reader(self):
        if self._dfu_busy or self._reconnecting:
            return
        self._reconnecting = True
        self._reader_stop.set()
        self.resume_button.configure(state="disabled")
        self.dfu_button.configure(state="disabled")
        self.port_combo.configure(state="disabled")
        self.maintenance_var.set("Releasing previous connection...")
        self._finish_reconnect()

    def _finish_reconnect(self):
        # Do not block Tk, or start a second owner of the same serial handle.
        if self.reader.is_alive():
            self.root.after(100, self._finish_reconnect)
            return
        while not self.q.empty():
            self.q.get_nowait()
        self._link_open = False
        selected = self.port_var.get()
        self.port = None if selected == "Auto-detect STM32" else selected
        self._reader_stop.clear()
        self._dfu_mode = False
        self._reconnecting = False
        self.port_combo.configure(state="readonly")
        self.resume_button.configure(state="normal")
        self._last_att = self._last_bmp = self._last_gps = 0
        self.status_var.set("dual IMU: waiting...")
        self.status_lbl.configure(fg=self.red)
        self.maintenance_var.set("Monitoring resumed. After flashing, allow several seconds for USB.")
        self.reader = threading.Thread(target=self._read_loop, args=(self.port,), daemon=True)
        self.reader.start()

    def _refresh_ports(self):
        ports = sorted(p.device for p in serial.tools.list_ports.comports())
        selected = self.port_var.get()
        if selected != "Auto-detect STM32" and selected not in ports:
            ports.append(selected)
        self.port_combo.configure(values=["Auto-detect STM32"] + ports)

    def _row(self, parent, label, var):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", padx=8, pady=3)
        ttk.Label(frame, text=label, width=14, anchor="w").pack(side="left")
        ttk.Label(frame, textvariable=var, style="Value.TLabel", anchor="e", wraplength=600).pack(side="right", fill="x", expand=True)

    @staticmethod
    def _kv(items):
        """['a=1','b=2'] -> {'a':'1','b':'2'}; tolerates stray non-kv tokens."""
        out = {}
        for it in items:
            if "=" in it:
                k, _, v = it.partition("=")
                out[k] = v
        return out

    @staticmethod
    def _fix_label(fix):
        return {0: "No fix", 1: "GPS fix", 2: "DGPS", 4: "RTK fixed",
                5: "RTK float", 6: "Dead reckoning"}.get(fix, f"fix {fix}")

    def _draw_horizon(self):
        c = self.horizon
        c.delete("all")
        w = int(c["width"]); h = int(c["height"])
        cx, cy = w / 2.0, h / 2.0
        if not self._last_att:
            c.create_text(cx, cy, text="ATTITUDE UNAVAILABLE", fill="#e74c3c")
            return
        roll = self._roll_deg
        pitch = self._pitch_deg

        # Pitch: positive = nose up -> horizon drops (more sky visible).
        horizon_y = cy + pitch * 2.0   # pixels

        # Roll: rotate the horizon line about the canvas centre.
        rad = math.radians(roll)
        cosr, sinr = math.cos(rad), math.sin(rad)
        ext = w + h   # endpoints well outside the canvas

        def rot(x, y):
            dx, dy = x - cx, y - cy
            return (cx + dx * cosr - dy * sinr, cy + dx * sinr + dy * cosr)

        x0, y0 = rot(cx - ext, horizon_y)
        x1, y1 = rot(cx + ext, horizon_y)

        c.create_rectangle(0, 0, w, h, fill="#3b82f6", outline="")             # sky
        c.create_polygon(x0, y0, x1, y1, w + ext, h + ext, -ext, h + ext,
                         fill="#7a4b22", outline="")                            # ground
        c.create_line(x0, y0, x1, y1, fill="#ffffff", width=2)                  # horizon
        c.create_line(cx - 20, cy, cx + 20, cy, fill="#fbbf24", width=2)        # wings
        c.create_line(cx, cy - 12, cx, cy + 12, fill="#fbbf24", width=2)        # nose/tail
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#fbbf24", outline="")

    def _read_loop(self, port):
        """Reconnect-forever serial reader. Pushes (kind, payload) into the queue."""
        try:
            capture = open(self.capture_path, "a", encoding="utf-8", buffering=1) if self.capture_path else None
        except OSError as exc:
            self.q.put(("status", f"Cannot open capture file: {exc}"))
            return
        while not self._reader_stop.is_set():
            try:
                current_port = port or detect_port()
                if current_port is None:
                    self.q.put(("status", "Waiting for one STM32 board. If several are connected, choose a port and Reconnect."))
                    self._reader_stop.wait(1)
                    continue
                ser = serial.Serial(current_port, 115200, timeout=0.2)
            except Exception as exc:
                self.q.put(("status", f"cannot open {port}: {exc}"))
                self._reader_stop.wait(2)
                continue

            self.q.put(("connected", current_port))
            try:
                ser.reset_input_buffer()
                while not self._reader_stop.is_set():
                    raw = ser.readline()
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        if capture:
                            capture.write(line + "\n")
                        self.q.put(("line", line))
            except Exception as exc:
                self.q.put(("status", f"disconnected: {exc}"))
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
                self.q.put(("disconnected", None))
            self._reader_stop.wait(1)
        if capture:
            capture.close()

    def _poll(self):
        try:
            while True:
                kind, val = self.q.get_nowait()
                if kind == "line":
                    self._last_data = time.time()
                    self._handle_line(val)
                elif kind == "status":
                    self.footer_var.set(val)
                elif kind == "connected":
                    self.port = val
                    self._link_open = True
                    self._last_data = time.time()
                    self.footer_var.set(f"Connected: {val}")
                    if not self._dfu_busy and not self._reconnecting:
                        self.dfu_button.configure(state="normal")
                elif kind == "disconnected":
                    self._link_open = False
                    self.dfu_button.configure(state="disabled")
                elif kind in ("dfu_ok", "dfu_error"):
                    self._dfu_busy = False
                    self._dfu_mode = kind == "dfu_ok"
                    self.resume_button.configure(state="normal")
                    self.port_combo.configure(state="readonly")
                    self.dfu_button.configure(state="disabled")
                    self.maintenance_var.set("DFU VERIFIED (0483:df11). Flash the selected image, then Reconnect."
                                             if self._dfu_mode else "DFU NOT VERIFIED: " + val)
                    self.footer_var.set("Monitoring paused; serial port released")
                    if self._dfu_mode:
                        self.status_var.set("ROM DFU active; flight application stopped")
                        self.status_lbl.configure(fg=self.amber)
                        self.armed_var.set("Application stopped")
                        self.out_var.set("PWM stopped in ROM DFU")
        except queue.Empty:
            pass

        # Data watchdog: if the stream stalls, surface it instead of freezing.
        if self._link_open and not self._reader_stop.is_set() and time.time() - self._last_data > 3:
            self.footer_var.set(f"Connected: {self.port} — no recent data")

        self._expire_sensor_data(time.time())
        self.root.after(50, self._poll)

    def _expire_sensor_data(self, now):
        if now - self._last_att > 0.3:
            self._last_att = 0
            for var in (self.roll_var, self.pitch_var, self.yaw_var, self.acc_var, self.gyr_var):
                var.set("STALE / unavailable")
            self._draw_horizon()
        if now - self._last_bmp > 0.3:
            for var in (self.pressure_var, self.temp_var, self.alt_var):
                var.set("STALE / unavailable")
        if now - self._last_gps > 2.0:
            self.fix_var.set("STALE / no valid fix")
            for var in (self.pos_var, self.gps_alt_var, self.gps_speed_var):
                var.set("--")

    def _handle_line(self, line):
        if line.startswith("{"):
            self.status_var.set("Sensor-test firmware; flight-app upload pending")
            self.status_lbl.configure(fg=self.amber)
            self.footer_var.set("JSON sensor-test stream; STM32FC tagged CSV required")
            return
        parts = line.split(",")
        if not parts:
            return
        tag = parts[0]
        if hasattr(self, "diag_tree"):
            key = tag + (":" + parts[1] if tag == "SCHED" and len(parts) > 1 else "")
            values = (key, ",".join(parts[2:] if tag == "SCHED" else parts[1:]))
            if self.diag_tree.exists(key):
                self.diag_tree.item(key, values=values)
            elif len(self.diag_tree.get_children()) < 100:
                self.diag_tree.insert("", "end", iid=key, values=values)

        if tag == "IMU_CONFIG":
            kv = self._kv(parts[1:])
            self.imu_model = kv.get("model", "IMU")
            if hasattr(self, "imu_diag_var"):
                self.imu_diag_var.set(f"Errors {kv.get('error0', '--')} / {kv.get('error1', '--')}  ·  "
                                      f"registers {kv.get('regs0', '--')} / {kv.get('regs1', '--')}")
            if hasattr(self, "root"):
                self.root.title(f"STM32FC — dual {self.imu_model} + BMP581 + GNSS")

        elif tag == "BMP_HEALTH":
            kv = self._kv(parts[1:])
            if hasattr(self, "bmp_health_var"):
                self.bmp_health_var.set(f"{'healthy' if kv.get('valid') == '1' else 'unavailable'}  ·  error {kv.get('error', '--')}")

        elif tag == "SD_DBG" and len(parts) > 2:
            kv = self._kv(parts[3:])
            self.sd_var.set(f"{'logging ' + parts[2] if parts[1] == '1' else 'not logging'}  ·  "
                            f"bytes {kv.get('bytes', '--')}  ·  log drops {kv.get('log_drops', '--')}  ·  USB drops {kv.get('usb_drops', '--')}")

        elif tag == "BMP" and len(parts) == 4:
            try:
                p = float(parts[1])
                t = float(parts[2])
                a = float(parts[3])
            except ValueError:
                return
            if not all(math.isfinite(v) for v in (p, t, a)):
                return
            self._last_bmp = time.time()
            self.pressure_var.set(f"{p:.3f} hPa")
            self.temp_var.set(f"{t:.2f} °C")
            self.alt_var.set(f"{a:.2f} m")

        elif tag == "BMI" and len(parts) == 7:
            try:
                vals = [float(x) for x in parts[1:]]
            except ValueError:
                return
            self.acc_var.set(f"{vals[0]:+.4f}, {vals[1]:+.4f}, {vals[2]:+.4f} g")
            self.gyr_var.set(f"{vals[3]:+7.2f}, {vals[4]:+7.2f}, {vals[5]:+7.2f} dps")

        elif tag == "ATT" and len(parts) == 4:
            try:
                self._roll_deg = float(parts[1])
                self._pitch_deg = float(parts[2])
                self._yaw_deg = float(parts[3])
            except ValueError:
                return
            if not all(math.isfinite(v) for v in (self._roll_deg, self._pitch_deg, self._yaw_deg)):
                self._last_att = 0
                self._expire_sensor_data(time.time())
                return
            self._last_att = time.time()
            self.roll_var.set(f"{self._roll_deg:+.1f} °")
            self.pitch_var.set(f"{self._pitch_deg:+.1f} °")
            self.yaw_var.set(f"{self._yaw_deg:+.1f} °")
            self._draw_horizon()

        elif tag == "MODE":
            kv = self._kv(parts[1:])
            active = kv.get("active", "--")
            self.mode_var.set(active)
            self.mode_lbl.configure(fg={"MANUAL": self.amber, "ASSIST": self.green,
                                        "TKOFF": "#c084fc", "AUTO": self.blue}
                                    .get(active, "#e0e0e0"))
            req = kv.get("req", "--")
            self.mode_sub_var.set(f"requested {req}   ·   last change {self._last_mode_change}")
            armed = kv.get("armed") == "1"
            self.armed_var.set("BENCH - motors inhibited" if kv.get("flight_enabled") == "0"
                               else "ARMED" if armed else "disarmed")
            fs = kv.get("failsafe", "0")
            self.failsafe_var.set("OK" if fs == "0" else f"ENGAGED (lvl {fs})")
            lock = kv.get("assist_lockout") == "1"
            self.lockout_var.set("LATCHED — IMU fault" if lock else "clear")
            flying = kv.get("flying") == "1"
            self.integ_var.set("active (flying)" if flying else "frozen (on ground)")

        elif tag == "CRSF_STAT":
            kv = self._kv(parts[1:])
            rx = kv.get("receiving") == "1"
            tx = kv.get("telem_tx", "0")
            self.footer_var.set(f"CRSF: link {'up' if rx else 'down'}  ·  "
                                f"telem TX {tx} frames  ·  {self.port}")

        elif tag == "MODE_CHANGE" and len(parts) == 2:
            # one-shot from the firmware on every bumpless switch; the 2 Hz MODE
            # line redraws the sub-label with this value.
            self._last_mode_change = f"{parts[1]} @ {time.strftime('%H:%M:%S')}"

        elif tag == "EST":
            kv = self._kv(parts[1:])
            ready = kv.get("bias_ready") == "1"
            trust = kv.get("acc_trust", "--")
            self.cal_var.set(f"done  (acc_trust {trust})" if ready
                             else f"calibrating…  hold still  (acc_trust {trust})")
            agl = kv.get("agl_m"); climb = kv.get("climb_mps")
            if agl is not None and time.time() - self._last_bmp < 0.3:
                extra = f"   ({float(climb):+.1f} m/s)" if climb is not None else ""
                self.alt_var.set(f"{float(agl):.2f} m relative{extra}")

        elif tag == "CAL_DONE":
            self.cal_var.set("done")

        elif tag == "RC" and len(parts) >= 17:
            try:
                us = [int(x) for x in parts[1:17]]
            except ValueError:
                return
            self.rc_var.set(f"A {us[0]}  E {us[1]}  T {us[2]}  R {us[3]}   "
                            f"arm {us[4]}  mode {us[6]}")

        elif tag == "OUT" and len(parts) == 9:
            try:
                o = [int(x) for x in parts[1:9]]
            except ValueError:
                return
            self.out_var.set(f"ail {o[0]}/{o[1]}  elev {o[2]}/{o[3]}  "
                             f"rud {o[4]}  nose {o[5]}  esc {o[6]}/{o[7]}")

        elif tag == "GPS" and len(parts) == 8:
            try:
                lat = float(parts[1])
                lon = float(parts[2])
                alt = float(parts[3])
                sats = int(parts[4])
                fix = int(parts[5])
                t = parts[6]
                spd = float(parts[7])
            except ValueError:
                return
            if fix <= 0 or not all(math.isfinite(v) for v in (lat, lon, alt, spd)):
                self._last_gps = 0
                self._expire_sensor_data(time.time())
                return
            self._last_gps = time.time()
            self.pos_var.set(f"{lat:.6f}, {lon:.6f}")
            self.gps_alt_var.set(f"{alt:.1f} m")
            self.sats_var.set(str(sats))
            self.fix_var.set(self._fix_label(fix))
            self.gps_time_var.set(t)
            self.gps_speed_var.set(f"{spd:.1f} km/h")

        elif tag == "GPS_STAT" and len(parts) == 5:
            try:
                fix = int(parts[1])
                sats = int(parts[2])
                t = parts[3]
                spd = float(parts[4])
            except ValueError:
                return
            self.sats_var.set(str(sats))
            self.fix_var.set(self._fix_label(fix))
            self.gps_time_var.set(t)
            self.gps_speed_var.set(f"{spd:.1f} km/h")

        elif tag == "IMU_HEALTH" and len(parts) == 6:
            h0, h1, active, ambiguous, valid = parts[1:]
            self.status_var.set(f"{getattr(self, 'imu_model', 'IMU')} {h0}/{h1}  active #{int(active)+1}" +
                                ("  DISAGREE" if ambiguous == "1" else ""))
            self.status_lbl.configure(fg=self.green if valid == "1" else self.red)
            if valid != "1":
                self._last_att = 0
                self._expire_sensor_data(time.time())

        elif tag == "BMI_STATUS" and len(parts) == 2:
            self._connected = (parts[1] == "1")
            if self._connected:
                self.status_var.set("IMU initialization OK; waiting for samples")
                self.status_lbl.configure(fg=self.amber)
            else:
                self.status_var.set("dual IMU: NOT FOUND")
                self.status_lbl.configure(fg=self.red)


def main():
    ap = argparse.ArgumentParser(description="dual IMU + BMP581 + GNSS monitor GUI")
    ap.add_argument("--port", "-p", default=None, help="serial port (default: auto-detect)")
    ap.add_argument("--capture", help="append received diagnostics to this local text file")
    ap.add_argument("--self-test", metavar="REPORT", help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.self_test:
        from gui_selftest import run
        return run(args.self_test)

    root = tk.Tk()
    def report_error(exc_type, exc, tb):
        import traceback
        import tempfile
        log = Path(os.environ.get("LOCALAPPDATA", tempfile.gettempdir())) / "STM32FC-GUI" / "error.log"
        try:
            log.parent.mkdir(parents=True, exist_ok=True)
            with log.open("a", encoding="utf-8") as stream:
                traceback.print_exception(exc_type, exc, tb, file=stream)
            detail = f"\n\nDetails: {log}"
        except OSError:
            detail = ""
        messagebox.showerror("STM32FC GUI error", str(exc) + detail, parent=root)
    root.report_callback_exception = report_error
    try:
        MonitorApp(root, args.port, args.capture)
    except Exception:
        report_error(*sys.exc_info())
        root.destroy()
        return 1
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
