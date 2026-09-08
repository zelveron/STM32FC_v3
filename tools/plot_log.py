#!/usr/bin/env python3
"""
plot_log.py -- interactive plotter for STM32FC FLTxxxxx.BIN flight logs.

    python3 tools/plot_log.py [FLT00007.BIN]

Open a .BIN (File > Open, or pass it on the command line). It is decoded to a
CSV next to it (real units + timestamps), and every channel appears in the tree
on the left, grouped. Select one or more (Ctrl / Shift click), then:

  Plot     -- draw the selection on fresh axes
  Overlay  -- add the selection on top of what is already plotted
  Clear    -- empty the axes
  Grid     -- stack the selection (or everything) in separate panels, shared X

X axis is time in seconds from the start of the log; Y axis is the channel
value. "normalize" rescales each series to 0..1 so channels with very different
ranges (1500 us vs 10 deg) can share one axis. The toolbar under the plot does
zoom / pan / save-png.

Needs: python3-matplotlib, python3-numpy, python3-tk  (apt install ...).
"""
import os
import sys
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from parse_bin_log import decode, write_csv, FIELDS   # noqa: E402

GROUPS = [
    ("IMU accel (g)",  ["ax_g", "ay_g", "az_g"]),
    ("IMU gyro (dps)", ["gx_dps", "gy_dps", "gz_dps"]),
    ("Attitude (deg)", ["roll_deg", "pitch_deg", "yaw_deg"]),
    ("Baro",           ["press_pa", "temp_c", "alt_m"]),
    ("GPS",            ["lat", "lon", "gps_alt_m", "gps_speed_ms", "gps_sats", "gps_fix"]),
    ("RC in (us)",     [f"rc{i}" for i in range(1, 9)]),
    ("Servo out (us)", [f"out{i}" for i in range(1, 9)]),
    ("State",          ["mode", "armed", "failsafe", "req_mode"]),
]
DISCRETE = {"mode", "armed", "failsafe", "req_mode", "gps_fix", "gps_sats"}


class PlotApp:
    def __init__(self, root, path=None):
        self.root = root
        root.title("STM32FC log plotter")
        root.geometry("1180x700")

        self.t = []          # time axis, seconds from start
        self.data = {}        # channel name -> list of values
        self.rows = []        # raw decoded rows (for Save CSV as)
        self.csv_path = None
        self.series = []      # channel names currently drawn on the main axes

        top = ttk.Frame(root)
        top.pack(fill="x", padx=6, pady=4)
        ttk.Button(top, text="Open .BIN…", command=self.open_file).pack(side="left")
        ttk.Button(top, text="Save CSV as…", command=self.save_csv).pack(side="left", padx=(4, 0))
        self.status = tk.StringVar(value="no file loaded")
        ttk.Label(top, textvariable=self.status).pack(side="left", padx=12)

        body = ttk.Frame(root)
        body.pack(fill="both", expand=True, padx=6, pady=4)

        left = ttk.Frame(body)
        left.pack(side="left", fill="y")
        ttk.Label(left, text="channels  (Ctrl / Shift click for multiple)").pack(anchor="w")

        tw = ttk.Frame(left)
        tw.pack(fill="y", expand=True)
        self.tree = ttk.Treeview(tw, show="tree", selectmode="extended", height=26)
        sb = ttk.Scrollbar(tw, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="y", expand=True)
        sb.pack(side="left", fill="y")
        self.tree.bind("<Double-1>", lambda e: self.plot(replace=True))

        self.norm = tk.BooleanVar(value=False)
        ttk.Checkbutton(left, text="normalize each series (0–1)",
                        variable=self.norm).pack(anchor="w", pady=(6, 2))

        row1 = ttk.Frame(left); row1.pack(anchor="w")
        ttk.Button(row1, text="Plot", width=8,
                   command=lambda: self.plot(replace=True)).pack(side="left")
        ttk.Button(row1, text="Overlay", width=8,
                   command=lambda: self.plot(replace=False)).pack(side="left", padx=3)
        row2 = ttk.Frame(left); row2.pack(anchor="w", pady=2)
        ttk.Button(row2, text="Clear", width=8, command=self.clear).pack(side="left")
        ttk.Button(row2, text="Select all", width=8,
                   command=self.select_all).pack(side="left", padx=3)
        ttk.Button(left, text="Grid (stacked panels)",
                   command=self.plot_grid).pack(anchor="w", pady=(4, 0))

        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True)
        self.fig = Figure(figsize=(8, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        NavigationToolbar2Tk(self.canvas, right)
        self._reset_axes()

        self._build_tree()
        if path:
            self.load(path)

    # ---- tree ----
    def _build_tree(self):
        for gi, (gname, chans) in enumerate(GROUPS):
            gid = self.tree.insert("", "end", iid=f"grp{gi}", text=gname, open=True)
            for c in chans:
                self.tree.insert(gid, "end", iid=c, text=c)

    def _selected_channels(self):
        return [i for i in self.tree.selection() if i in self.data]

    def select_all(self):
        self.tree.selection_set([c for _, cs in GROUPS for c in cs if c in self.data])

    # ---- axes ----
    def _reset_axes(self):
        self.ax.clear()
        self.ax.set_xlabel("time (s)")
        self.ax.set_ylabel("value")
        self.ax.grid(True, alpha=0.3)
        self.series = []
        self.canvas.draw_idle()

    def clear(self):
        self._reset_axes()

    def _series_xy(self, name):
        y = self.data[name]
        label = name
        if self.norm.get():
            lo, hi = min(y), max(y)
            rng = (hi - lo) or 1.0
            y = [(v - lo) / rng for v in y]
            label = f"{name} (norm)"
        return y, label

    def plot(self, replace):
        chans = self._selected_channels()
        if not chans:
            if self.data:
                messagebox.showinfo("nothing selected", "Select one or more channels first.")
            return
        if replace:
            self._reset_axes()
        for c in chans:
            if c in self.series:
                continue
            y, label = self._series_xy(c)
            style = dict(drawstyle="steps-post") if c in DISCRETE else {}
            self.ax.plot(self.t, y, label=label, linewidth=1.0, **style)
            self.series.append(c)
        self.ax.set_ylabel("normalized (0–1)" if self.norm.get() else "value")
        leg = self.ax.get_legend()
        if len(self.series) > 1:
            self.ax.legend(loc="best", fontsize=8)
        elif leg:
            leg.remove()
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

    def plot_grid(self):
        chans = self._selected_channels() or \
            [c for _, cs in GROUPS for c in cs if c in self.data]
        if not chans:
            return
        n = len(chans)
        win = tk.Toplevel(self.root)
        win.title(f"grid — {n} channel(s)")
        win.geometry("900x700")

        fig = Figure(figsize=(8.5, max(2.2, 1.5 * n)), dpi=100)
        axes = fig.subplots(n, 1, sharex=True, squeeze=False)[:, 0]
        for ax, c in zip(axes, chans):
            style = dict(drawstyle="steps-post") if c in DISCRETE else {}
            ax.plot(self.t, self.data[c], linewidth=0.9, **style)
            ax.set_ylabel(c, fontsize=8, rotation=0, ha="right", va="center")
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=7)
        axes[-1].set_xlabel("time (s)")
        fig.tight_layout(h_pad=0.35)

        # scrollable embed so a tall stack stays usable
        container = ttk.Frame(win)
        container.pack(fill="both", expand=True)
        scroll = tk.Canvas(container, highlightthickness=0)
        vbar = ttk.Scrollbar(container, orient="vertical", command=scroll.yview)
        scroll.configure(yscrollcommand=vbar.set)
        vbar.pack(side="right", fill="y")
        scroll.pack(side="left", fill="both", expand=True)
        holder = ttk.Frame(scroll)
        scroll.create_window((0, 0), window=holder, anchor="nw")

        mcanvas = FigureCanvasTkAgg(fig, master=holder)
        widget = mcanvas.get_tk_widget()
        widget.configure(width=840, height=int(max(2.2, 1.5 * n) * 100))
        widget.pack()
        NavigationToolbar2Tk(mcanvas, win)
        mcanvas.draw()
        holder.update_idletasks()
        scroll.configure(scrollregion=scroll.bbox("all"))

        def _wheel(evt):
            step = -1 if getattr(evt, "num", None) == 4 or evt.delta > 0 else 1
            scroll.yview_scroll(step, "units")
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            scroll.bind(seq, _wheel)
            holder.bind(seq, _wheel)

    # ---- file ----
    def open_file(self):
        p = filedialog.askopenfilename(
            title="Open flight log",
            filetypes=[("STM32FC binary log", ("*.BIN", "*.bin")), ("all files", "*")])
        if p:
            self.load(p)

    def load(self, path):
        try:
            meta, rows = decode(path)
        except Exception as e:                       # noqa: BLE001
            messagebox.showerror("decode failed", str(e))
            return
        if not rows:
            messagebox.showwarning("empty log", "no valid frames in this file")
            return

        self.rows = rows
        cols = list(zip(*rows))                      # column-major
        tms = cols[0]
        self.t = [(v - tms[0]) / 1000.0 for v in tms]
        self.data = {name: list(cols[k]) for k, name in enumerate(FIELDS) if name != "t_ms"}

        self.csv_path = os.path.splitext(path)[0] + ".csv"
        try:
            write_csv(rows, self.csv_path)
            csv_note = f"CSV → {os.path.basename(self.csv_path)}"
        except OSError:
            csv_note = "(could not write CSV next to the .BIN)"

        self.status.set(
            f"{os.path.basename(path)}   •   {meta['n_ok']} frames   •   "
            f"{meta['duration_s']:.1f} s   •   {meta['hz']:.0f} Hz   •   "
            f"{meta['n_bad_crc']} bad-crc, {meta['n_resync']} resync   •   {csv_note}")
        self._reset_axes()

    def save_csv(self):
        if not self.rows:
            return
        p = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=os.path.basename(self.csv_path or "flight.csv"),
            filetypes=[("CSV", "*.csv"), ("all files", "*")])
        if p:
            write_csv(self.rows, p)
            messagebox.showinfo("saved", f"wrote {p}")


def main():
    path = None
    for a in sys.argv[1:]:
        if a.lower().endswith(".bin"):
            path = a
    root = tk.Tk()
    PlotApp(root, path)
    root.mainloop()


if __name__ == "__main__":
    main()
