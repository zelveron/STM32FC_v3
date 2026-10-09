"""Map validated current telemetry into dashboard widgets."""
import time
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QTableWidgetItem
from gui_telemetry import fix_name, TTL
from gui_widgets import TEXT, MUTED, CYAN


class DashboardPresenter:
    def render(self):
        m=self.model; now=m.clock(); online=self.link_open or self.demo
        live=lambda tag: m.get(tag) if online else None
        attitude=m.attitude() if online else None; gps=m.position() if online else None
        mode=live("MODE"); est=live("EST"); bmp=m.barometer() if online else None; gpsh=live("GPS_HEALTH")
        self.preview_button.setText("Exit preview" if self.demo else "Preview")
        self.record_button.setEnabled(not self.demo and not self.pending and not (self.dfu_thread and self.dfu_thread.is_alive()))
        self.source_label.setText("SIMULATED PREVIEW" if self.demo else "USB TELEMETRY")
        self.connection_badge.setText("●  PREVIEW / SIMULATED" if self.demo else "●  ROM DFU" if self.dfu_mode else "●  USB CONNECTED" if self.link_open else "●  DISCONNECTED")
        self.connection_badge.setStyleSheet(f"color:{CYAN if online else MUTED};font-size:11px;font-weight:600;")
        warnings=[]
        if self.demo: warnings.append("PREVIEW — SIMULATED DATA · No serial connection or hardware commands")
        elif self.dfu_mode: warnings.append("ROM DFU ACTIVE · Flight application and PWM stopped")
        elif not online: warnings.append("OFFLINE · Connect your controller, or use Preview to explore the dashboard")
        elif mode is None: warnings.append("FLIGHT STATUS UNAVAILABLE · Waiting for fresh mode / arming telemetry")
        if mode:
            if not mode["flight_enabled"]: warnings.append("BENCH · Motors inhibited")
            if mode["failsafe"]: warnings.append("RC FAILSAFE ENGAGED")
            if mode["assist_lockout"]: warnings.append("ASSIST LOCKOUT")
            if mode["timing_fault"]: warnings.append("CONTROL TIMING FAULT")
        self.banner.setText("    •    ".join(warnings) if warnings else "LIVE TELEMETRY · Verify aircraft setup before flight")
        self.metrics["mode"].set(mode["active"] if mode else "—",("Requested "+mode["req"]) if mode else "No current status")
        self.metrics["speed"].set(f"{gps['speed']:.1f}" if gps else "—")
        self.metrics["altitude"].set(f"{est['agl_m']:.1f}" if est and bmp and est["baro_ref"] else "—")
        self.metrics["climb"].set(f"{est['climb_mps']:+.1f}" if est and bmp and est["baro_ref"] else "—")
        sats=gpsh["used"] if gpsh and gpsh["nmea"] else gps["sats"] if gps else -1
        self.metrics["sats"].set(sats if sats>=0 else "—",fix_name(gpsh["fix"]) if gpsh and gpsh["nmea"] else fix_name(gps["fix"]) if gps else "No current fix status")
        for widget in (self.horizon,self.aircraft): widget.attitude=attitude; widget.update()
        for i,key in enumerate(("roll","pitch","yaw")): self.att_labels[key].setText(f"{attitude[i]:+.1f}°" if attitude else "—")
        yaw=live("YAW_STATUS") if attitude else None
        reason={"disabled":"Magnetometer disabled", "setup_required":"Compass setup required",
                "waiting":"Waiting for magnetic samples", "no_attitude":"Attitude unavailable",
                "field_rejected":"Magnetic field rejected", "innovation_rejected":"Heading innovation rejected",
                "qualifying":"Checking magnetic samples", "aligning":"Aligning magnetic heading",
                "tracking":"Magnetic north reference", "stale":"Magnetic data stale",
                "driver_unavailable":"BMM350 data unavailable"}
        if yaw and yaw["source"] != "NONE":
            magnetic=yaw["source"]=="BMM350"
            self.yaw_title.setText("MAGNETIC HEADING" if magnetic else "GYRO YAW / COASTING")
            self.att_labels["yaw"].setText(f"{attitude[2]%360:.1f}°")
            self.yaw_source.setText(("Source: BMM350 + gyro" if magnetic else "Source: gyro fallback")+"\n"+reason[yaw["reason"]])
            holding=bool(yaw["hold"] and mode and mode["active"]=="ASSIST" and mode["armed"] and mode["flying"] and not mode["failsafe"])
            self.heading_hold_status.setText(f"Holding {yaw['target']:.1f}° magnetic" if holding else "Heading hold: ready / pilot control" if yaw["valid"] else "Heading hold unavailable")
            self.heading_details.setText(f"{reason[yaw['reason']]}\nCalibration: {'installed' if yaw['configured'] else 'required'}\nField {yaw['field']:.2f} µT · innovation {yaw['innovation']:+.1f}°\nMagnetic heading is not true north or GPS ground track.")
        else:
            self.yaw_title.setText("HEADING / YAW")
            self.yaw_source.setText("Source unavailable / stale")
            self.heading_hold_status.setText("Heading hold unavailable")
            self.heading_details.setText("Heading source unavailable / stale. A healthy BMM350 alone does not prove heading fusion is active.")
        self.track.points=list(m.track); self.track.live=bool(gps); self.track.update()
        self.gps_position.setText(f"{gps['lat']:.6f}°\n{gps['lon']:.6f}°" if gps else "No valid position")
        gh=m.health("gps",online); visibility={-1:"Visibility unknown",0:"No satellites reported in view",1:"Satellites in view"}
        self.gps_summary.setText(gh.detail+"\n"+(visibility[gpsh["visible"]] if gpsh else gh.label))
        self.gps_more.setText(f"GPS altitude {gps['alt']:.1f} m MSL · UTC {gps['utc']}" if gps else "GPS altitude — · UTC —")
        for key,tile in self.health_tiles.items(): tile.set(m.health(key,online))
        if mode:
            self.flight_state.setText(f"{'ARMED' if mode['armed'] else 'DISARMED'} / {'Motor authorization enabled' if mode['flight_enabled'] else 'Motors inhibited'} / {'Failsafe engaged' if mode['failsafe'] else 'Failsafe clear'} / {'Assist locked' if mode['assist_lockout'] else 'Assist lockout clear'} / {'Integrators active' if mode['flying'] else 'Integrators frozen'}")
        else: self.flight_state.setText("Mode, arming, failsafe and motor authorization unavailable / stale")
        crsf=live("CRSF_STAT"); link=live("LINK") if m.receiver_live() and online else None
        for key,field in (("lq","up_lq"),("rssi","up_rssi_dbm"),("snr","up_snr")): self.link_metrics[key].set(link[field] if link else "—")
        self.link_metrics["frames"].set(f"{crsf['frames_ok']:,}" if crsf else "—")
        self.rc_bars.values=live("RC") if m.receiver_live() and online else None; self.rc_bars.update()
        self.out_bars.values=live("OUT"); self.out_bars.update()
        self.radio_detail.setText(f"Receiver: {'receiving' if crsf['receiving'] else 'link lost'}\nCRC errors: {crsf['crc_err']}    Resyncs: {crsf['resync']}\nTelemetry queued: {crsf['telem_tx']} frames\nRF mode: {link['rf_mode'] if link else '—'}\nRF statistics are receiver-reported; no handset acknowledgement." if crsf else "Receiver statistics unavailable / stale")
        cfg=live("IMU_CONFIG"); health=live("IMU_HEALTH")
        for i,key in enumerate(("imu1","imu2")):
            h=m.health(key,online)
            self.sensor_details[key].setText(f"{h.label} · {h.detail}\n"+(f"Error {cfg['error'+str(i)]} / registers {cfg.get('regs'+str(i),'—')}\n" if cfg else "")+(f"Selected IMU: #{health[2]+1} · estimator {'eligible' if health[4] else 'ineligible'}" if health else "Health status unavailable"))
        bh=m.health("baro",online)
        self.sensor_details["baro"].setText(bh.label+" · "+bh.detail+(f"\n{bmp[0]:.3f} hPa / {bmp[1]:.2f} °C\nPressure altitude {bmp[2]:.2f} m" if bmp else "\nPressure / temperature unavailable"))
        self.sensor_details["gps"].setText(gh.label+" · "+gh.detail+(f"\nSatellites used: {gpsh['used'] if gpsh['used']>=0 else 'unknown'} · {visibility[gpsh['visible']]}\nUART bytes {gpsh.get('bytes','—')} / valid messages {gpsh.get('messages','—')}" if gpsh else ""))
        bmi=live("BMI") if attitude else None
        self.imu_signals.setText("Acceleration  "+" / ".join(f"{v:+.4f}" for v in bmi[:3])+" g\nAngular rate  "+" / ".join(f"{v:+.2f}" for v in bmi[3:])+" °/s" if bmi else "Acceleration —\nAngular rate —")
        self.calibration.setText(f"Gyro calibration: {'ready' if est['bias_ready'] else 'hold still — calibrating'}\nGravity trust: {est['acc_trust']:.2f} / gyro bias {est.get('gbias','—')} °/s" if est else "Calibration status unavailable / stale")
        sd=live("SD_DBG"); mag=live("MAG")
        self.storage_details.setText("Magnetometer: "+m.health("mag",online).label+("\n"+" / ".join(f"{v:+.2f}" for v in mag)+" µT" if mag else "")+(f"\nSD: {'logging '+sd['file'] if sd['active'] else 'inactive'}\nWritten {sd['bytes']:,} bytes / log drops {sd['log_drops']} / USB drops {sd['usb_drops']}" if sd else "\nSD status unavailable"))
        self.trend.samples=list(m.history); self.trend.now=now; self.trend.update()
        self.dfu_button.setEnabled(self.link_open and not self.demo and not self.pending and not (self.dfu_thread and self.dfu_thread.is_alive()))
        self.packet_count.setText(f"{m.received:,} messages · {m.rejected} rejected · {self.session.dropped} UI drops")
        age=f"{now-m.last_rx:.1f}s" if m.last_rx is not None else "—"
        if self.demo: self.footer.setText("SIMULATED PREVIEW · Illustrative position and values")
        elif self.link_open: self.footer.setText(f"{self.session.current_port} · 115200 baud · Last telemetry {age} ago")
        self.session_detail.setText(f"Source: {'simulated preview' if self.demo else self.session.current_port or 'disconnected'}\nLast message age: {age}\nCapture: {self.session.capture_path or 'not recording'}\nGUI receive drops: {self.session.dropped} / rejected telemetry: {m.rejected}")
        self.event_log.setText("\n".join(f"{t}   {message}" for t,message in list(m.events)[-10:]) or "Waiting for telemetry")
        self.update_table()

    def update_table(self, force=False):
        now=time.monotonic()
        if not force and now-self._last_table<.5: return
        self._last_table=now; query=self.filter.text().lower()
        rows=[(key,line,stamp) for key,(line,stamp) in self.model.raw.items() if query in (key+line).lower()]
        rows.sort(); self.table.setRowCount(len(rows))
        for i,(key,line,stamp) in enumerate(rows):
            age=self.model.clock()-stamp; fresh=(self.demo or self.link_open) and age<=TTL.get(key.split(':')[0],3)
            for j,value in enumerate((key,line.partition(',')[2],f"{age:.1f}s","Recent" if fresh else "Stale")):
                item=self.table.item(i,j)
                if item is None: item=QTableWidgetItem(); self.table.setItem(i,j,item)
                item.setText(value); item.setToolTip(line); item.setForeground(QColor(TEXT if fresh else MUTED))
