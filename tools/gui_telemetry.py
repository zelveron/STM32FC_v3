"""Validated USB telemetry and freshness rules, independent of the UI toolkit."""
from collections import deque
from dataclasses import dataclass
import math
import re
import time

TTL = {"ATT": .35, "BMI": .35, "BMP": .35, "MAG": .5, "RC": .5, "OUT": .5,
       "MODE": 1.5, "LINK": 1.5, "GPS": 2.5, "GPS_STAT": 2.5, "GPS_HEALTH": 3,
       "SD_DBG": 12, "IMU_CONFIG": 3, "IMU_HEALTH": 3, "BMP_HEALTH": 3,
       "CRSF_STAT": 3, "EST": 3, "TELEM": 3, "YAW_STATUS": 3, "MAG_HEALTH": 3, "MAG_DATA": 3, "MAG_CHECK": 3}


@dataclass(frozen=True)
class Health:
    state: str
    label: str
    detail: str


def imu_error(code):
    return {0: "No driver error", 1: "SPI setup failed", 2: "Power-save setup failed",
            3: "Configuration read failed", 4: "Sensor setup failed", 5: "FIFO setup failed",
            6: "FIFO flush failed", 7: "Identity / health check failed", 8: "FIFO backlog / length fault",
            9: "Sensor clock fault", 10: "FIFO transfer failed", 11: "Invalid paired FIFO frame",
            12: "Clipped sensor sample", 103: "Chip ID mismatch (expected 0x24)",
            109: "Configuration image load failed"}.get(code, "Bosch initialization failed" if code >= 100 else "Unknown driver error")


def bmp_error(code):
    return {0: "No fresh sample", 1: "I²C setup failed", 2: "Reset write failed; communication not established",
            3: "Sensor setup failed", 4: "Invalid output data rate", 5: "Status read failed",
            6: "Sample read failed"}.get(code, "Bosch initialization failed")


def fix_name(code):
    return {0: "No fix", 1: "GPS fix", 2: "DGPS fix", 4: "RTK fixed", 5: "RTK float",
            6: "Dead reckoning"}.get(code, f"Fix type {code}")


class Telemetry:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.records = {}
        self.raw = {}
        self.events = deque(maxlen=60)
        self.track = deque(maxlen=1200)
        self.history = deque(maxlen=600)
        self.received = 0
        self.rejected = 0
        self.last_rx = None
        self.revision = 0

    def get(self, tag, fresh=True):
        entry = self.records.get(tag)
        if entry is None or (fresh and self.clock() - entry[1] > TTL.get(tag, 3)):
            return None
        return entry[0]

    def age(self, tag):
        return self.clock() - self.records[tag][1] if tag in self.records else math.inf

    def event(self, text):
        if not self.events or self.events[-1][1] != text:
            self.events.append((time.strftime("%H:%M:%S"), text))

    @staticmethod
    def _floats(parts, count):
        if len(parts) != count:
            raise ValueError("wrong field count")
        values = tuple(float(x) for x in parts)
        if not all(math.isfinite(x) for x in values):
            raise ValueError("nonfinite sample")
        return values

    def feed(self, line):
        now = self.clock()
        if not line or len(line) > 4096:
            self.rejected += 1
            return False
        parts = line.strip().split(",")
        tag = parts[0]
        if not re.fullmatch(r"[A-Z][A-Z0-9_]*", tag):
            if line.startswith("{"):
                self.event("Incompatible JSON firmware; expected v3 tagged USB telemetry")
            self.rejected += 1
            return False
        kv = dict(x.split("=", 1) for x in parts[1:] if "=" in x)
        try:
            value = kv
            if tag in ("ATT", "BMP", "MAG"):
                value = self._floats(parts[1:], 3)
                if tag == "ATT" and (abs(value[0]) > 360 or abs(value[1]) > 180 or abs(value[2]) > 360):
                    raise ValueError("attitude range")
            elif tag == "BMI":
                value = self._floats(parts[1:], 6)
            elif tag in ("RC", "OUT", "IMU_HEALTH", "TELEM"):
                count = {"RC": 16, "OUT": 8, "IMU_HEALTH": 5, "TELEM": 6}[tag]
                if len(parts) != count + 1:
                    raise ValueError("channel count")
                value = tuple(int(x) for x in parts[1:])
                if tag == "IMU_HEALTH" and any(x not in (0, 1) for x in value):
                    raise ValueError("health bit")
                if tag in ("RC", "OUT") and any(not 0 <= x <= 2500 for x in value):
                    raise ValueError("pulse range")
            elif tag in ("GPS", "GPS_STAT"):
                if len(parts) != (8 if tag == "GPS" else 5):
                    raise ValueError("GPS field count")
                offset = 4 if tag == "GPS" else 2
                sats = int(parts[offset]); fix = int(parts[5] if tag == "GPS" else parts[1])
                speed = float(parts[-1])
                if not 0 <= sats <= 99 or not 0 <= fix <= 8 or not math.isfinite(speed) or speed < 0:
                    raise ValueError("GPS range")
                value = {"sats": sats, "fix": fix, "speed": speed, "utc": parts[-2]}
                if tag == "GPS":
                    lat, lon, alt = self._floats(parts[1:4], 3)
                    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
                        raise ValueError("position range")
                    value.update(lat=lat, lon=lon, alt=alt)
            elif tag == "GPS_HEALTH":
                value = {key: int(kv[key]) for key in ("rx", "nmea", "fix", "used", "visible", "baud")}
                if value["rx"] not in (0, 1) or value["nmea"] not in (0, 1) or value["visible"] not in (-1, 0, 1):
                    raise ValueError("GNSS flags")
                if not -1 <= value["used"] <= 99 or not 0 <= value["fix"] <= 8 or value["baud"] <= 0:
                    raise ValueError("GNSS range")
                value.update({k: int(kv[k]) for k in ("bytes", "messages") if k in kv})
            elif tag == "IMU_CONFIG":
                value = dict(kv, error0=int(kv["error0"]), error1=int(kv["error1"]), mag=int(kv["mag"]))
                if value["mag"] not in (0, 1):
                    raise ValueError("mag flag")
            elif tag == "BMP_HEALTH":
                value = {"valid": int(kv["valid"]), "error": int(kv["error"])}
                if value["valid"] not in (0, 1):
                    raise ValueError("barometer flag")
            elif tag == "MAG_HEALTH":
                value = {k:int(v) for k,v in kv.items()}
                for k in ("enabled", "initialized", "healthy"):
                    if value[k] not in (0,1): raise ValueError("magnetic flag")
                if "communicating" in value and value["communicating"] not in (0,1): raise ValueError("magnetic link")
                if not 0 <= value["stage"] <= 14 or not -127 <= value["result"] <= 127: raise ValueError("magnetic error")
                if "self_test" in value and value["self_test"] not in (0,1): raise ValueError("magnetic self-test")
                for k in ("samples", "bus_errors", "reads", "invalid", "consecutive", "recoveries"):
                    if value.get(k,0) < 0: raise ValueError("magnetic counter")
                for k in ("chip_id", "last_reg", "status", "otp_error"):
                    if not 0 <= value.get(k,0) <= 255: raise ValueError("magnetic register")
                if value.get("bus_status",0) not in (0,-1,-2,-3,-4,-5): raise ValueError("magnetic bus status")
            elif tag == "MAG_CHECK":
                value={k:int(kv[k]) for k in ("err","pmu","aggr","axes","st")}
                if any(not 0<=v<=255 for v in value.values()): raise ValueError("magnetic configuration")
                for k in ("x","y"):
                    v=float(kv[k]); value[k]=v if math.isfinite(v) else None
            elif tag == "MAG_DATA":
                value={k:float(kv[k]) for k in ("x", "y", "z", "temp")}
                value={k:(v if math.isfinite(v) else None) for k,v in value.items()}
                value["raw"]=[int(v) for v in kv["raw"].split("/")]
                if len(value["raw"])!=4 or any(not -8388608<=v<=8388607 for v in value["raw"]): raise ValueError("magnetic raw data")
            elif tag == "YAW_STATUS":
                value = dict(kv)
                if kv["source"] not in ("NONE", "GYRO", "BMM350"):
                    raise ValueError("yaw source")
                if kv["reason"] not in ("disabled", "setup_required", "waiting", "no_attitude",
                        "field_rejected", "innovation_rejected", "qualifying", "aligning",
                        "tracking", "stale", "driver_unavailable"):
                    raise ValueError("yaw state")
                for k in ("valid", "configured", "hold"):
                    value[k] = int(kv[k])
                    if value[k] not in (0, 1): raise ValueError("yaw flag")
                for k in ("heading", "mag_heading", "field", "innovation", "target"):
                    value[k] = float(kv[k])
                    if not math.isfinite(value[k]): raise ValueError("yaw value")
                if any(not 0 <= value[k] <= 360 for k in ("heading", "mag_heading", "target")) or value["field"] < 0 or abs(value["innovation"]) > 180:
                    raise ValueError("heading range")
                if value["valid"] and (value["source"] != "BMM350" or not value["configured"] or value["reason"] != "tracking"):
                    raise ValueError("heading eligibility")
                if value["source"] == "BMM350" and (not value["configured"] or value["reason"] not in ("aligning", "tracking")):
                    raise ValueError("magnetic source without qualified calibration")
                if value["hold"] and not value["valid"]: raise ValueError("hold without heading")
            elif tag == "MODE":
                value = {"active": kv["active"], "req": kv["req"]}
                for k in ("armed", "failsafe", "assist_lockout", "flight_enabled", "flying", "timing_fault"):
                    value[k] = int(kv.get(k, "0"))
                    if value[k] not in (0, 1):
                        raise ValueError("mode flag")
            elif tag == "EST":
                value = dict(kv)
                for k in ("acc_trust", "agl_m", "climb_mps"):
                    value[k] = float(kv[k])
                    if not math.isfinite(value[k]):
                        raise ValueError("estimator field")
                value["bias_ready"] = int(kv["bias_ready"])
                value["baro_ref"] = int(kv.get("baro_ref", "0"))
                if value["bias_ready"] not in (0, 1) or value["baro_ref"] not in (0, 1) or not 0 <= value["acc_trust"] <= 1:
                    raise ValueError("estimator validity")
            elif tag in ("LINK", "CRSF_STAT"):
                value = {k: int(v) for k, v in kv.items()}
                required = ("receiving", "frames_ok", "crc_err", "resync", "telem_tx") if tag == "CRSF_STAT" else ("up_rssi_dbm", "up_lq", "up_snr", "rf_mode")
                if any(k not in value for k in required):
                    raise ValueError("link fields")
                if tag == "CRSF_STAT" and value["receiving"] not in (0, 1):
                    raise ValueError("receiver flag")
                if tag == "LINK" and not 0 <= value["up_lq"] <= 100:
                    raise ValueError("link quality")
            elif tag == "SD_DBG":
                value = dict(kv, active=int(parts[1]), file=parts[2])
                for k in ("bytes", "log_drops", "usb_drops"):
                    value[k] = int(kv[k])
                    if value[k] < 0: raise ValueError("negative counter")
                for k in ("fs", "fatfs", "hw", "sectors"):
                    if k in kv:
                        value[k] = int(kv[k])
                        if value[k] < 0: raise ValueError("negative SD diagnostic")
                if value["active"] not in (0, 1): raise ValueError("SD state")
            elif tag == "MODE_CHANGE":
                value = parts[1]
        except (KeyError, ValueError, IndexError, OverflowError):
            self.rejected += 1
            return False
        previous = self.get(tag, False)
        # Only known display records need parsed storage. Unknown extensions
        # remain available in the bounded raw table without growing memory.
        if tag in TTL or tag == "MODE_CHANGE":
            self.records[tag] = (value, now)
        key = f"SCHED:{parts[1]}" if tag == "SCHED" and len(parts) > 1 else tag
        if key in self.raw or len(self.raw) < 150:
            self.raw[key] = (line, now)
        self.received += 1
        self.last_rx = now
        self.revision += 1
        if tag == "MODE" and value != previous:
            self.event(f"{value['active']} · {'ARMED' if value['armed'] else 'disarmed'} · failsafe {value['failsafe']}")
        if tag == "MODE_CHANGE":
            self.event("Mode changed to " + value)
        if tag == "ATT":
            self.history.append((now, *value))
        if tag == "GPS" and value["fix"] > 0:
            pos = (value["lat"], value["lon"])
            if not self.track or self.track[-1] != pos:
                self.track.append(pos)
        return True

    def attitude(self):
        health = self.get("IMU_HEALTH")
        if health is None or not health[4] or health[3]:
            return None
        return self.get("ATT")

    def position(self):
        pos = self.get("GPS")
        if not pos or not pos["fix"]:
            return None
        for tag in ("GPS_HEALTH", "GPS_STAT"):
            h = self.get(tag)
            if h and (not h["fix"] or (tag == "GPS_HEALTH" and (not h["rx"] or not h["nmea"]))) and self.records[tag][1] >= self.records["GPS"][1]:
                return None
        return pos

    def barometer(self):
        health = self.get("BMP_HEALTH")
        return self.get("BMP") if health and health["valid"] else None

    def receiver_live(self):
        state = self.get("CRSF_STAT")
        return bool(state and state["receiving"] and self.get("RC"))

    def health(self, component, link_open=True):
        if not link_open:
            return Health("off", "Offline", "No controller connection")
        tag = {"usb": "MODE", "imu1": "IMU_HEALTH", "imu2": "IMU_HEALTH", "baro": "BMP_HEALTH",
               "gps": "GPS_HEALTH", "rc": "CRSF_STAT", "sd": "SD_DBG", "mag": "IMU_CONFIG"}[component]
        if component=="mag" and "MAG_HEALTH" in self.records: tag="MAG_HEALTH"
        h = self.get(tag)
        if component == "usb":
            if self.last_rx is not None and self.clock() - self.last_rx < 2:
                return Health("ok", "Streaming", "USB telemetry received")
            return Health("warn", "No data", "Serial port open; waiting for telemetry")
        if h is None:
            return Health("stale" if tag in self.records else "off", "Stale" if tag in self.records else "Waiting", "No recent status received")
        if component.startswith("imu"):
            i = int(component[-1]) - 1
            config = self.get("IMU_CONFIG")
            detail = imu_error(config[f"error{i}"]) if config else "Driver diagnostics unavailable"
            if not h[i]:
                return Health("bad", "Unavailable", detail)
            if h[3]:
                return Health("warn", "Disagree", "Dual-IMU disagreement; estimator ineligible")
            return Health("ok", "Primary" if h[2] == i else "Standby", detail)
        if component == "baro":
            if not h["valid"]:
                return Health("bad", "Unavailable", bmp_error(h["error"]))
            return Health("ok" if self.get("BMP") else "stale", "Healthy" if self.get("BMP") else "Stale", "BMP581 pressure / temperature")
        if component == "gps":
            if not h["rx"]:
                return Health("bad", "No data", "No recent GNSS UART bytes")
            if not h["nmea"]:
                return Health("warn", "No NMEA", "UART bytes received; no valid NMEA")
            return Health("ok" if self.position() else "warn", fix_name(h["fix"]) if h["fix"] else "No fix", f"Valid NMEA at {h['baud']} baud")
        if component == "rc":
            return Health("ok" if self.receiver_live() else "bad", "Linked" if self.receiver_live() else "Link lost", "ER8 / CRSF receiver")
        if component == "sd":
            if h["active"]:
                return Health("ok", "Recording", f"{h['file']} — {h['bytes']:,} bytes written")
            stage=h.get("stage", "")
            reasons={
                "mount_failed": ("Unavailable", "Card initialization or filesystem mount failed"),
                "requires_fat32": ("Needs FAT32", "Card is readable; flight logging requires FAT32"),
                "allocate_failed": ("Allocation failed", "Could not reserve a contiguous 128 MiB log file"),
                "open_failed": ("File error", "Could not create a new flight log"),
                "close_failed": ("File error", "Could not commit the log file allocation"),
                "rng_failed": ("Startup error", "Could not generate a log session identifier"),
                "extent_invalid": ("Storage error", "Allocated log location is outside the card"),
                "dma_busy": ("Storage busy", "SD transfer resources were unavailable at startup"),
                "write_bounds": ("Write stopped", "Requested write was outside the allocated log"),
                "write_start": ("Write failed", "Could not start an SD write; restart required"),
                "write_failed": ("Write failed", "SD write failed or timed out; restart required"),
            }
            if stage in reasons:
                label,detail=reasons[stage]
                if h.get("fatfs"): detail+=f" (filesystem error {h['fatfs']})"
                return Health("bad", label, detail)
            return Health("warn", "Inactive", "No active SD log; card presence is not reported")
        if tag=="MAG_HEALTH":
            if not h["enabled"]: return Health("off", "Disabled", "BMM350 disabled by firmware")
            stage=h["stage"]
            counts=f"Accepted {h.get('samples',0)} / rejected {h.get('invalid',0)} / bus errors {h.get('bus_errors',0)} / recovered {h.get('recoveries',0)}"
            if stage in (8,9):
                error={-2:"I²C timeout",-3:"I²C busy",-5:"I²C NACK"}.get(h.get("bus_status"),"I²C read failed")
                return Health("bad", error, f"{'Status' if stage==8 else 'Sample'} read at 0x{h.get('last_reg',0):02X}; consecutive failures {h.get('consecutive',0)}\n{counts}")
            if stage in (10,11):
                return Health("bad", "Out of range" if stage==11 else "Invalid values", f"{'Live communication' if h.get('communicating') else 'Last received data'}; measurements rejected\n{counts}")
            if stage==13:
                return Health("bad", "Self-test I/O failed" if h["result"] else "Self-test failed", "BMM350 startup X/Y self-test unavailable/failed; heading unavailable\n"+counts)
            if stage==14: return Health("bad", "Configuration failed", "BMM350 startup register readback failed\n"+counts)
            if not h["initialized"] or stage in range(1,8) or stage==12:
                return Health("bad", "Init failed", f"{'OTP calibration acquisition failed' if stage==12 else 'BMM350 startup failed'}; stage {stage}, Bosch result {h['result']}\n{counts}")
            if not h["healthy"] or not self.get("MAG"):
                return Health("stale", "No sample", "No fresh valid BMM350 measurement\n"+counts)
            return Health("ok", "Streaming", "Live valid BMM350 measurements\n"+counts)
        if not h["mag"]:
            return Health("off", "Disabled", "Magnetometer disabled by firmware configuration")
        return Health("ok" if self.get("MAG") else "stale", "Streaming" if self.get("MAG") else "No sample", "BMM350 measurements; see heading source for fusion status")
