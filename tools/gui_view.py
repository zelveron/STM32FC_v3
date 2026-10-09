"""Qt dashboard layout; telemetry and serial ownership live in separate modules."""
from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QColor, QIcon, QPainter, QPixmap, QPolygonF
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QFrame, QLabel, QPushButton,
    QComboBox, QVBoxLayout, QHBoxLayout, QGridLayout, QStackedWidget, QScrollArea,
    QTableWidget, QHeaderView, QLineEdit)
from gui_widgets import Instrument, TrackView, TrendView, ChannelBars, COLORS, TEXT, MUTED, CYAN

STYLE = """
QWidget { background: #0b1018; color: #edf3fa; font-family: 'Segoe UI'; font-size: 13px; }
QFrame#sidebar { background: #0e1520; border-right: 1px solid #223044; }
QFrame#card { background: #121b27; border: 1px solid #253346; border-radius: 12px; }
QFrame#card QLabel, QFrame#card QWidget { background: transparent; }
QLabel { background: transparent; border: none; }
QPushButton { background: #182535; border: 1px solid #2b3e53; border-radius: 7px; padding: 9px 13px; font-weight: 600; }
QPushButton:hover { background: #25394e; border-color: #59768a; }
QPushButton:pressed { background: #304b62; }
QPushButton:disabled { color: #58667a; border-color: #243040; background: #111a26; }
QPushButton#primary { background: #55d6cd; color: #092729; border-color: #55d6cd; }
QPushButton#nav { text-align: left; background: transparent; border: none; border-radius: 7px; padding: 13px 10px; color: #8b9cb1; }
QPushButton#nav:checked { background: #1a3039; color: #69e0d2; }
QPushButton#nav:hover { background: #1a2737; color: #edf3fa; }
QComboBox, QLineEdit { background: #111c2a; border: 1px solid #304157; border-radius: 7px; padding: 8px; selection-background-color: #255a60; }
QComboBox QAbstractItemView { background: #152332; color: #edf3fa; selection-background-color: #255a60; }
QScrollArea { border: none; }
QScrollBar:vertical { background: #0b1018; width: 9px; margin: 0; }
QScrollBar::handle:vertical { background: #334458; min-height: 30px; border-radius: 4px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QTableWidget { background: #121b27; alternate-background-color: #152130; gridline-color: #243245; border: none; selection-background-color: #23434e; }
QHeaderView::section { background: #192536; color: #9bacc2; border: none; padding: 9px; font-weight: 600; }
QToolTip { background: #233348; color: #edf3fa; border: 1px solid #52667d; padding: 8px; }
"""


def label(value="—", size=13, color=TEXT, bold=False):
    item = QLabel(str(value)); item.setTextFormat(Qt.PlainText)
    item.setStyleSheet(f"color:{color}; font-size:{size}px; font-weight:{600 if bold else 400};")
    return item


def card(title, subtitle=None):
    box = QFrame(); box.setObjectName("card")
    layout = QVBoxLayout(box); layout.setContentsMargins(18, 16, 18, 16); layout.setSpacing(10)
    layout.addWidget(label(title, 13, TEXT, True))
    if subtitle:
        sub = label(subtitle, 11, MUTED); sub.setWordWrap(True); layout.addWidget(sub)
    return box, layout


class Metric(QFrame):
    def __init__(self, title, unit, accent=False):
        super().__init__(); self.setObjectName("card")
        layout = QVBoxLayout(self); layout.setContentsMargins(16, 12, 16, 12); layout.setSpacing(4)
        layout.addWidget(label(title.upper(), 10, MUTED, True))
        self.value = label("—", 27, CYAN if accent else TEXT, True); layout.addWidget(self.value)
        self.unit = label(unit, 11, MUTED); layout.addWidget(self.unit)

    def set(self, value, unit=None):
        self.value.setText(str(value))
        if unit is not None: self.unit.setText(unit)


class StatusTile(QFrame):
    def __init__(self, title):
        super().__init__(); self.setObjectName("card")
        layout = QVBoxLayout(self); layout.setContentsMargins(12, 11, 12, 11); layout.setSpacing(5)
        layout.addWidget(label(title, 11, MUTED, True)); self.state = label("●  Offline", 12); layout.addWidget(self.state)

    def set(self, status):
        self.state.setText("●  " + status.label)
        self.state.setStyleSheet(f"color:{COLORS[status.state]}; font-size:12px; font-weight:600;")
        self.setToolTip(status.detail)


class DashboardView(QMainWindow):
    def setup_view(self, port):
        self.setWindowTitle("STM32FC · Ground Station")
        available = QApplication.primaryScreen().availableGeometry()
        self.resize(min(1440, available.width()-40), min(910, available.height()-50))
        self.setMinimumSize(1080, 700); self.setStyleSheet(STYLE); self.setWindowIcon(self.make_icon())
        root=QWidget(); self.setCentralWidget(root)
        shell=QHBoxLayout(root); shell.setContentsMargins(0,0,0,0); shell.setSpacing(0)
        sidebar=QFrame(); sidebar.setObjectName("sidebar"); sidebar.setFixedWidth(176)
        nav=QVBoxLayout(sidebar); nav.setContentsMargins(16,27,16,20); nav.setSpacing(8)
        nav.addWidget(label("✦  STM32FC",20,CYAN,True)); nav.addWidget(label("GROUND STATION",10,MUTED,True)); nav.addSpacing(31)
        self.nav_buttons=[]
        for i,title in enumerate(("Flight deck", "Receiver", "Sensors", "Diagnostics", "System")):
            button=QPushButton(f"0{i+1}   {title}"); button.setObjectName("nav"); button.setCheckable(True)
            button.clicked.connect(lambda checked=False,n=i:self.select_page(n)); nav.addWidget(button); self.nav_buttons.append(button)
        nav.addStretch(); nav.addWidget(label("FIXED-WING",10,MUTED,True)); nav.addWidget(label("STM32F407 · v2.2",11,MUTED))
        self.source_label=label("USB TELEMETRY",10,CYAN,True); nav.addWidget(self.source_label); shell.addWidget(sidebar)
        content=QWidget(); outer=QVBoxLayout(content); outer.setContentsMargins(24,21,24,12); outer.setSpacing(13)
        heading=QHBoxLayout(); titles=QVBoxLayout(); titles.setSpacing(3)
        self.page_title=label("Flight deck",26,TEXT,True); titles.addWidget(self.page_title)
        self.page_description=label("Aircraft attitude, navigation and component health",12,MUTED); titles.addWidget(self.page_description)
        heading.addLayout(titles); heading.addStretch()
        self.connection_badge=label("●  DISCONNECTED",11,MUTED,True); heading.addWidget(self.connection_badge); outer.addLayout(heading)
        controls=QHBoxLayout(); controls.setSpacing(8)
        self.port_combo=QComboBox(); self.port_combo.setMinimumWidth(175); controls.addWidget(self.port_combo)
        self.refresh_button=QPushButton("↻"); self.refresh_button.setToolTip("Refresh serial ports"); self.refresh_button.clicked.connect(self.refresh_ports); controls.addWidget(self.refresh_button)
        self.connect_button=QPushButton("Reconnect"); self.connect_button.setObjectName("primary"); self.connect_button.clicked.connect(lambda:self.request_transition("connect")); controls.addWidget(self.connect_button)
        self.disconnect_button=QPushButton("Disconnect"); self.disconnect_button.clicked.connect(lambda:self.request_transition("disconnect")); controls.addWidget(self.disconnect_button)
        controls.addStretch()
        self.record_button=QPushButton("Stop capture" if self.session.capture_path else "Record USB"); self.record_button.clicked.connect(self.toggle_record); controls.addWidget(self.record_button)
        self.preview_button=QPushButton("Preview"); self.preview_button.clicked.connect(lambda:self.request_transition("connect" if self.demo else "demo")); controls.addWidget(self.preview_button)
        self.refresh_ports(port); outer.addLayout(controls)
        self.banner=label("",12,"#efbd69",True); self.banner.setWordWrap(True); self.banner.setMinimumHeight(24); outer.addWidget(self.banner)
        self.stack=QStackedWidget(); outer.addWidget(self.stack,1); self.pages=[]
        for _ in range(5):
            scroll=QScrollArea(); scroll.setWidgetResizable(True)
            page=QWidget(); lay=QVBoxLayout(page); lay.setContentsMargins(0,0,8,4); lay.setSpacing(14)
            scroll.setWidget(page); self.stack.addWidget(scroll); self.pages.append(lay)
        self.build_flight(); self.build_receiver(); self.build_sensors(); self.build_diagnostics(); self.build_system()
        footer=QHBoxLayout(); self.footer=label("Waiting for a controller",11,MUTED); footer.addWidget(self.footer,1)
        self.packet_count=label("0 messages",10,MUTED); footer.addWidget(self.packet_count); outer.addLayout(footer)
        shell.addWidget(content,1); self.select_page(0)

    @staticmethod
    def make_icon():
        pix=QPixmap(64,64); pix.fill(QColor("#0e202b"))
        p=QPainter(pix); p.setRenderHint(QPainter.Antialiasing); p.setPen(Qt.NoPen); p.setBrush(QColor(CYAN))
        p.drawPolygon(QPolygonF([QPointF(32,7),QPointF(38,29),QPointF(57,43),QPointF(37,37),QPointF(36,49),QPointF(44,55),QPointF(32,51),QPointF(20,55),QPointF(28,49),QPointF(27,37),QPointF(7,43),QPointF(26,29)])); p.end()
        return QIcon(pix)

    def select_page(self, n):
        names=("Flight deck","Receiver & outputs","Sensors","Diagnostics","System")
        subtitles=("Aircraft attitude, navigation and component health", "TX16S / ER8 pilot input and commanded PWM outputs",
                   "Sensor health, calibration and measured signals", "Every USB field, with last-update age", "USB session and ground maintenance")
        self.stack.setCurrentIndex(n); self.page_title.setText(names[n]); self.page_description.setText(subtitles[n])
        for i,b in enumerate(self.nav_buttons): b.setChecked(i==n)

    def build_flight(self):
        page=self.pages[0]; row=QHBoxLayout(); row.setSpacing(12); self.metrics={}
        for key,title,unit in (("mode","Flight mode","Waiting for status"),("speed","Ground speed","km/h · GPS"),("altitude","Relative altitude","m · barometer"),("climb","Vertical speed","m/s · barometer"),("sats","Satellites used","GNSS fix")):
            metric=Metric(title,unit,key=="mode"); self.metrics[key]=metric; row.addWidget(metric,1)
        page.addLayout(row); instruments=QHBoxLayout(); instruments.setSpacing(12)
        box,layout=card("ATTITUDE", "Artificial horizon · roll / pitch")
        self.horizon=Instrument(); layout.addWidget(self.horizon,1); self.att_labels={}; angles=QHBoxLayout()
        for key,title in (("roll","ROLL"),("pitch","PITCH")):
            col=QVBoxLayout(); col.addWidget(label(title,10,MUTED,True)); v=label("—",22,TEXT,True); col.addWidget(v); angles.addLayout(col); self.att_labels[key]=v
        layout.addLayout(angles); instruments.addWidget(box,4)
        box,layout=card("AIRCRAFT ORIENTATION", "Fixed observer · body attitude")
        self.aircraft=Instrument("aircraft"); layout.addWidget(self.aircraft,1)
        self.yaw_title=label("HEADING / YAW",10,MUTED,True); layout.addWidget(self.yaw_title)
        self.att_labels["yaw"]=label("—",22,TEXT,True); layout.addWidget(self.att_labels["yaw"])
        self.yaw_source=label("Source unavailable",11,MUTED); self.yaw_source.setWordWrap(True); layout.addWidget(self.yaw_source)
        self.heading_hold_status=label("Heading hold unavailable",11,MUTED); self.heading_hold_status.setWordWrap(True); layout.addWidget(self.heading_hold_status)
        instruments.addWidget(box,4)
        box,layout=card("POSITION & GROUND TRACK", "Offline view · north up")
        self.track=TrackView(); layout.addWidget(self.track,1)
        self.gps_position=label("No valid position",15,TEXT,True); self.gps_position.setWordWrap(True); layout.addWidget(self.gps_position)
        self.gps_summary=label("Waiting for GNSS telemetry",11,MUTED); self.gps_summary.setWordWrap(True); layout.addWidget(self.gps_summary)
        self.gps_more=label("GPS altitude —   ·   UTC —",11,MUTED); self.gps_more.setWordWrap(True); layout.addWidget(self.gps_more)
        instruments.addWidget(box,4); page.addLayout(instruments,1)
        page.addWidget(label("COMPONENT HEALTH",11,MUTED,True)); row=QHBoxLayout(); row.setSpacing(8); self.health_tiles={}
        for key,name in (("usb","USB / FC"),("imu1","IMU 1"),("imu2","IMU 2"),("baro","BMP581"),("gps","GNSS"),("rc","ER8 / RC"),("sd","SD LOG"),("mag","BMM350")):
            tile=StatusTile(name); self.health_tiles[key]=tile; row.addWidget(tile,1)
        page.addLayout(row); box,layout=card("FLIGHT STATE")
        self.flight_state=label("No current flight-controller status",12,MUTED); self.flight_state.setWordWrap(True); layout.addWidget(self.flight_state); page.addWidget(box)

    def build_receiver(self):
        page=self.pages[1]; row=QHBoxLayout(); self.link_metrics={}
        for key,title,unit in (("lq","Link quality","% uplink"),("rssi","Signal strength","dBm uplink"),("snr","Signal / noise","dB uplink"),("frames","Valid RC frames","controller total")):
            m=Metric(title,unit); self.link_metrics[key]=m; row.addWidget(m)
        page.addLayout(row); panels=QHBoxLayout()
        box,layout=card("RECEIVER CHANNELS", "AETR · CH5 arm · CH7 mode · PWM equivalent in µs")
        names=["01  Aileron / roll","02  Elevator / pitch","03  Throttle","04  Rudder / yaw","05  Arm switch","06  Spare","07  Mode switch","08  Spare"]+[f"{i:02}  Unassigned" for i in range(9,17)]
        self.rc_bars=ChannelBars(names); layout.addWidget(self.rc_bars); panels.addWidget(box,1)
        right=QVBoxLayout(); box,layout=card("COMMANDED OUTPUTS", "Sent PWM commands; physical servo motion is not measured")
        self.out_bars=ChannelBars(["01  Aileron (SERVO1)","02  Elevator (SERVO2)","03  Throttle (SERVO3)","04  Rudder (SERVO4)","05  Spare / center (SERVO5)","06  Aileron reversed (SERVO6)","07  Reserved / idle (SERVO8)","08  Reserved / idle (SERVO9)"]); layout.addWidget(self.out_bars); right.addWidget(box)
        box,layout=card("RADIO DIAGNOSTICS")
        self.radio_detail=label("No receiver statistics",13,MUTED); self.radio_detail.setWordWrap(True); layout.addWidget(self.radio_detail)
        note=label("RF mode is the raw ELRS enumeration. Packet-rate names vary by radio firmware.",11,MUTED); note.setWordWrap(True); layout.addWidget(note); right.addWidget(box); right.addStretch()
        panels.addLayout(right,1); page.addLayout(panels); page.addStretch()

    def build_sensors(self):
        page=self.pages[2]; grid=QGridLayout(); grid.setSpacing(12); self.sensor_details={}
        for index,(key,title) in enumerate((("imu1","PRIMARY / BACKUP · BMI270 #1"),("imu2","PRIMARY / BACKUP · BMI270 #2"),("baro","PRESSURE · BMP581"),("gps","NAVIGATION · SAM-M10Q"))):
            box,layout=card(title); detail=label("Waiting for telemetry",13); detail.setWordWrap(True); layout.addWidget(detail)
            self.sensor_details[key]=detail; grid.addWidget(box,index//2,index%2)
        page.addLayout(grid); row=QHBoxLayout()
        box,layout=card("SELECTED IMU SIGNALS", "Filtered / bias-corrected; separate per-IMU raw signals are not in USB telemetry")
        self.imu_signals=label("Acceleration —\nAngular rate —",14); self.imu_signals.setWordWrap(True); layout.addWidget(self.imu_signals)
        self.calibration=label("Calibration unavailable",12,MUTED); self.calibration.setWordWrap(True); layout.addWidget(self.calibration); row.addWidget(box,1)
        box,layout=card("MAGNETOMETER & LOGGING")
        self.storage_details=label("Waiting for telemetry",12); self.storage_details.setWordWrap(True); layout.addWidget(self.storage_details); row.addWidget(box,1); page.addLayout(row)
        box,layout=card("COMPASS SETUP", "Installed-aircraft calibration is separate from factory compensation")
        self.heading_details=label("Heading source unavailable",12); self.heading_details.setWordWrap(True); layout.addWidget(self.heading_details)
        note=label("Record USB while slowly rotating through all orientations. Analyze the capture with tools/calibrate_mag.py to derive offsets, soft-iron correction and sensor-axis mapping. Heading hold remains unavailable until measured calibration is installed. See docs/MAGNETIC_HEADING.md.",12,MUTED)
        note.setWordWrap(True); layout.addWidget(note); page.addWidget(box)
        box,layout=card("ATTITUDE HISTORY", "Cyan: roll · Lavender: pitch · Gaps indicate missing data")
        self.trend=TrendView(); layout.addWidget(self.trend); page.addWidget(box); page.addStretch()

    def build_diagnostics(self):
        page=self.pages[3]; self.filter=QLineEdit(); self.filter.setPlaceholderText("Filter by message, task or field…")
        self.filter.textChanged.connect(lambda:self.update_table(True)); page.addWidget(self.filter)
        self.table=QTableWidget(0,4); self.table.setHorizontalHeaderLabels(["MESSAGE / TASK","LATEST FIELDS","AGE","STATE"])
        self.table.setAlternatingRowColors(True); self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows); self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(1,QHeaderView.Stretch)
        self.table.setColumnWidth(0,170); self.table.setColumnWidth(2,75); self.table.setColumnWidth(3,95)
        self.table.setMinimumHeight(420); page.addWidget(self.table,1)
        note=label("Raw values remain visible after disconnect. Age and state indicate freshness; the table does not validate physical hardware.",11,MUTED); note.setWordWrap(True); page.addWidget(note)
        export=QPushButton("Export diagnostic snapshot…"); export.clicked.connect(self.export_snapshot); page.addWidget(export,0,Qt.AlignLeft)

    def build_system(self):
        page=self.pages[4]; box,layout=card("SESSION", "Local USB monitoring · no cloud service")
        self.session_detail=label("Disconnected",13); self.session_detail.setWordWrap(True); layout.addWidget(self.session_detail)
        note=label("Record USB saves incoming tagged CSV. A diagnostic snapshot saves the latest values and their ages.",12,MUTED); note.setWordWrap(True); layout.addWidget(note); page.addWidget(box)
        box,layout=card("FIRMWARE MAINTENANCE", "Ground use only")
        note=label("ROM DFU stops the flight application, stabilization and PWM. Land, disconnect propulsion, disarm CH5 and lower throttle before requesting DFU. The GUI verifies the ROM device; it does not flash an image.",13,MUTED); note.setWordWrap(True); layout.addWidget(note)
        self.dfu_button=QPushButton("Enter ROM DFU…"); self.dfu_button.clicked.connect(self.enter_dfu); layout.addWidget(self.dfu_button,0,Qt.AlignLeft)
        self.dfu_status=label("No DFU request made",12,MUTED); self.dfu_status.setWordWrap(True); layout.addWidget(self.dfu_status); page.addWidget(box)
        box,layout=card("SESSION EVENTS"); self.event_log=label("Waiting for telemetry",12,MUTED); self.event_log.setWordWrap(True); self.event_log.setTextInteractionFlags(Qt.TextSelectableByMouse); layout.addWidget(self.event_log); page.addWidget(box); page.addStretch()
