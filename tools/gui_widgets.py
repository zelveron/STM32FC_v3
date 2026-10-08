"""Vector flight instruments; CPU rendered, no browser or OpenGL dependency."""
import math
from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF, QPainterPath, QLinearGradient
from PySide6.QtWidgets import QWidget

BG = "#0b1018"
PANEL = "#121b27"
EDGE = "#263447"
TEXT = "#edf3fa"
MUTED = "#8b9cb1"
CYAN = "#55d6cd"
COLORS = {"ok": "#5dddac", "warn": "#efbd69", "bad": "#ff7887", "off": "#718198", "stale": "#efbd69"}


def font(size=10, bold=False, mono=False):
    f = QFont("Consolas" if mono else "Segoe UI", size)
    f.setBold(bold)
    return f


def text(p, rect, value, color=TEXT, size=10, bold=False, align=Qt.AlignCenter, mono=False):
    p.setPen(QColor(color)); p.setFont(font(size, bold, mono))
    p.drawText(QRectF(*rect), align, str(value))


def rotate_body(point, roll, pitch, yaw):
    """Body FRD -> earth NED; positive roll right-wing-down, pitch nose-up."""
    x, y, z = point
    r, t, h = map(math.radians, (roll, pitch, yaw))
    y, z = y*math.cos(r)-z*math.sin(r), y*math.sin(r)+z*math.cos(r)
    x, z = x*math.cos(t)+z*math.sin(t), -x*math.sin(t)+z*math.cos(t)
    return (x*math.cos(h)-y*math.sin(h), x*math.sin(h)+y*math.cos(h), z)


class Instrument(QWidget):
    def __init__(self, kind="horizon"):
        super().__init__()
        self.kind = kind
        self.attitude = None
        self.setMinimumSize(240, 245)
        self.setAccessibleName("Artificial horizon" if kind == "horizon" else "Three-dimensional aircraft attitude")

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor(PANEL))
        if self.attitude is None:
            # Empty instruments must not depict a healthy level aircraft.
            p.setPen(QPen(QColor(EDGE), 1))
            for x in range(20, w, 28): p.drawLine(x, 20, x, h-20)
            for y in range(20, h, 28): p.drawLine(20, y, w-20, y)
            text(p, (0, h/2-27, w, 30), "ATTITUDE UNAVAILABLE", MUTED, 11, True)
            text(p, (0, h/2+2, w, 25), "Waiting for fresh, valid IMU data", MUTED, 9)
        elif self.kind == "horizon":
            self.horizon(p, w, h)
        else:
            self.aircraft(p, w, h)
        p.end()

    def horizon(self, p, w, h):
        roll, pitch, yaw = self.attitude
        cx, cy = w/2, h/2
        radius = min(w/2-18, h/2-14)
        clip = QPainterPath(); clip.addRoundedRect(QRectF(12, 8, w-24, h-16), 12, 12)
        p.save(); p.setClipPath(clip)
        p.translate(cx, cy); p.rotate(-roll)
        pixels = radius/35
        p.translate(0, pitch*pixels)
        sky = QLinearGradient(0, -radius, 0, 0)
        sky.setColorAt(0, QColor("#183e63")); sky.setColorAt(1, QColor("#2d7090"))
        p.fillRect(QRectF(-w*3, -h*5, w*6, h*5), sky)
        p.fillRect(QRectF(-w*3, 0, w*6, h*5), QColor("#574437"))
        p.setPen(QPen(QColor("#d9eff3"), 2)); p.drawLine(QPointF(-w*3, 0), QPointF(w*3, 0))
        for deg in range(-80, 81, 10):
            if not deg or abs(deg-pitch)>22: continue
            y = -deg*pixels; length = 44 if deg % 20 == 0 else 26
            p.setPen(QPen(QColor("#e8f5fa"), 1.2))
            p.drawLine(QPointF(-length, y), QPointF(length, y))
            text(p, (-length-31, y-9, 24, 18), abs(deg), "#e8f5fa", 8, mono=True)
            text(p, (length+7, y-9, 24, 18), abs(deg), "#e8f5fa", 8, mono=True)
        p.restore()
        p.save(); p.translate(cx, cy)
        p.setPen(QPen(QColor(TEXT), 2))
        for a in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            angle = math.radians(a-90)
            rr = radius-8
            length = 13 if a % 30 == 0 else 7
            p.drawLine(QPointF(math.cos(angle)*rr, math.sin(angle)*rr),
                       QPointF(math.cos(angle)*(rr-length), math.sin(angle)*(rr-length)))
        p.setPen(Qt.NoPen); p.setBrush(QColor(CYAN))
        p.save(); p.rotate(roll)
        p.drawPolygon(QPolygonF([QPointF(0, -radius+20), QPointF(-5, -radius+29), QPointF(5, -radius+29)]))
        p.restore()
        p.setPen(QPen(QColor("#f4d38b"), 3))
        p.drawPolyline(QPolygonF([QPointF(-63, 0), QPointF(-22, 0), QPointF(-10, 10), QPointF(0, 0),
                                  QPointF(10, 10), QPointF(22, 0), QPointF(63, 0)]))
        p.restore()
        text(p, (12, h-40, w-24, 24), "ROLL / PITCH · EARTH REFERENCE", "#e9e5df", 8, True)

    def aircraft(self, p, w, h):
        # Isometric observer is fixed in the relative-yaw reference frame.
        scale = min(w/8.7, h/7.3)
        def project(point, rotated=False):
            x, y, z = rotate_body(point, *self.attitude) if rotated else point
            return QPointF(w/2 + (.72*x+.69*y)*scale, h*.49 + (-.38*x+.40*y+.83*z)*scale)
        p.setPen(QPen(QColor("#223144"), 1))
        for n in range(-5, 6):
            p.drawLine(project((n, -5, 1.2)), project((n, 5, 1.2)))
            p.drawLine(project((-5, n, 1.2)), project((5, n, 1.2)))
        # Flat-shaded mesh: tapered fuselage, main wings, tailplane, canopy, fin.
        faces = [([(2.8, 0, 0), (.7, -.25, .16), (-2.1, -.12, 0), (-2.1, .12, 0), (.7, .25, .16)], "#9fbbc9"),
                 ([(2.8, 0, 0), (.7, .25, .16), (.4, .19, -.27), (1.7, .10, -.17)], "#d7e9f0"),
                 ([(2.8, 0, 0), (1.7, -.10, -.17), (.4, -.19, -.27), (.7, -.25, .16)], "#ecf5f8"),
                 ([(-2.1, -.12, 0), (.4, -.19, -.27), (.4, .19, -.27), (-2.1, .12, 0)], "#b3cdd7"),
                 ([(1.7, -.10, -.17), (.4, -.19, -.27), (.4, .19, -.27), (1.7, .10, -.17)], "#397b8d"),
                 ([(-2, 0, -.06), (-1.7, 0, -1.0), (-1.1, 0, -.16)], "#60d1c9")]
        for sign in (-1, 1):
            faces += [([(.8, sign*.17, 0), (-.05, sign*3.3, .05), (-.65, sign*3.3, .05), (-.35, sign*.17, 0)], "#d9e8ec" if sign < 0 else "#a4bec9"),
                      ([(-1.4, sign*.1, 0), (-1.55, sign*1.15, .02), (-2.03, sign*1.15, .02), (-2.05, sign*.1, 0)], "#bcd3dc"),
                      ([(-.05, sign*3.0, .048), (-.05, sign*3.3, .048), (-.65, sign*3.3, .048), (-.60, sign*3.0, .048)], CYAN)]
        def depth(face):
            pts = [rotate_body(pt, *self.attitude) for pt in face[0]]
            return sum(-.57*x+.60*y-.56*z for x,y,z in pts)/len(pts)
        for points, color in sorted(faces, key=depth):
            p.setBrush(QColor(color)); p.setPen(QPen(QColor("#668590"), .8))
            p.drawPolygon(QPolygonF([project(pt, True) for pt in points]))
        nose = project((3.2, 0, 0), True)
        p.setPen(QPen(QColor(CYAN), 1)); p.drawEllipse(nose, 3, 3)
        text(p, (8, h-34, w-16, 22), "YAW IS RELATIVE · NOT COMPASS HEADING", MUTED, 8, True)


class TrackView(QWidget):
    def __init__(self):
        super().__init__(); self.points = []; self.live = False
        self.setMinimumHeight(150)
        self.setAccessibleName("Offline GPS ground track")

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor("#101d2a"))
        p.setPen(QPen(QColor("#213448"), 1))
        for x in range(0, w, 28): p.drawLine(x, 0, x, h)
        for y in range(0, h, 28): p.drawLine(0, y, w, y)
        text(p, (w-34, 4, 26, 24), "N ↑", CYAN, 9, True)
        if self.points:
            lat0, lon0 = self.points[0]
            pts = [((((lon-lon0+180)%360)-180)*111320*math.cos(math.radians(lat0)), (lat-lat0)*111320) for lat,lon in self.points]
            xs, ys = zip(*pts); xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
            scale = min((w-50)/max(100, xmax-xmin), (h-65)/max(100, ymax-ymin))
            screen = [QPointF(w/2+(x-(xmin+xmax)/2)*scale, h/2-(y-(ymin+ymax)/2)*scale) for x,y in pts]
            p.setPen(QPen(QColor(CYAN if self.live else MUTED), 2)); p.drawPolyline(QPolygonF(screen))
            p.setBrush(QColor(CYAN if self.live else MUTED)); p.drawEllipse(screen[-1], 4, 4)
            text(p, (10, h-26, w-20, 20), f"{1/scale*50:.0f} m / 50 px  ·  {'LIVE FIX' if self.live else 'LAST TRACK / STALE'}", MUTED, 8, align=Qt.AlignLeft)
        else:
            text(p, (5, h/2-12, w-10, 24), "Waiting for a valid GPS position", MUTED, 9)
        p.end()


class TrendView(QWidget):
    def __init__(self):
        super().__init__(); self.samples = []; self.now = 0
        self.setMinimumHeight(160)

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor(PANEL))
        p.setPen(QPen(QColor(EDGE), 1))
        for y in (h*.25, h*.5, h*.75): p.drawLine(QPointF(10, y), QPointF(w-10, y))
        for axis, color in ((1, CYAN), (2, "#af99f5")):
            path = QPainterPath(); last = None
            for entry in self.samples:
                age = self.now-entry[0]
                if age > 30: continue
                pt = QPointF(12+(w-24)*(1-age/30), h/2-entry[axis]/90*(h/2-22))
                if last is None or entry[0]-last > .5: path.moveTo(pt)
                else: path.lineTo(pt)
                last = entry[0]
            p.setPen(QPen(QColor(color), 1.7)); p.drawPath(path)
        text(p, (12, h-25, w-24, 20), "ROLL  /  PITCH     ±90°     LAST 30 SECONDS", MUTED, 8, align=Qt.AlignLeft)
        p.end()


class ChannelBars(QWidget):
    def __init__(self, labels):
        super().__init__(); self.labels = labels; self.values = None
        self.setMinimumHeight(len(labels)*31+14)

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        for i, name in enumerate(self.labels):
            y = 6+i*31
            text(p, (0, y, 156, 25), name, MUTED, 9, align=Qt.AlignLeft)
            left, width = 161, max(20, w-232)
            p.setPen(Qt.NoPen); p.setBrush(QColor("#233145"))
            p.drawRoundedRect(QRectF(left, y+8, width, 8), 4, 4)
            if self.values is not None:
                pulse = self.values[i]
                frac = max(0, min(1, (pulse-1000)/1000))
                p.setBrush(QColor(CYAN)); p.drawRoundedRect(QRectF(left, y+8, width*frac, 8), 4, 4)
                text(p, (w-63, y, 63, 25), pulse, TEXT, 10, mono=True)
            else:
                text(p, (w-63, y, 63, 25), "—", MUTED, 10)
            p.setPen(QPen(QColor("#72879b"), 1)); p.drawLine(QPointF(left+width/2, y+5), QPointF(left+width/2, y+19))
        p.end()
