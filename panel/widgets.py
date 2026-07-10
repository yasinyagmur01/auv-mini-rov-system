"""Ozel cizim widget'lari: dikey profil ("net Y ekseni") ve DR haritasi."""
import math
from collections import deque

from PySide6.QtCore import Qt, QPointF, QRectF
from PySide6.QtGui import QPainter, QPen, QBrush, QColor, QPolygonF, QFont
from PySide6.QtWidgets import QWidget

EARTH_R = 6371000.0


def latlon_to_local(origin_lat, origin_lon, lat, lon):
    dlat = math.radians(lat - origin_lat)
    dlon = math.radians(lon - origin_lon)
    x_east = dlon * math.cos(math.radians(origin_lat)) * EARTH_R
    y_north = dlat * EARTH_R
    return x_east, y_north


class VerticalProfileWidget(QWidget):
    """Yuzey cizgisi - arac - deniz tabani kesiti."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(180, 260)
        self.depth = 0.0
        self.altitude = 0.0
        self.depth_valid = False
        self.altitude_valid = False

    def set_data(self, depth, altitude, depth_valid, altitude_valid):
        self.depth = depth or 0.0
        self.altitude = altitude or 0.0
        self.depth_valid = depth_valid
        self.altitude_valid = altitude_valid
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor('#0b2740'))

        margin = 24
        usable = h - 2 * margin
        column = self.depth + (self.altitude if self.altitude_valid else 2.0)
        scale_max = max(3.0, column * 1.25)

        def y_of(depth_m):
            return margin + usable * (depth_m / scale_max)

        # yuzey
        p.setPen(QPen(QColor('#7fd4ff'), 2))
        p.drawLine(0, int(y_of(0)), w, int(y_of(0)))
        p.setPen(QColor('#7fd4ff'))
        p.drawText(6, int(y_of(0)) - 6, 'yuzey 0 m')

        # taban
        if self.altitude_valid and self.depth_valid:
            yb = int(y_of(column))
            p.setPen(QPen(QColor('#c9a24b'), 2))
            p.drawLine(0, yb, w, yb)
            for x in range(0, w, 14):  # taramali zemin
                p.drawLine(x, yb, x + 8, yb + 8)
            p.drawText(6, min(h - 6, yb + 20), f'taban {column:.1f} m')

        # arac
        if self.depth_valid:
            yv = y_of(self.depth)
            body = QPolygonF([QPointF(w / 2 - 16, yv), QPointF(w / 2 + 16, yv),
                              QPointF(w / 2 + 22, yv + 7), QPointF(w / 2 - 22, yv + 7)])
            p.setBrush(QBrush(QColor('#ffd23f')))
            p.setPen(Qt.NoPen)
            p.drawPolygon(body)
            p.setPen(QColor('white'))
            p.drawText(int(w / 2 + 28), int(yv + 6), f'{self.depth:.2f} m')
            if self.altitude_valid:
                p.setPen(QPen(QColor('#66ff99'), 1, Qt.DashLine))
                p.drawLine(int(w / 2), int(yv + 8), int(w / 2), int(y_of(column)))
                p.setPen(QColor('#66ff99'))
                p.drawText(int(w / 2) + 4, int((yv + y_of(column)) / 2),
                           f'{self.altitude:.2f} m')
        else:
            p.setPen(QColor('#888'))
            p.drawText(QRectF(0, 0, w, h), Qt.AlignCenter, 'derinlik verisi yok')


class DRMapWidget(QWidget):
    """Yerel ENU duzleminde iz + hedefler (cevrimdisi mini harita)."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(260, 260)
        self.path = deque(maxlen=2000)   # (x_east, y_north)
        self.pos = None
        self.heading = 0.0
        self.origin = None               # (lat, lon)
        self.targets = {}                # ad -> (x_east, y_north)
        self.orbit_radius = 12.0

    def set_state(self, dr: dict):
        if not dr or not dr.get('origin_valid'):
            return
        self.origin = (dr.get('est_lat'), dr.get('est_lon'))
        x, y = dr.get('x_east_m', 0.0), dr.get('y_north_m', 0.0)
        self.pos = (x, y)
        if not self.path or (abs(self.path[-1][0] - x) + abs(self.path[-1][1] - y)) > 0.2:
            self.path.append((x, y))
        self.update()

    def set_heading(self, hdg):
        self.heading = hdg or 0.0

    def set_targets_latlon(self, origin_lat, origin_lon, targets: dict):
        """targets: ad -> (lat, lon); origin DR baslangic fix'i."""
        self.targets = {
            name: latlon_to_local(origin_lat, origin_lon, lat, lon)
            for name, (lat, lon) in targets.items()
        }
        self.update()

    def clear(self):
        self.path.clear()
        self.pos = None
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor('#10141a'))

        pts = list(self.path) + list(self.targets.values())
        if self.pos:
            pts.append(self.pos)
        if not pts:
            p.setPen(QColor('#666'))
            p.drawText(QRectF(0, 0, w, h), Qt.AlignCenter,
                       'DR verisi yok\n(origin fix bekleniyor)')
            return

        xs = [q[0] for q in pts] + [0.0]
        ys = [q[1] for q in pts] + [0.0]
        span = max(max(xs) - min(xs), max(ys) - min(ys), 20.0) * 1.2
        cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
        scale = min(w, h) / span

        def to_px(x, y):
            return QPointF(w / 2 + (x - cx) * scale, h / 2 - (y - cy) * scale)

        # izgara (10 m)
        p.setPen(QPen(QColor('#233040'), 1))
        step = 10.0 * scale
        if step > 8:
            import itertools
            x0 = (w / 2 - cx * scale) % step
            y0 = (h / 2 + cy * scale) % step
            for gx in itertools.count():
                px = x0 + gx * step
                if px > w:
                    break
                p.drawLine(int(px), 0, int(px), h)
            for gy in itertools.count():
                py = y0 + gy * step
                if py > h:
                    break
                p.drawLine(0, int(py), w, int(py))

        # baslangic (origin)
        p.setPen(QPen(QColor('#7fd4ff'), 2))
        o = to_px(0, 0)
        p.drawEllipse(o, 5, 5)
        p.drawText(o + QPointF(8, -4), 'start')

        # hedefler
        for name, (tx, ty) in self.targets.items():
            q = to_px(tx, ty)
            if name == 'turn':
                p.setPen(QPen(QColor('#ff9955'), 2, Qt.DashLine))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(q, self.orbit_radius * scale, self.orbit_radius * scale)
                p.setPen(QPen(QColor('#ff9955'), 2))
                p.drawEllipse(q, 4, 4)
            else:
                p.setPen(QPen(QColor('#66ff99'), 2))
                s = max(8.0, 5.0 * scale)
                p.drawRect(QRectF(q.x() - s / 2, q.y() - s / 2, s, s))
            p.drawText(q + QPointF(8, 4), name)

        # iz
        if len(self.path) > 1:
            p.setPen(QPen(QColor('#ffd23f'), 2))
            last = None
            for x, y in self.path:
                q = to_px(x, y)
                if last is not None:
                    p.drawLine(last, q)
                last = q

        # arac (pruva oku)
        if self.pos:
            q = to_px(*self.pos)
            a = math.radians(self.heading)
            tip = QPointF(q.x() + 12 * math.sin(a), q.y() - 12 * math.cos(a))
            l = QPointF(q.x() + 6 * math.sin(a + 2.5), q.y() - 6 * math.cos(a + 2.5))
            r = QPointF(q.x() + 6 * math.sin(a - 2.5), q.y() - 6 * math.cos(a - 2.5))
            p.setBrush(QBrush(QColor('#ff5555')))
            p.setPen(Qt.NoPen)
            p.drawPolygon(QPolygonF([tip, l, r]))

        # olcek etiketi
        p.setPen(QColor('#888'))
        p.setFont(QFont('sans', 8))
        p.drawText(8, h - 8, f'izgara: 10 m  |  pruva: {self.heading:.0f} deg')
