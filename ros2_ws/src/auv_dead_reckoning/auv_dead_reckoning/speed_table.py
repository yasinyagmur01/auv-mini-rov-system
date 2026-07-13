"""Gaz (MANUAL_CONTROL x, -1000..1000) -> hiz (m/s) tablosu.

CSV formati (config/speed_table.csv):
    throttle,speed_mps
    0,0.0
    200,0.18
    ...
Negatif gaz icin simetri varsayilir (geri gidis olcumu varsa satir eklenebilir).
Havuzda `scripts/calibrate_speed.py` ile olculur; yarisma yerinde tekrarlanmali.
"""
import csv


class SpeedTable:
    def __init__(self, csv_path=None):
        # kalibrasyondan once kaba varsayilan (tipik kucuk AUV):
        self.points = [(0, 0.0), (1000, 1.0)]
        if csv_path:
            self.load(csv_path)

    def load(self, csv_path):
        pts = []
        with open(csv_path, newline='') as f:
            lines = [ln for ln in f if ln.strip() and not ln.lstrip().startswith('#')]
        for row in csv.DictReader(lines):
            pts.append((float(row['throttle']), float(row['speed_mps'])))
        if pts:
            pts.sort()
            if pts[0][0] > 0:
                pts.insert(0, (0.0, 0.0))
            self.points = pts

    def speed(self, throttle: float) -> float:
        """Dogrusal interpolasyon; isaret korunur."""
        t = abs(throttle)
        sign = 1.0 if throttle >= 0 else -1.0
        pts = self.points
        if t <= pts[0][0]:
            return sign * pts[0][1]
        for (t0, v0), (t1, v1) in zip(pts, pts[1:]):
            if t <= t1:
                f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                return sign * (v0 + f * (v1 - v0))
        return sign * pts[-1][1]

    def throttle_for(self, speed_mps: float) -> float:
        """Ters arama: istenen hiz icin gaz degeri."""
        s = abs(speed_mps)
        sign = 1.0 if speed_mps >= 0 else -1.0
        pts = self.points
        if s <= pts[0][1]:
            return sign * pts[0][0]
        for (t0, v0), (t1, v1) in zip(pts, pts[1:]):
            if s <= v1:
                f = (s - v0) / (v1 - v0) if v1 > v0 else 0.0
                return sign * (t0 + f * (t1 - t0))
        return sign * pts[-1][0]
