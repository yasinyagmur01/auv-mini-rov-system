#!/usr/bin/env python3
"""Kamera (RGB) ozelligi.

Abonelikler:
  /zed/zed_node/rgb/color/rect/image    sensor_msgs/Image  (bgra8 -> RGB)
  /zed/zed_node/left/color/rect/image   sensor_msgs/Image  (sol goz)
  /zed/zed_node/right/color/rect/image  sensor_msgs/Image  (sag goz)

Canli/kayit kanallari:
  rgb (6 Hz kare), left/right (2.5 Hz kare), img_stats (2 Hz skaler; 'fps'
  alani kameradan GELEN gercek kare hizidir)

Analitik (pandas/numpy):
  parlaklik, kontrast (std), netlik (gradyan varyansi) ve pozlama tasma orani
  profilleri — su alti fener/aydinlatma testlerinde kullanilir.
"""

import os
import sys
import time
from collections import deque

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'common'))
from feature_base import FeatureBase, Throttle, sensor_qos, run_feature  # noqa: E402
from ros_np import image_to_np  # noqa: E402

from sensor_msgs.msg import Image  # noqa: E402


class RgbFeature(FeatureBase):
    FEATURE_ID = 'rgb'
    TITLE = 'Kamera (RGB)'

    def __init__(self, session_dir):
        super().__init__(session_dir)
        self._frame_thr = Throttle(6.0)
        self._stats_thr = Throttle(2.0)
        self._eye_thr = {'left': Throttle(2.5), 'right': Throttle(2.5)}
        self._arrivals = deque(maxlen=300)   # gercek gelis FPS olcumu

        self.create_subscription(
            Image, '/zed/zed_node/rgb/color/rect/image',
            self._on_image, sensor_qos())
        self.create_subscription(
            Image, '/zed/zed_node/left/color/rect/image',
            lambda m: self._on_eye('left', m), sensor_qos())
        self.create_subscription(
            Image, '/zed/zed_node/right/color/rect/image',
            lambda m: self._on_eye('right', m), sensor_qos())

    # ---------------- callbacks ----------------

    @staticmethod
    def _shrink(img):
        small = img[::2, ::2]
        while small.shape[1] > 640:
            small = small[::2, ::2]
        return small

    def _on_eye(self, side, msg: Image):
        if not self._eye_thr[side].ok():
            return
        img = image_to_np(msg)
        if img.ndim != 3:
            return
        self.emit_frame(side, np.ascontiguousarray(self._shrink(img)),
                        quality=70)

    def _measured_fps(self):
        now = time.monotonic()
        self._arrivals.append(now)
        recent = [x for x in self._arrivals if now - x <= 3.0]
        return round(len(recent) / 3.0, 1)

    def _on_image(self, msg: Image):
        fps = self._measured_fps()
        want_frame = self._frame_thr.ok()
        want_stats = self._stats_thr.ok()
        if not (want_frame or want_stats):
            return
        img = image_to_np(msg)          # bgra8 -> RGB uint8 (H,W,3)
        if img.ndim != 3:
            return                       # beklenmedik encoding (or. 32FC1)
        small = self._shrink(img)        # 1280x720 -> 640x360

        if want_frame:
            self.emit_frame('rgb', np.ascontiguousarray(small), quality=75)

        if want_stats:
            gray = small.mean(axis=2)    # 3 kanal ortalamasi (float, 0-255)
            brightness = float(gray.mean())
            contrast = float(gray.std())
            gx, gy = np.gradient(gray.astype(np.float32))
            sharpness = float((gx ** 2 + gy ** 2).mean())
            clip_ratio = float(((gray < 10) | (gray > 245)).mean())
            self.emit_scalar('img_stats', {
                'brightness': round(brightness, 2),
                'contrast': round(contrast, 2),
                'sharpness': round(sharpness, 3),
                'clip_ratio': round(clip_ratio, 4),
                'fps': fps,
            })
            self.record_series('fps', fps)
            self.record_series('brightness', brightness)
            self.record_series('contrast', contrast)
            self.record_series('sharpness', sharpness)
            self.record_series('clip_ratio', clip_ratio)

    # ---------------- analitik ----------------

    def compute_analytics(self):
        import pandas as pd  # noqa: F401 (series_df kullanir)

        def to_arr(name):
            vals = self.series.get(name, [])
            if not vals:
                return np.empty(0), np.empty(0)
            a = np.array(vals)
            return a[:, 0], a[:, 1]

        summary = {'kare_sayisi': int(self._counts.get('rgb', 0))}
        charts = []

        specs = [
            ('fps', 'Kare Hizi (kameradan gelen)', 'FPS', 'ort_fps', 1),
            ('brightness', 'Parlaklik', '0-255', 'ort_parlaklik', 2),
            ('contrast', 'Kontrast (std)', '0-255', 'ort_kontrast', 2),
            ('sharpness', 'Netlik (gradyan varyansi)', 'a.u.',
             'ort_netlik', 3),
            ('clip_ratio', 'Pozlama Tasmasi Orani', '0-1',
             'pozlama_tasma_orani', 4),
        ]
        for name, title, unit, skey, ndig in specs:
            t, v = to_arr(name)
            if len(t) == 0:
                continue
            summary[skey] = round(float(v.mean()), ndig)
            charts.append({'id': name, 'title': title, 'unit': unit,
                           'type': 'line',
                           'series': [{'name': name,
                                       'points': self.downsample(
                                           list(zip(t, v)))}]})
        return {'summary': summary, 'charts': charts}


if __name__ == '__main__':
    run_feature(RgbFeature)
