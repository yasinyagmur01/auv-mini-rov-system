#!/usr/bin/env python3
"""Derinlik Algilama ozelligi.

Abonelikler:
  /zed/zed_node/depth/depth_registered     sensor_msgs/Image (32FC1, metre;
                                           NaN/inf pikseller gecersiz)
  /zed/zed_node/confidence/confidence_map  sensor_msgs/Image (32FC1, 0-100;
                                           0=guvenilir, ~100=GUVENILMEZ)
  /zed/zed_node/depth/depth_info           zed_msgs/DepthInfoStamped
                                           (min_depth, max_depth)

Canli/kayit kanallari:
  depth (frame, 3 Hz)      : BLUE_RAMP ile renklendirilmis derinlik
                             (acik=yakin, koyu=uzak; gecersiz=koyu gri)
  confidence (frame, 2 Hz) : AQUA_RAMP ile guvenilirlik (acik=guvenilir)
  depth_stats (4 Hz)       : gecerli oran, min/ort/maks/p50 derinlik
  depth_hist (0.5 Hz)      : 0.3-8.0 m arasi 16 kutulu histogram
  depth_info (2 Hz)        : ZED'in bildirdigi min/max derinlik

Analitik (pandas/numpy):
  gecerli piksel orani, ortalama derinlik, gozlenen min/maks derinlik,
  guvenilirlik profili, son derinlik histogrami.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'common'))
from feature_base import FeatureBase, Throttle, sensor_qos, run_feature  # noqa: E402
from ros_np import image_to_np, make_lut, BLUE_RAMP, AQUA_RAMP  # noqa: E402

from sensor_msgs.msg import Image  # noqa: E402
from zed_msgs.msg import DepthInfoStamped  # noqa: E402

DEPTH_NEAR = 0.3   # m — renklendirme/histogram alt siniri
DEPTH_FAR = 8.0    # m — renklendirme/histogram ust siniri
INVALID_RGB = np.array([26, 26, 25], dtype=np.uint8)  # gecersiz piksel rengi
HIST_BINS = 16


class DepthFeature(FeatureBase):
    FEATURE_ID = 'depth'
    TITLE = 'Derinlik Algilama'

    def __init__(self, session_dir):
        super().__init__(session_dir)
        self._depth_thr = Throttle(3.0)
        self._conf_thr = Throttle(2.0)
        self._stats_thr = Throttle(4.0)
        self._hist_thr = Throttle(0.5)
        self._info_thr = Throttle(2.0)

        self._blue_lut = make_lut(BLUE_RAMP)   # index 0=acik(yakin) 255=koyu(uzak)
        self._aqua_lut = make_lut(AQUA_RAMP)   # index 0=acik(guvenilir) 255=koyu

        self._last_hist = None   # analitik icin son histogram
        self._depth_min_seen = None
        self._depth_max_seen = None

        self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered',
            self._on_depth, sensor_qos())
        self.create_subscription(
            Image, '/zed/zed_node/confidence/confidence_map',
            self._on_conf, sensor_qos())
        self.create_subscription(
            DepthInfoStamped, '/zed/zed_node/depth/depth_info',
            self._on_info, sensor_qos())

    # ---------------- yardimcilar ----------------

    @staticmethod
    def _shrink(arr):
        """numpy dilimlemeyle genisligi <=640 piksele indir (1280 -> 640)."""
        while arr.shape[1] > 640:
            arr = arr[::2, ::2]
        return arr

    # ---------------- callbacks ----------------

    def _on_depth(self, msg: Image):
        do_frame = self._depth_thr.ok()
        do_stats = self._stats_thr.ok()
        do_hist = self._hist_thr.ok()
        if not (do_frame or do_stats or do_hist):
            return
        d = image_to_np(msg)          # float32 (H,W), metre
        if d.dtype == np.uint16:      # openni modu (mm) -> metre
            d = d.astype(np.float32) / 1000.0
        elif d.dtype != np.float32:   # mono8 vb. yorumlanamaz kare -> atla
            return
        valid = np.isfinite(d) & (d > 0)

        if do_frame:
            small = self._shrink(d)
            vs = self._shrink(valid)
            dd = np.where(vs, small, DEPTH_NEAR)
            norm = np.clip((dd - DEPTH_NEAR) / (DEPTH_FAR - DEPTH_NEAR), 0.0, 1.0)
            idx = (norm * 255).astype(np.uint8)   # yakin=0(acik) uzak=255(koyu)
            rgb = self._blue_lut[idx]
            rgb[~vs] = INVALID_RGB
            self.emit_frame('depth', rgb)

        if do_stats or do_hist:
            dv = d[valid]
            valid_ratio = float(valid.mean())

            if do_stats:
                data = {'valid_ratio': round(valid_ratio, 3)}
                if dv.size:
                    dmin = float(dv.min())
                    dmax = float(dv.max())
                    dmean = float(dv.mean())
                    data.update({'min': round(dmin, 3),
                                 'mean': round(dmean, 3),
                                 'max': round(dmax, 3),
                                 'p50': round(float(np.median(dv)), 3)})
                    if self._depth_min_seen is None or dmin < self._depth_min_seen:
                        self._depth_min_seen = dmin
                    if self._depth_max_seen is None or dmax > self._depth_max_seen:
                        self._depth_max_seen = dmax
                    self.record_series('depth_mean', dmean)
                self.emit_scalar('depth_stats', data)
                self.record_series('valid_ratio', valid_ratio)

            if do_hist and dv.size:
                counts, edges = np.histogram(
                    dv, bins=HIST_BINS, range=(DEPTH_NEAR, DEPTH_FAR))
                bins = [round(float(e), 2) for e in edges[:-1]]  # sol kenarlar
                centers = [round(float((edges[i] + edges[i + 1]) / 2), 3)
                           for i in range(HIST_BINS)]
                hist = {'bins': bins, 'counts': [int(c) for c in counts]}
                self._last_hist = {'centers': centers,
                                   'counts': [int(c) for c in counts]}
                self.emit_scalar('depth_hist', hist)

    def _on_conf(self, msg: Image):
        if not self._conf_thr.ok():
            return
        conf = image_to_np(msg)       # float32 (H,W), 0=guvenilir 100=guvenilmez
        fin = np.isfinite(conf)
        small = self._shrink(conf)
        vs = self._shrink(fin)
        cc = np.where(vs, small, 100.0)
        idx = (np.clip(cc / 100.0, 0.0, 1.0) * 255).astype(np.uint8)
        rgb = self._aqua_lut[idx]     # 0 -> acik (guvenilir), 100 -> koyu
        rgb[~vs] = INVALID_RGB
        self.emit_frame('confidence', rgb)
        if fin.any():
            # guvenilirlik 0-1: 1 - (ortalama/100)
            self.record_series('conf_mean', 1.0 - float(conf[fin].mean()) / 100.0)

    def _on_info(self, msg: DepthInfoStamped):
        if not self._info_thr.ok():
            return
        self.emit_scalar('depth_info', {'min': round(float(msg.min_depth), 3),
                                        'max': round(float(msg.max_depth), 3)})

    # ---------------- analitik ----------------

    def compute_analytics(self):
        import pandas as pd  # noqa: F401 (series_df kullanir)

        def to_arr(name):
            vals = self.series.get(name, [])
            if not vals:
                return np.empty(0), np.empty(0)
            a = np.array(vals)
            return a[:, 0], a[:, 1]

        summary, charts = {}, []

        t, vr = to_arr('valid_ratio')
        if len(vr):
            summary['ort_gecerli_piksel_orani'] = round(float(vr.mean()), 3)
            charts.append({'id': 'valid_ratio', 'title': 'Gecerli Piksel Orani',
                           'unit': '', 'type': 'line',
                           'series': [{'name': 'oran',
                                       'points': self.downsample(
                                           list(zip(t, vr)))}]})

        td, dm = to_arr('depth_mean')
        if len(dm):
            summary['ort_derinlik_m'] = round(float(dm.mean()), 3)
            charts.append({'id': 'depth_mean', 'title': 'Ortalama Derinlik',
                           'unit': 'm', 'type': 'line',
                           'series': [{'name': 'derinlik',
                                       'points': self.downsample(
                                           list(zip(td, dm)))}]})
        if self._depth_min_seen is not None:
            summary['min_derinlik_m'] = round(self._depth_min_seen, 3)
        if self._depth_max_seen is not None:
            summary['maks_derinlik_m'] = round(self._depth_max_seen, 3)

        tc, cr = to_arr('conf_mean')
        if len(cr):
            summary['ort_guvenilirlik'] = round(float(cr.mean()), 3)
            charts.append({'id': 'reliability', 'title': 'Derinlik Guvenilirligi',
                           'unit': '', 'type': 'line',
                           'series': [{'name': 'guvenilirlik',
                                       'points': self.downsample(
                                           list(zip(tc, cr)))}]})

        if self._last_hist:
            pts = [[c, n] for c, n in zip(self._last_hist['centers'],
                                          self._last_hist['counts'])]
            charts.append({'id': 'depth_hist',
                           'title': 'Derinlik Dagilimi (son kare)',
                           'unit': 'piksel', 'type': 'histogram',
                           'series': [{'name': 'derinlik_m',
                                       'points': self.downsample(pts)}]})

        return {'summary': summary, 'charts': charts}


if __name__ == '__main__':
    run_feature(DepthFeature)
