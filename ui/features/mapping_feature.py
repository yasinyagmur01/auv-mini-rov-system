#!/usr/bin/env python3
"""3D Haritalama (spatial mapping) ozelligi.

Abonelikler:
  /zed/zed_node/mapping/fused_cloud   sensor_msgs/PointCloud2  (~1 Hz)

Canli/kayit kanallari:
  fused (cloud; yalniz nokta sayisi degistiginde veya son kayittan >5 sn
  gectiginde — diski sisirmemek icin), map_stats (scalar, her mesajda)

Analitik (pandas/numpy):
  nokta sayisi buyumesi, haritalanan hacim (bbox), buyume hizi (nokta/s),
  harita kapsami (son bbox boyutlari).
"""

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'common'))
from feature_base import FeatureBase, sensor_qos, run_feature  # noqa: E402
from ros_np import pointcloud2_to_xyzrgb  # noqa: E402

from sensor_msgs.msg import PointCloud2  # noqa: E402


class MappingFeature(FeatureBase):
    FEATURE_ID = 'mapping'
    TITLE = '3D Haritalama'

    def __init__(self, session_dir):
        super().__init__(session_dir)
        self._last_cloud_count = None    # son KAYDEDILEN bulut nokta sayisi
        self._last_cloud_time = 0.0      # son KAYDEDILEN bulut zamani (monotonic)
        self._prev_count = None          # buyume hizi icin onceki mesaj sayisi
        self._prev_time = None           # buyume hizi icin onceki mesaj zamani
        self._last_bbox = None           # analitik icin son bbox [dx, dy, dz]

        self.create_subscription(
            PointCloud2, '/zed/zed_node/mapping/fused_cloud',
            self._on_fused_cloud, sensor_qos())

    # ---------------- callbacks ----------------

    def _on_fused_cloud(self, msg: PointCloud2):
        xyz, rgb = pointcloud2_to_xyzrgb(msg)
        n = int(xyz.shape[0])
        now = time.monotonic()

        # bbox + hacim (GEN_3 modunda ya da baslangicta bulut bos olabilir)
        if n > 0:
            mins = xyz.min(axis=0)
            maxs = xyz.max(axis=0)
            dx, dy, dz = (float(v) for v in (maxs - mins))
        else:
            dx = dy = dz = 0.0
        volume = dx * dy * dz
        self._last_bbox = [round(dx, 2), round(dy, 2), round(dz, 2)]

        # buyume hizi (nokta/s): ilk mesajda 0, dt<=0 durumuna karsi korumali
        if self._prev_count is None or self._prev_time is None:
            growth = 0.0
        else:
            dt = now - self._prev_time
            growth = (n - self._prev_count) / dt if dt > 1e-6 else 0.0
        self._prev_count = n
        self._prev_time = now

        # scalar her mesajda
        self.emit_scalar('map_stats', {
            'count': n,
            'bbox': [round(dx, 2), round(dy, 2), round(dz, 2)],
            'volume_m3': round(volume, 2),
            'growth_pts_s': round(growth, 1),
        })
        self.record_series('count', n)
        self.record_series('volume', volume)
        self.record_series('growth', growth)

        # cloud yalniz nokta sayisi degistiyse VEYA 30 sn'lik nabizda (degismeyen
        # ~800KB'lik bulutu her 5 sn'de yeniden gondermek UI'yi bogar)
        if n != self._last_cloud_count or now - self._last_cloud_time > 30.0:
            self.emit_cloud('fused', xyz, rgb, live_max=25000)
            self._last_cloud_count = n
            self._last_cloud_time = now

    # ---------------- analitik ----------------

    def compute_analytics(self):
        import pandas as pd  # noqa: F401 (series_df kullanir)

        def to_arr(name):
            vals = self.series.get(name, [])
            if not vals:
                return np.empty(0), np.empty(0)
            a = np.array(vals)
            return a[:, 0], a[:, 1]

        t, count = to_arr('count')
        tv, volume = to_arr('volume')
        tg, growth = to_arr('growth')

        summary, charts = {}, []
        if len(t) >= 1:
            summary.update({
                'son_nokta_sayisi': int(count[-1]),
                'maks_nokta_sayisi': int(count.max()),
            })
            charts.append({'id': 'count', 'title': 'Nokta Sayisi Buyumesi',
                           'unit': 'nokta', 'type': 'line',
                           'series': [{'name': 'nokta',
                                       'points': self.downsample(
                                           list(zip(t, count)))}]})
        if len(tv) >= 1:
            summary['son_hacim_m3'] = round(float(volume[-1]), 2)
            charts.append({'id': 'volume', 'title': 'Haritalanan Hacim',
                           'unit': 'm3', 'type': 'line',
                           'series': [{'name': 'hacim',
                                       'points': self.downsample(
                                           list(zip(tv, volume)))}]})
        if len(tg) >= 1:
            summary['ort_buyume_nokta_s'] = round(float(growth.mean()), 1)
            charts.append({'id': 'growth', 'title': 'Buyume Hizi',
                           'unit': 'nokta/s', 'type': 'line',
                           'series': [{'name': 'buyume',
                                       'points': self.downsample(
                                           list(zip(tg, growth)))}]})
        if self._last_bbox is not None:
            # harita kapsami: son bilinen bbox boyutlari [dx, dy, dz]
            summary['harita_kapsami_m'] = self._last_bbox
        return {'summary': summary, 'charts': charts}


if __name__ == '__main__':
    run_feature(MappingFeature)
