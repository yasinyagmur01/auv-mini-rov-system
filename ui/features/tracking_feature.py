#!/usr/bin/env python3
"""Positional Tracking (VIO) ozelligi.

Abonelikler:
  /zed/zed_node/pose                  geometry_msgs/PoseStamped   (map cercevesi)
  /zed/zed_node/odom                  nav_msgs/Odometry           (odom cercevesi)
  /zed/zed_node/pose_with_covariance  geometry_msgs/PoseWithCovarianceStamped

Canli/kayit kanallari:
  pose (10 Hz), odom (10 Hz), cov (2 Hz)

Analitik (pandas/numpy):
  kat edilen yol, yer degistirme, hiz profili, pose-odom sapmasi (loop closure
  duzeltme buyuklugu), yukseklik profili, kovaryans izi.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'common'))
from feature_base import FeatureBase, Throttle, sensor_qos, run_feature  # noqa: E402
from ros_np import quat_to_rpy  # noqa: E402

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402


class TrackingFeature(FeatureBase):
    FEATURE_ID = 'tracking'
    TITLE = 'Konum Takibi (VIO)'

    def __init__(self, session_dir):
        super().__init__(session_dir)
        self._pose_thr = Throttle(10.0)
        self._odom_thr = Throttle(10.0)
        self._cov_thr = Throttle(2.0)

        self.create_subscription(
            PoseStamped, '/zed/zed_node/pose', self._on_pose, sensor_qos())
        self.create_subscription(
            Odometry, '/zed/zed_node/odom', self._on_odom, sensor_qos())
        self.create_subscription(
            PoseWithCovarianceStamped, '/zed/zed_node/pose_with_covariance',
            self._on_cov, sensor_qos())

    # ---------------- callbacks ----------------

    def _on_pose(self, msg: PoseStamped):
        if not self._pose_thr.ok():
            return
        p, q = msg.pose.position, msg.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        data = {'x': round(p.x, 4), 'y': round(p.y, 4), 'z': round(p.z, 4),
                'roll': round(roll, 4), 'pitch': round(pitch, 4),
                'yaw': round(yaw, 4)}
        self.emit_pose('pose', data)
        self.record_series('pose_x', p.x)
        self.record_series('pose_y', p.y)
        self.record_series('pose_z', p.z)
        self.record_series('yaw', yaw)

    def _on_odom(self, msg: Odometry):
        if not self._odom_thr.ok():
            return
        p = msg.pose.pose.position
        self.emit_pose('odom', {'x': round(p.x, 4), 'y': round(p.y, 4),
                                'z': round(p.z, 4)})
        self.record_series('odom_x', p.x)
        self.record_series('odom_y', p.y)
        self.record_series('odom_z', p.z)

    def _on_cov(self, msg: PoseWithCovarianceStamped):
        if not self._cov_thr.ok():
            return
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        trace_pos = float(cov[0, 0] + cov[1, 1] + cov[2, 2])
        self.emit_scalar('cov', {'trace_pos': round(trace_pos, 6),
                                 'sx': round(float(cov[0, 0]), 6),
                                 'sy': round(float(cov[1, 1]), 6),
                                 'sz': round(float(cov[2, 2]), 6)})
        self.record_series('cov_trace', trace_pos)

    # ---------------- analitik ----------------

    def compute_analytics(self):
        import pandas as pd  # noqa: F401 (series_df kullanir)

        def to_arr(name):
            vals = self.series.get(name, [])
            if not vals:
                return np.empty(0), np.empty(0)
            a = np.array(vals)
            return a[:, 0], a[:, 1]

        t, x = to_arr('pose_x')
        _, y = to_arr('pose_y')
        _, z = to_arr('pose_z')

        summary, charts = {}, []
        if len(t) >= 2:
            n = min(len(x), len(y), len(z))
            t, x, y, z = t[:n], x[:n], y[:n], z[:n]
            seg = np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2 + np.diff(z) ** 2)
            dt = np.clip(np.diff(t), 1e-3, None)
            speed = seg / dt
            path_len = float(seg.sum())
            disp = float(np.sqrt((x[-1] - x[0]) ** 2 + (y[-1] - y[0]) ** 2 +
                                 (z[-1] - z[0]) ** 2))
            summary.update({
                'sure_s': round(float(t[-1] - t[0]), 1),
                'yol_uzunlugu_m': round(path_len, 3),
                'yer_degistirme_m': round(disp, 3),
                'ort_hiz_m_s': round(float(speed.mean()), 3),
                'maks_hiz_m_s': round(float(np.percentile(speed, 99)), 3),
                'derinlik_araligi_m': [round(float(z.min()), 3),
                                       round(float(z.max()), 3)],
            })
            charts.append({'id': 'xy', 'title': 'XY Yorunge (ustten)',
                           'type': 'trajectory',
                           'series': [
                               {'name': 'pose',
                                'points': self.downsample(list(zip(x, y)))},
                           ]})
            # odom yorungesi (drift karsilastirmasi)
            to_, xo = to_arr('odom_x')
            _, yo = to_arr('odom_y')
            if len(xo) >= 2:
                no = min(len(xo), len(yo))
                charts[-1]['series'].append(
                    {'name': 'odom',
                     'points': self.downsample(list(zip(xo[:no], yo[:no])))})
                # pose-odom sapmasi: ortak zaman izgarasina enterpolasyon
                nmin = min(n, no)
                gx = np.interp(t[:nmin], to_[:no], xo[:no])
                gy = np.interp(t[:nmin], to_[:no], yo[:no])
                div = np.sqrt((x[:nmin] - gx) ** 2 + (y[:nmin] - gy) ** 2)
                summary['maks_pose_odom_sapmasi_m'] = round(float(div.max()), 4)
                charts.append({'id': 'div', 'title': 'Pose–Odom Sapmasi (loop closure)',
                               'unit': 'm', 'type': 'line',
                               'series': [{'name': 'sapma',
                                           'points': self.downsample(
                                               list(zip(t[:nmin], div)))}]})
            charts.append({'id': 'z', 'title': 'Z (yukseklik/derinlik) Profili',
                           'unit': 'm', 'type': 'line',
                           'series': [{'name': 'z',
                                       'points': self.downsample(list(zip(t, z)))}]})
            charts.append({'id': 'speed', 'title': 'Hiz Profili', 'unit': 'm/s',
                           'type': 'line',
                           'series': [{'name': 'hiz',
                                       'points': self.downsample(
                                           list(zip(t[1:], speed)))}]})
        ct, cv = to_arr('cov_trace')
        if len(ct) >= 2:
            summary['ort_kovaryans_izi'] = round(float(cv.mean()), 6)
            charts.append({'id': 'cov', 'title': 'Konum Kovaryans Izi',
                           'unit': 'm²', 'type': 'line',
                           'series': [{'name': 'iz(cov)',
                                       'points': self.downsample(
                                           list(zip(ct, cv)))}]})
        return {'summary': summary, 'charts': charts}


if __name__ == '__main__':
    run_feature(TrackingFeature)
