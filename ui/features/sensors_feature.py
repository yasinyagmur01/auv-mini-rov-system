#!/usr/bin/env python3
"""Sensorler (IMU / Manyetometre / Barometre / Sicaklik) ozelligi.

Abonelikler:
  /zed/zed_node/imu/data   sensor_msgs/Imu            (~100 Hz)
  /zed/zed_node/imu/mag    sensor_msgs/MagneticField  (Tesla)
  /zed/zed_node/atm_press  sensor_msgs/FluidPressure  (Pascal)
  + baslangicta 2 sn'lik tek atimlik kesif: adinda 'temperature' gecen ve tipi
    sensor_msgs/Temperature olan TUM topic'lere abone olunur.

Canli/kayit kanallari:
  imu (10 Hz), mag (5 Hz), baro (2 Hz), temp_* (0.5 Hz, topic basina)

Analitik (pandas/numpy):
  ortalama ivme, titresim RMS, ortalama acisal hiz, yaw kaymasi (derece),
  ortalama basinc, son sicakliklar; ivme/gyro/RPY/manyetik/basinc grafikleri.
"""

import math
import os
import sys
from functools import partial

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'common'))
from feature_base import FeatureBase, Throttle, sensor_qos, run_feature  # noqa: E402
from ros_np import quat_to_rpy  # noqa: E402

from sensor_msgs.msg import (FluidPressure, Imu, MagneticField,  # noqa: E402
                             Temperature)

SERIES_CAP = 200000  # accel/gyro tam hizda kaydedilir; RAM icin ust sinir


class SensorsFeature(FeatureBase):
    FEATURE_ID = 'sensors'
    TITLE = 'Sensorler (IMU/Mag/Baro)'

    def __init__(self, session_dir):
        super().__init__(session_dir)
        self._imu_thr = Throttle(10.0)
        self._mag_thr = Throttle(5.0)
        self._baro_thr = Throttle(2.0)
        self._temp_thrs = {}        # channel -> Throttle(0.5)
        self._temp_topics = set()   # cift abonelige karsi koruma

        self.create_subscription(
            Imu, '/zed/zed_node/imu/data', self._on_imu, sensor_qos())
        self.create_subscription(
            MagneticField, '/zed/zed_node/imu/mag', self._on_mag, sensor_qos())
        self.create_subscription(
            FluidPressure, '/zed/zed_node/atm_press', self._on_baro,
            sensor_qos())

        # Sicaklik topic'leri baslangicta bilinmiyor: 2 sn sonra kesfet
        # (tek atimlik timer — graf olusana kadar bekler).
        self._discover_timer = self.create_timer(2.0, self._discover_temps)

    # ---------------- sicaklik kesfi ----------------

    def _discover_temps(self):
        self._discover_timer.cancel()
        for topic, types in self.get_topic_names_and_types():
            if 'temperature' not in topic.lower():
                continue
            if 'sensor_msgs/msg/Temperature' not in types:
                continue
            if topic in self._temp_topics:
                continue
            self._temp_topics.add(topic)
            parts = [p for p in topic.split('/') if p]
            seg = parts[-1]
            if seg.lower() in ('temperature', 'temp') and len(parts) >= 2:
                seg = parts[-2]  # /..../temperature gibi adlarda ust segment
            channel = seg if seg.startswith('temp_') else f'temp_{seg}'
            self._temp_thrs[channel] = Throttle(0.5)
            self.create_subscription(
                Temperature, topic, partial(self._on_temp, channel),
                sensor_qos())

    # ---------------- callbacks ----------------

    def _on_imu(self, msg: Imu):
        a, w, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
        accel_norm = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        gyro_norm = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)

        # Titresim analizi icin TAM callback hizinda kayit (throttlesiz),
        # ancak RAM'i korumak icin ust sinirli.
        if len(self.series['accel_norm']) < SERIES_CAP:
            self.record_series('accel_norm', accel_norm)
            self.record_series('gyro_norm', gyro_norm)

        if not self._imu_thr.ok():
            return
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.emit_scalar('imu', {
            'ax': round(a.x, 4), 'ay': round(a.y, 4), 'az': round(a.z, 4),
            'wx': round(w.x, 4), 'wy': round(w.y, 4), 'wz': round(w.z, 4),
            'roll': round(roll, 4), 'pitch': round(pitch, 4),
            'yaw': round(yaw, 4)})
        self.record_series('roll', roll)
        self.record_series('pitch', pitch)
        self.record_series('yaw', yaw)

    def _on_mag(self, msg: MagneticField):
        if not self._mag_thr.ok():
            return
        m = msg.magnetic_field
        mx, my, mz = m.x * 1e6, m.y * 1e6, m.z * 1e6  # Tesla -> microtesla
        norm_ut = math.sqrt(mx * mx + my * my + mz * mz)
        self.emit_scalar('mag', {'mx': round(mx, 3), 'my': round(my, 3),
                                 'mz': round(mz, 3),
                                 'norm_ut': round(norm_ut, 3)})
        self.record_series('mag_norm_ut', norm_ut)

    def _on_baro(self, msg: FluidPressure):
        if not self._baro_thr.ok():
            return
        hpa = msg.fluid_pressure / 100.0  # Pascal -> hPa
        self.emit_scalar('baro', {'hpa': round(hpa, 2)})
        self.record_series('baro_hpa', hpa)

    def _on_temp(self, channel, msg: Temperature):
        if not self._temp_thrs[channel].ok():
            return
        self.emit_scalar(channel, {'c': round(msg.temperature, 2)})
        self.record_series(channel, msg.temperature)

    # ---------------- analitik ----------------

    def compute_analytics(self):
        dfs = self.series_df()

        summary, charts = {}, []

        def line_chart(cid, title, unit, series_names):
            entries = []
            for name in series_names:
                pairs = self.series.get(name, [])
                if len(pairs) >= 2:
                    entries.append({'name': name,
                                    'points': self.downsample(pairs)})
            if entries:
                charts.append({'id': cid, 'title': title, 'unit': unit,
                               'type': 'line', 'series': entries})

        if 'accel_norm' in dfs:
            acc = dfs['accel_norm']['accel_norm']
            summary['ort_ivme_m_s2'] = round(float(acc.mean()), 4)
            summary['titresim_rms'] = round(float(acc.std()), 4)
        if 'gyro_norm' in dfs:
            summary['ort_gyro_rad_s'] = round(
                float(dfs['gyro_norm']['gyro_norm'].mean()), 4)
        if 'yaw' in dfs and len(dfs['yaw']) >= 2:
            yw = dfs['yaw']['yaw']
            summary['yaw_kaymasi_deg'] = round(
                float(np.degrees(yw.iloc[-1] - yw.iloc[0])), 3)
        if 'baro_hpa' in dfs:
            summary['ort_basinc_hpa'] = round(
                float(dfs['baro_hpa']['baro_hpa'].mean()), 2)
        temps = {name: round(float(df[name].iloc[-1]), 2)
                 for name, df in dfs.items() if name.startswith('temp_')}
        if temps:
            summary['sicakliklar'] = temps

        line_chart('accel', 'Ivme Buyuklugu', 'm/s2', ['accel_norm'])
        line_chart('gyro', 'Acisal Hiz', 'rad/s', ['gyro_norm'])
        line_chart('rpy', 'Yonelim (RPY)', 'rad', ['roll', 'pitch', 'yaw'])
        line_chart('mag', 'Manyetik Alan', 'uT', ['mag_norm_ut'])
        line_chart('baro', 'Basinc', 'hPa', ['baro_hpa'])

        return {'summary': summary, 'charts': charts}


if __name__ == '__main__':
    run_feature(SensorsFeature)
