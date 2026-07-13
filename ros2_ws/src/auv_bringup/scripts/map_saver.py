#!/usr/bin/env python3
# ============================================================================
# AUV 3D harita kaydedici
#
# zed-ros2-wrapper v5.4.0'da 'save_3d_map' servisi YOKTUR; fused point cloud
# yalnizca topic olarak yayinlanir (~/mapping/fused_cloud, alanlar: x,y,z,rgb
# FLOAT32 — zed_camera_component_main.cpp'den dogrulandi). Bu dugum haritayi
# diske .ply olarak yazarak depolamayi tamamlar:
#
#  - Son fused_cloud mesajini tutar.
#  - ~/save_map (std_srvs/Trigger) servisi: zaman damgali .ply yazar.
#  - autosave_period > 0 ise periyodik olarak '<prefix>_latest.ply' gunceller
#    (guc kesintisine karsi dayaniklilik; tek dosyayi ezerek diski doldurmaz).
#
# Cozumleme numpy ile vektorizedir: milyonlarca noktali haritalarda dahi
# kayit milisaniyeler surer, dugumun executor'unu bloklamaz.
# ============================================================================

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger

# PointField.datatype -> numpy tip karakteri
_DATATYPE_FMT = {
    PointField.INT8: 'i1', PointField.UINT8: 'u1',
    PointField.INT16: 'i2', PointField.UINT16: 'u2',
    PointField.INT32: 'i4', PointField.UINT32: 'u4',
    PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
}

_PLY_VERTEX = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
])


class MapSaver(Node):

    def __init__(self):
        super().__init__('map_saver')

        self.declare_parameter('cloud_topic', '/zed/zed_node/mapping/fused_cloud')
        self.declare_parameter('output_dir', '/data/maps')
        self.declare_parameter('file_prefix', 'fused_map')
        self.declare_parameter('autosave_period', 60.0)  # saniye; 0 = kapali

        self._output_dir = self.get_parameter('output_dir').value
        self._prefix = self.get_parameter('file_prefix').value
        autosave = float(self.get_parameter('autosave_period').value)
        topic = self.get_parameter('cloud_topic').value

        os.makedirs(self._output_dir, exist_ok=True)

        self._last_msg = None

        # BEST_EFFORT + VOLATILE abonelik her yayinci QoS'u ile uyumludur
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(PointCloud2, topic, self._on_cloud, qos)
        self._srv = self.create_service(Trigger, '~/save_map', self._on_save_request)

        if autosave > 0.0:
            self._timer = self.create_timer(autosave, self._on_autosave)

        self.get_logger().info(
            f"Harita kaydedici hazir: topic={topic} cikti={self._output_dir} "
            f"autosave={autosave}s")

    # ------------------------------------------------------------------ #

    def _on_cloud(self, msg: PointCloud2):
        self._last_msg = msg

    def _on_save_request(self, request, response):
        if self._last_msg is None:
            response.success = False
            response.message = 'Henuz fused_cloud mesaji alinmadi (mapping acik mi?)'
            return response
        try:
            stamp = time.strftime('%Y%m%d_%H%M%S')
            path = os.path.join(self._output_dir, f'{self._prefix}_{stamp}.ply')
            count = self._write_ply(self._last_msg, path)
            response.success = True
            response.message = f'{count} nokta kaydedildi: {path}'
            self.get_logger().info(response.message)
        except Exception as exc:  # noqa: BLE001 - servis cevabina hatayi tasi
            response.success = False
            response.message = f'Kayit hatasi: {exc}'
            self.get_logger().error(response.message)
        return response

    def _on_autosave(self):
        if self._last_msg is None:
            return
        try:
            path = os.path.join(self._output_dir, f'{self._prefix}_latest.ply')
            tmp_path = path + '.tmp'
            count = self._write_ply(self._last_msg, tmp_path)
            os.replace(tmp_path, path)  # atomik guncelleme
            self.get_logger().info(f'Otomatik kayit: {count} nokta -> {path}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Otomatik kayit hatasi: {exc}')

    # ------------------------------------------------------------------ #

    @staticmethod
    def _as_structured_array(msg: PointCloud2) -> np.ndarray:
        """PointCloud2 verisini kopyasiz (mumkunse) yapisal numpy dizisine cevir.

        row_step > width*point_step olan (dolgulu/organize) bulutlari da
        dogru cozumler.
        """
        prefix = '>' if msg.is_bigendian else '<'
        names, formats, offsets = [], [], []
        for f in msg.fields:
            if f.datatype not in _DATATYPE_FMT:
                continue
            names.append(f.name)
            formats.append(prefix + _DATATYPE_FMT[f.datatype])
            offsets.append(f.offset)
        dtype = np.dtype({'names': names, 'formats': formats,
                          'offsets': offsets, 'itemsize': msg.point_step})

        for required in ('x', 'y', 'z'):
            if required not in dtype.names:
                raise ValueError(f"PointCloud2 '{required}' alani icermiyor")

        data = np.frombuffer(msg.data, dtype=np.uint8)
        height = msg.height or 1
        row_bytes = msg.width * msg.point_step
        if msg.row_step and msg.row_step != row_bytes and height > 1:
            # satir sonu dolgusunu at
            data = data.reshape(height, msg.row_step)[:, :row_bytes].reshape(-1)
        count = msg.width * height
        return np.frombuffer(data.tobytes(), dtype=dtype, count=count)

    def _write_ply(self, msg: PointCloud2, path: str) -> int:
        cloud = self._as_structured_array(msg)

        xyz = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=1).astype(np.float32)
        finite = np.isfinite(xyz).all(axis=1)  # NaN/inf suzgeci
        xyz = xyz[finite]

        out = np.empty(xyz.shape[0], dtype=_PLY_VERTEX)
        out['x'], out['y'], out['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]

        if 'rgb' in cloud.dtype.names:
            # ROS 'rgb': float32 icinde paketlenmis 0x00RRGGBB
            packed = np.ascontiguousarray(cloud['rgb'][finite]).view(np.uint32)
            out['red'] = (packed >> 16) & 0xFF
            out['green'] = (packed >> 8) & 0xFF
            out['blue'] = packed & 0xFF
        else:
            out['red'] = out['green'] = out['blue'] = 255

        header = (
            'ply\n'
            'format binary_little_endian 1.0\n'
            f'comment AUV fused point cloud ({msg.header.frame_id}, '
            f'stamp {msg.header.stamp.sec})\n'
            f'element vertex {out.shape[0]}\n'
            'property float x\n'
            'property float y\n'
            'property float z\n'
            'property uchar red\n'
            'property uchar green\n'
            'property uchar blue\n'
            'end_header\n'
        )
        with open(path, 'wb') as fh:
            fh.write(header.encode('ascii'))
            fh.write(out.tobytes())
        return int(out.shape[0])


def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
