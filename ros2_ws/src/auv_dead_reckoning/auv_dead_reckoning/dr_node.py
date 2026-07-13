#!/usr/bin/env python3
"""Olu hesap (dead reckoning) node'u.

Yuzeyde GPS fix'i alinir (servis: /dr/capture_origin), dalista pusula +
komutlanan gaz -> hiz tablosu ile 10 Hz konum entegrasyonu yapilir.
Akinti onyargisi (/dr/set_current_bias) panelden ayarlanabilir.

Yayin : /dr/state (auv_msgs/DeadReckonState)
Servis: /dr/capture_origin (std_srvs/Trigger)  - siradaki N ornekten medyan fix
        /dr/reset          (std_srvs/Trigger)  - konumu sifirla (origin kalir)
Abone : /mav/heading_deg, /mav/gps_info, /mav/manual_control (komutlanan gaz),
        /dr/set_current_bias (geometry_msgs kullanilmaz; std_msgs Float32MultiArray
        [dogu_mps, kuzey_mps])
"""
import math
import time
from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Trigger

from auv_msgs.msg import DeadReckonState, GpsInfo, ManualControl

from .geo import offset_to_latlon
from .speed_table import SpeedTable


class DeadReckonNode(Node):
    def __init__(self):
        super().__init__('dead_reckoning')

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('speed_table_csv', '')
        self.declare_parameter('origin_samples', 20)
        self.declare_parameter('origin_min_fix_type', 3)
        self.declare_parameter('origin_min_satellites', 6)

        csv_path = self.get_parameter('speed_table_csv').value
        self.table = SpeedTable(csv_path if csv_path else None)
        if not csv_path:
            self.get_logger().warn(
                'speed_table_csv verilmedi - kaba varsayilan kullaniliyor. '
                'Havuz kalibrasyonundan sonra config/speed_table.csv gecirin!')

        self.origin = None          # (lat, lon)
        self.x_east = 0.0
        self.y_north = 0.0
        self.dist_total = 0.0
        self.heading_deg = 0.0
        self.heading_stamp = 0.0
        self.throttle_x = 0.0       # son komutlanan ileri gaz
        self.bias_e = 0.0           # akinti onyargisi (m/s)
        self.bias_n = 0.0

        self._capturing = 0
        self._cap_lats = []
        self._cap_lons = []

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float32, '/mav/heading_deg', self.on_heading, qos)
        self.create_subscription(GpsInfo, '/mav/gps_info', self.on_gps, qos)
        self.create_subscription(ManualControl, '/mav/manual_control', self.on_manual, 10)
        self.create_subscription(Float32MultiArray, '/dr/set_current_bias',
                                 self.on_bias, 10)

        self.pub = self.create_publisher(DeadReckonState, '/dr/state', qos)
        self.create_service(Trigger, '/dr/capture_origin', self.srv_capture)
        self.create_service(Trigger, '/dr/reset', self.srv_reset)

        self._last_tick = time.monotonic()
        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self.tick)

    # ---------------- girisler ----------------

    def on_heading(self, msg: Float32):
        self.heading_deg = msg.data
        self.heading_stamp = time.monotonic()

    def on_manual(self, msg: ManualControl):
        self.throttle_x = float(msg.x)

    def on_bias(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.bias_e, self.bias_n = float(msg.data[0]), float(msg.data[1])
            self.get_logger().info(
                f'Akinti onyargisi: dogu={self.bias_e:.2f} kuzey={self.bias_n:.2f} m/s')

    def on_gps(self, msg: GpsInfo):
        if self._capturing <= 0:
            return
        min_fix = int(self.get_parameter('origin_min_fix_type').value)
        min_sat = int(self.get_parameter('origin_min_satellites').value)
        if msg.fix_type < min_fix or msg.satellites < min_sat:
            return
        self._cap_lats.append(msg.lat)
        self._cap_lons.append(msg.lon)
        self._capturing -= 1
        if self._capturing == 0:
            self.origin = (median(self._cap_lats), median(self._cap_lons))
            self.x_east = self.y_north = self.dist_total = 0.0
            self.get_logger().info(
                f'Origin fix: {self.origin[0]:.7f}, {self.origin[1]:.7f} '
                f'({len(self._cap_lats)} ornek)')

    # ---------------- servisler ----------------

    def srv_capture(self, req, res):
        n = int(self.get_parameter('origin_samples').value)
        self._cap_lats, self._cap_lons = [], []
        self._capturing = n
        res.success = True
        res.message = f'{n} GPS ornegi toplaniyor (3D fix bekleniyor)'
        return res

    def srv_reset(self, req, res):
        self.x_east = self.y_north = self.dist_total = 0.0
        res.success = True
        res.message = 'DR konumu sifirlandi'
        return res

    # ---------------- entegrasyon ----------------

    def tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        if dt <= 0 or dt > 1.0:
            return

        v = self.table.speed(self.throttle_x)
        hdg = math.radians(self.heading_deg)
        # pruva tazeligi: 2 sn'den eskiyse ilerletme (guvenli taraf)
        if (now - self.heading_stamp) < 2.0:
            de = (v * math.sin(hdg) + self.bias_e) * dt
            dn = (v * math.cos(hdg) + self.bias_n) * dt
            self.x_east += de
            self.y_north += dn
            self.dist_total += abs(v) * dt

        out = DeadReckonState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.origin_valid = self.origin is not None
        if self.origin:
            out.origin_lat, out.origin_lon = self.origin
            out.est_lat, out.est_lon = offset_to_latlon(
                self.origin[0], self.origin[1], self.x_east, self.y_north)
        out.x_east_m = self.x_east
        out.y_north_m = self.y_north
        out.heading_deg = self.heading_deg
        out.speed_mps = v
        out.distance_total_m = self.dist_total
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
