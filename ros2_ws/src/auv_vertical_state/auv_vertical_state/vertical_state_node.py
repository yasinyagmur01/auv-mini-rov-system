#!/usr/bin/env python3
"""Bar30 (derinlik) + Ping Sonar (dipten yukseklik) -> /vertical_state.

"Net Y ekseni" verisi: her iki sensor medyan filtreden gecirilir,
tazelik ve menzil kontrolu yapilir, su sutunu (taban derinligi) hesaplanir.
Panel gosterimi ve otonomideki minimum-yukseklik korumasi bunu kullanir.
"""
import time
from collections import deque
from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from auv_msgs.msg import VerticalState


class VerticalStateNode(Node):
    def __init__(self):
        super().__init__('vertical_state')

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('median_window', 5)
        self.declare_parameter('sensor_timeout_s', 1.5)
        self.declare_parameter('altitude_min_m', 0.3)   # Ping olu bolgesi
        self.declare_parameter('altitude_max_m', 30.0)

        self.win = int(self.get_parameter('median_window').value)
        self.timeout = float(self.get_parameter('sensor_timeout_s').value)
        self.alt_min = float(self.get_parameter('altitude_min_m').value)
        self.alt_max = float(self.get_parameter('altitude_max_m').value)

        self.depth_buf = deque(maxlen=self.win)
        self.alt_buf = deque(maxlen=self.win)
        self.depth_stamp = 0.0
        self.alt_stamp = 0.0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float32, '/mav/depth', self.on_depth, qos)
        self.create_subscription(Range, '/mav/rangefinder', self.on_range, qos)
        self.pub = self.create_publisher(VerticalState, '/vertical_state', qos)

        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self.tick)

    def on_depth(self, msg: Float32):
        self.depth_buf.append(msg.data)
        self.depth_stamp = time.monotonic()

    def on_range(self, msg: Range):
        # menzil disi olcumler tampona girmez (gecersiz sayilir)
        if self.alt_min <= msg.range <= self.alt_max:
            self.alt_buf.append(msg.range)
            self.alt_stamp = time.monotonic()

    def tick(self):
        now = time.monotonic()
        out = VerticalState()
        out.header.stamp = self.get_clock().now().to_msg()

        out.depth_valid = bool(self.depth_buf) and (now - self.depth_stamp) < self.timeout
        out.altitude_valid = bool(self.alt_buf) and (now - self.alt_stamp) < self.timeout

        out.depth_m = float(median(self.depth_buf)) if out.depth_valid else 0.0
        out.altitude_m = float(median(self.alt_buf)) if out.altitude_valid else 0.0
        out.water_column_m = (out.depth_m + out.altitude_m
                              if (out.depth_valid and out.altitude_valid) else 0.0)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VerticalStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
