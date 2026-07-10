#!/usr/bin/env python3
"""Serit takibi node'u.

Girdi : /camera/color/image_raw   (sensor_msgs/Image, D435 renk)
Cikti : /lane/cmd_vel             (geometry_msgs/Twist; linear.x 0..1 ileri,
                                   angular.z -1..1 yaw, + = saga)
        /lane/status              (auv_msgs/LaneStatus)
        /lane/debug_image         (sensor_msgs/Image, istege bagli maske)

Kontrol yetkisi auv_mission'dadir: bu node yalnizca ONERILEN hizi yayinlar;
mission node LANE_FOLLOW durumundayken bunu MANUAL_CONTROL'e cevirir.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from auv_msgs.msg import LaneStatus
from cv_bridge import CvBridge

from .lane_detector import LaneDetector


class LaneFollowNode(Node):
    def __init__(self):
        super().__init__('lane_follow')

        self.declare_parameter('cruise_speed', 0.4)      # 0..1 normalize ileri hiz
        self.declare_parameter('kp_lateral', 0.9)
        self.declare_parameter('kp_angle', 0.02)         # derece basina
        self.declare_parameter('lost_timeout_s', 2.0)
        self.declare_parameter('end_hint_frames', 8)     # ust uste kare esigi
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('hsv_lower', [0, 80, 80])
        self.declare_parameter('hsv_upper', [15, 255, 255])
        self.declare_parameter('hsv_lower2', [165, 80, 80])
        self.declare_parameter('hsv_upper2', [180, 255, 255])

        self.cruise = float(self.get_parameter('cruise_speed').value)
        self.kp_lat = float(self.get_parameter('kp_lateral').value)
        self.kp_ang = float(self.get_parameter('kp_angle').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout_s').value)
        self.end_frames_needed = int(self.get_parameter('end_hint_frames').value)

        self.detector = LaneDetector(
            hsv_lower=tuple(self.get_parameter('hsv_lower').value),
            hsv_upper=tuple(self.get_parameter('hsv_upper').value),
            hsv_lower2=tuple(self.get_parameter('hsv_lower2').value),
            hsv_upper2=tuple(self.get_parameter('hsv_upper2').value),
        )
        self.bridge = CvBridge()

        self.status = LaneStatus.SEARCHING
        self.last_seen = 0.0
        self.ever_tracked = False
        self.end_hint_count = 0

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, qos)
        self.pub_cmd = self.create_publisher(Twist, '/lane/cmd_vel', qos)
        self.pub_status = self.create_publisher(LaneStatus, '/lane/status', qos)
        self.pub_debug = (self.create_publisher(Image, '/lane/debug_image', qos)
                          if self.get_parameter('publish_debug').value else None)

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        res = self.detector.detect(frame)
        now = time.monotonic()

        st = LaneStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        cmd = Twist()

        if res.found:
            self.last_seen = now
            self.ever_tracked = True
            self.end_hint_count = self.end_hint_count + 1 if res.end_of_line_hint else 0

            if self.end_hint_count >= self.end_frames_needed:
                # serit ust ucu goruntu icinde bitiyor -> tahta sonu
                st.status = LaneStatus.END_OF_LINE
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                st.status = LaneStatus.TRACKING
                cmd.linear.x = self.cruise * max(0.3, res.confidence)
                steer = self.kp_lat * res.lateral_offset + self.kp_ang * res.angle_deg
                cmd.angular.z = max(-1.0, min(1.0, steer))

            st.lateral_offset = res.lateral_offset
            st.angle_deg = res.angle_deg
            st.confidence = res.confidence
        else:
            self.end_hint_count = 0
            if not self.ever_tracked:
                st.status = LaneStatus.SEARCHING
            elif (now - self.last_seen) > self.lost_timeout:
                st.status = LaneStatus.LOST
            else:
                # kisa kesinti: son bilinen yonde yavas devam
                st.status = LaneStatus.TRACKING
                cmd.linear.x = self.cruise * 0.3

        self.pub_cmd.publish(cmd)
        self.pub_status.publish(st)

        if self.pub_debug is not None and res.debug_mask is not None:
            self.pub_debug.publish(
                self.bridge.cv2_to_imgmsg(res.debug_mask, encoding='mono8'))


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
