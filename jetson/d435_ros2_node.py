#!/usr/bin/env python3
"""D435 -> ROS2 yayinci (pyrealsense2 tabanli, realsense-ros yerine hafif cozum).

Neden: Jetson'da librealsense 2.58 KAYNAKTAN kurulu ve pyrealsense2 zaten
calisiyor (rsweb kullaniyor). apt ros-humble-realsense2-camera kurmak bu
kaynak kurulumla cakisabilir. Bu node ayni calisan pyrealsense2 stack'ini
kullanip goruntuyu ROS2 topic'ine yayinlar; web_video_server ile web'e cikar.

Yayinlar:
  /camera/color/image_raw    sensor_msgs/Image  (rgb8)
  /camera/depth/image_raw    sensor_msgs/Image  (16UC1, mm)  [--depth ile]

Calistirma (Jetson'da, rsweb DURDURULDUKTAN sonra - D435 tek process claim eder):
  source /opt/ros/humble/setup.bash
  python3 d435_ros2_node.py --width 640 --height 480 --fps 30
"""
import argparse

import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def make_image(stamp, frame_id, arr, encoding):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = arr.shape[0], arr.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = arr.shape[1] * arr.itemsize * (arr.shape[2] if arr.ndim == 3 else 1)
    msg.data = arr.tobytes()
    return msg


class D435Node(Node):
    def __init__(self, args):
        super().__init__('d435_camera')
        self.args = args
        self.pub_color = self.create_publisher(Image, '/camera/color/image_raw', 5)
        self.pub_depth = (self.create_publisher(Image, '/camera/depth/image_raw', 5)
                          if args.depth else None)

        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, args.width, args.height,
                          rs.format.rgb8, args.fps)
        if args.depth:
            cfg.enable_stream(rs.stream.depth, args.width, args.height,
                              rs.format.z16, args.fps)
        self.pipe.start(cfg)
        self.get_logger().info(
            f'D435 basladi {args.width}x{args.height}@{args.fps} '
            f'(depth={"acik" if args.depth else "kapali"})')
        self.create_timer(1.0 / args.fps, self.tick)

    def tick(self):
        try:
            frames = self.pipe.wait_for_frames(timeout_ms=1000)
        except Exception as e:
            self.get_logger().warn(f'kare alinamadi: {e}')
            return
        stamp = self.get_clock().now().to_msg()
        cf = frames.get_color_frame()
        if cf:
            arr = np.asanyarray(cf.get_data())
            self.pub_color.publish(make_image(stamp, 'd435_color', arr, 'rgb8'))
        if self.pub_depth:
            df = frames.get_depth_frame()
            if df:
                arr = np.asanyarray(df.get_data())  # uint16, mm
                self.pub_depth.publish(make_image(stamp, 'd435_depth', arr, '16UC1'))

    def destroy_node(self):
        try:
            self.pipe.stop()
        except Exception:
            pass
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--depth', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    node = D435Node(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
