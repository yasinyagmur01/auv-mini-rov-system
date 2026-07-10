#!/usr/bin/env python3
"""/camera/color/image_raw -> H.264/RTP -> UDP (kontrol PC).

OpenCV'nin GStreamer arka ucu ile yazilir. Jetson'da donanim kodlayici
(nvv4l2h264enc) tercih edilir; yoksa x264enc'e duser.

Not: OpenCV'nin GStreamer destegiyle derlenmis olmasi gerekir (Jetson'da
JetPack OpenCV'si destekler). Kontrol PC tarafinda panel/sdp/d435.sdp ile acilir.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream')

        self.declare_parameter('host', '192.168.2.1')
        self.declare_parameter('port', 5601)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('bitrate_kbps', 2000)
        self.declare_parameter('use_hw_encoder', True)

        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        self.w = int(self.get_parameter('width').value)
        self.h = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)
        kbps = int(self.get_parameter('bitrate_kbps').value)

        if self.get_parameter('use_hw_encoder').value:
            enc = (f'nvvidconv ! nvv4l2h264enc bitrate={kbps * 1000} '
                   'insert-sps-pps=true idrinterval=15')
        else:
            enc = (f'videoconvert ! x264enc tune=zerolatency bitrate={kbps} '
                   'speed-preset=ultrafast key-int-max=15')

        pipeline = (
            f'appsrc ! video/x-raw,format=BGR,width={self.w},height={self.h},'
            f'framerate={fps}/1 ! {enc} ! h264parse ! rtph264pay config-interval=1 '
            f'pt=96 ! udpsink host={host} port={port} sync=false'
        )
        self.get_logger().info(f'GStreamer: {pipeline}')
        self.writer = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0,
                                      float(fps), (self.w, self.h))
        if not self.writer.isOpened():
            self.get_logger().error(
                'GStreamer pipeline acilamadi! OpenCV GStreamer destegiyle '
                'derlenmis mi? use_hw_encoder:=false deneyin.')

        self.bridge = CvBridge()
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, qos)

    def on_image(self, msg: Image):
        if not self.writer.isOpened():
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame.shape[1] != self.w or frame.shape[0] != self.h:
            frame = cv2.resize(frame, (self.w, self.h))
        self.writer.write(frame)


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
