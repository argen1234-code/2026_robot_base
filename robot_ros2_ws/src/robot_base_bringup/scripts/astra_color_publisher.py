#!/usr/bin/env python3
"""Standalone Astra Pro UVC color publisher.

This node owns ``/dev/astra-color`` and publishes JPEG
``/camera/color/image_raw/compressed`` plus camera info.  It exists so the
vision stack can run without the AI ``model_service``: when both run, the AI
node is started with ``camera_source:=ros_compressed`` and consumes this topic
instead of opening the same UVC device twice.
"""

import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage


class AstraColorPublisher(Node):
    def __init__(self):
        super().__init__('astra_color_publisher')
        self.declare_parameter('device', '/dev/astra-color')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('quality', 70)
        self.declare_parameter('compressed_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('fx', 570.0)
        self.declare_parameter('fy', 570.0)
        self.declare_parameter('cx', 319.5)
        self.declare_parameter('cy', 239.5)

        self.device = str(self.get_parameter('device').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = max(1.0, float(self.get_parameter('fps').value))
        self.quality = int(self.get_parameter('quality').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)

        self.cap = self._open_camera()
        self.compressed_pub = self.create_publisher(
            CompressedImage, str(self.get_parameter('compressed_topic').value), 1)
        self.info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter('camera_info_topic').value), 1)
        self._last_info = 0.0
        self._last_warn = 0.0
        self.create_timer(1.0 / self.fps, self.tick)
        self.get_logger().info(
            f'Astra color ready: {self.device} {self.width}x{self.height}@{self.fps:g} '
            f'-> {self.compressed_pub.topic_name}')

    def _open_camera(self):
        for fourcc in ('MJPG', 'YUYV'):
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok, frame = cap.read()
            if ok and frame is not None:
                self.get_logger().info(
                    f'Opened {self.device} with {fourcc} '
                    f'({frame.shape[1]}x{frame.shape[0]})')
                return cap
            cap.release()
        raise RuntimeError(
            f'Cannot open {self.device}; check the camera and `fuser -v {self.device}` '
            f'(another model_service/astra publisher may own it)')

    def _camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0,
                  0.0, self.fy, self.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def tick(self):
        # Skip capture/encoding when nobody is watching.
        if self.compressed_pub.get_subscription_count() == 0:
            return
        ok, frame = self.cap.read()
        if not ok or frame is None:
            now = time.monotonic()
            if now - self._last_warn > 5.0:
                self._last_warn = now
                self.get_logger().warn('Astra color frame read failed')
            return
        ok, buf = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return
        stamp = self.get_clock().now().to_msg()
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.compressed_pub.publish(msg)
        now = time.monotonic()
        if self.info_pub.get_subscription_count() > 0 and now - self._last_info > 1.0:
            self._last_info = now
            self.info_pub.publish(self._camera_info(stamp))


def main():
    rclpy.init()
    node = None
    try:
        node = AstraColorPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        elif not isinstance(exc, KeyboardInterrupt):
            print(f'astra_color_publisher: {exc}', flush=True)
        if isinstance(exc, RuntimeError):
            raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
