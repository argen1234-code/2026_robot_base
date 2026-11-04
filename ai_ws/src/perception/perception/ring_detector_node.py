#!/usr/bin/env python3
"""
ROS 2 wrapper for the A4 two-ring detector.

Current stage: reads the Orbbec RGB stream directly from /dev/video0 (UVC)
and publishes the target position in the camera optical frame.  Depth and
camera_info will be connected after the Orbbec ROS 2 driver is installed.

Topics:
    /perception/target_pose   geometry_msgs/PoseStamped
    /perception/target_debug  std_msgs/String (JSON)
    /perception/debug_image   sensor_msgs/Image
"""

import json
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from perception.ring_detector import RingSpec, detect


class RingDetectorNode(Node):
    def __init__(self):
        super().__init__('ring_detector_node')

        self.declare_parameter('device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('camera_fx', 600.0)
        self.declare_parameter('camera_fy', 600.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('test_image', '')
        self.declare_parameter('outer_outer_d_mm', 140.0)
        self.declare_parameter('outer_inner_d_mm', 110.0)
        self.declare_parameter('white_inner_d_mm', 80.0)
        self.declare_parameter('inner_inner_d_mm', 50.0)

        self.device = int(self.get_parameter('device').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = max(1.0, float(self.get_parameter('fps').value))
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value)
        self.test_image = str(self.get_parameter('test_image').value or '')

        self.spec = RingSpec(
            outer_outer_d_mm=float(self.get_parameter('outer_outer_d_mm').value),
            outer_inner_d_mm=float(self.get_parameter('outer_inner_d_mm').value),
            white_inner_d_mm=float(self.get_parameter('white_inner_d_mm').value),
            inner_inner_d_mm=float(self.get_parameter('inner_inner_d_mm').value),
        )
        self.spec.validate()

        self.camera_matrix = np.array([
            [float(self.get_parameter('camera_fx').value), 0.0,
             float(self.get_parameter('camera_cx').value)],
            [0.0, float(self.get_parameter('camera_fy').value),
             float(self.get_parameter('camera_cy').value)],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)

        self.bridge = CvBridge()
        self.pose_pub = self.create_publisher(
            PoseStamped, '/perception/target_pose', 10)
        self.debug_pub = self.create_publisher(
            String, '/perception/target_debug', 10)
        self.image_pub = self.create_publisher(
            Image, '/perception/debug_image', 10)

        self.cap = None
        self.test_frame = None
        if self.test_image:
            if not os.path.exists(self.test_image):
                self.get_logger().error(
                    f'test_image does not exist: {self.test_image}')
            else:
                self.test_frame = cv2.imread(self.test_image)
                self.get_logger().info(
                    f'Using test image: {self.test_image}')
        else:
            self._open_camera()

        self.timer = self.create_timer(1.0 / self.fps, self._tick)
        self.get_logger().info(
            f'RingDetectorNode ready: frame={self.frame_id}, '
            f'fps={self.fps}, '
            f'radii={[round(r, 1) for r in self.spec.radii_mm]} mm')

    def _open_camera(self):
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Cannot open /dev/video{self.device}. '
                'If the camera was plugged in after the container started, '
                'recreate the device node (mknod) or restart the container.')
        else:
            self.get_logger().info(
                f'Camera /dev/video{self.device} opened at '
                f'{self.width}x{self.height} MJPG')

    def _tick(self):
        if self.test_frame is not None:
            frame = self.test_frame.copy()
        else:
            if self.cap is None or not self.cap.isOpened():
                return
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.get_logger().warning('Failed to read a camera frame')
                return

        result = detect(frame, self.spec, self.camera_matrix)
        stamp = self.get_clock().now().to_msg()

        debug = {
            'valid': bool(result.valid),
            'method': result.method,
            'message': result.message,
        }
        if result.center_px is not None:
            debug['center_px'] = [float(v) for v in result.center_px]
        if result.radii_px is not None:
            debug['radii_px'] = [float(v) for v in result.radii_px]
        if result.bbox_px is not None:
            debug['bbox_px'] = [int(v) for v in result.bbox_px]
        if result.distance_m is not None:
            debug['distance_m'] = float(result.distance_m)
        if result.center_camera_mm is not None:
            debug['position_camera_m'] = [
                float(v) / 1000.0 for v in result.center_camera_mm]
        self.debug_pub.publish(
            String(data=json.dumps(debug, ensure_ascii=False)))

        if result.valid and result.center_camera_mm is not None:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(result.center_camera_mm[0]) / 1000.0
            pose.pose.position.y = float(result.center_camera_mm[1]) / 1000.0
            pose.pose.position.z = float(result.center_camera_mm[2]) / 1000.0
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)

        if self.publish_debug_image:
            if result.annotated is not None:
                out = result.annotated
            else:
                # Publish the raw camera frame even when no target is found,
                # so the temporary viewer always shows the live image.
                out = frame.copy()
                cv2.putText(out, 'NO TARGET', (20, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255), 2)
                if result.message:
                    cv2.putText(out, result.message, (20, 65),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 0, 255), 1)
            msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            self.image_pub.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RingDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
