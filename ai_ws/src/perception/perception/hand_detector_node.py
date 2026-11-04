#!/usr/bin/env python3
"""
USB-camera hand detector using MediaPipe Hands.

Publishes:
    /perception/hand_debug   std_msgs/String (JSON: valid, bbox, center, area)
    /perception/hand_image   sensor_msgs/Image (annotated BGR frame)

The depth camera is not required for this node; it opens the UVC camera
directly (default /dev/video0).
"""

import json
import math
import time

import cv2
import mediapipe as mp
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class HandDetectorNode(Node):
    def __init__(self):
        super().__init__('hand_detector_node')

        self.declare_parameter('device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('max_num_hands', 1)
        self.declare_parameter('model_complexity', 0)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('frame_id', 'usb_camera_optical_frame')
        # Monocular Cartesian estimate: Z from the hand bbox height, then
        # back-project the bbox centre.  Calibrate fx/fy/cx/cy for accuracy.
        self.declare_parameter('camera_fx', 600.0)
        self.declare_parameter('camera_fy', 600.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        self.declare_parameter('hand_height_m', 0.20)
        self.declare_parameter('min_distance_m', 0.20)
        self.declare_parameter('max_distance_m', 3.0)

        self.device = int(self.get_parameter('device').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = max(1.0, float(self.get_parameter('fps').value))
        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.fx = float(self.get_parameter('camera_fx').value)
        self.fy = float(self.get_parameter('camera_fy').value)
        self.cx = float(self.get_parameter('camera_cx').value)
        self.cy = float(self.get_parameter('camera_cy').value)
        self.hand_height_m = float(self.get_parameter('hand_height_m').value)
        self.min_distance_m = float(
            self.get_parameter('min_distance_m').value)
        self.max_distance_m = float(
            self.get_parameter('max_distance_m').value)

        max_num_hands = max(1, int(self.get_parameter('max_num_hands').value))
        model_complexity = int(self.get_parameter('model_complexity').value)
        det_conf = float(self.get_parameter('min_detection_confidence').value)
        track_conf = float(self.get_parameter('min_tracking_confidence').value)

        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            model_complexity=model_complexity,
            min_detection_confidence=det_conf,
            min_tracking_confidence=track_conf,
        )

        self.bridge = CvBridge()
        self.debug_pub = self.create_publisher(
            String, '/perception/hand_debug', 10)
        self.image_pub = self.create_publisher(
            Image, '/perception/hand_image', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/perception/hand_pose', 10)

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Cannot open /dev/video{self.device}. '
                'Run scripts/setup_orbbec_uvc.sh inside the container if '
                'the camera was plugged in after container start.')
        else:
            self.get_logger().info(
                f'USB camera /dev/video{self.device} opened at '
                f'{self.width}x{self.height} MJPG')

        self._last_frame_time = time.time()
        self.create_timer(1.0 / self.fps, self._tick)
        self.get_logger().info(
            f'HandDetector ready: max_hands={max_num_hands}, '
            f'model_complexity={model_complexity}, fps={self.fps}')

    def _tick(self):
        if self.cap is None or not self.cap.isOpened():
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warning('Failed to read a camera frame')
            return

        now = time.time()
        dt = max(1e-6, now - self._last_frame_time)
        self._last_frame_time = now

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)

        out = frame.copy()
        debug = {
            'valid': False,
            'message': 'no_hand',
            'fps': round(1.0 / dt, 1),
        }
        h, w = out.shape[:2]

        if result.multi_hand_landmarks:
            # Use the first hand (max_num_hands defaults to 1).
            landmarks = result.multi_hand_landmarks[0]
            xs = [lm.x * w for lm in landmarks.landmark]
            ys = [lm.y * h for lm in landmarks.landmark]
            pad = 20
            x1 = int(max(0, min(xs) - pad))
            y1 = int(max(0, min(ys) - pad))
            x2 = int(min(w - 1, max(xs) + pad))
            y2 = int(min(h - 1, max(ys) + pad))

            handedness = 'unknown'
            score = 0.0
            if result.multi_handedness:
                cls = result.multi_handedness[0].classification[0]
                handedness = cls.label
                score = float(cls.score)

            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(out, f'HAND {handedness} {score:.2f}',
                        (x1, max(y1 - 8, 18)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            self.mp_drawing.draw_landmarks(
                out, landmarks, self.mp_hands.HAND_CONNECTIONS)

            u = (x1 + x2) / 2.0
            v = (y1 + y2) / 2.0
            h_px = max(1.0, float(y2 - y1))

            # Monocular distance proxy: known/assumed hand height.  This is
            # suitable for direction control; calibrate for metric accuracy.
            z_m = self.fy * self.hand_height_m / h_px
            z_m = max(self.min_distance_m, min(self.max_distance_m, z_m))
            x_m = (u - self.cx) * z_m / self.fx
            y_m = (v - self.cy) * z_m / self.fy
            bearing_rad = math.atan2(x_m, z_m)
            distance_m = math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m)

            debug.update({
                'valid': True,
                'message': 'ok',
                'bbox_px': [x1, y1, x2 - x1, y2 - y1],
                'center_px': [u, v],
                'area_px': (x2 - x1) * (y2 - y1),
                'handedness': handedness,
                'score': round(score, 3),
                'cartesian_m': [round(x_m, 3), round(y_m, 3),
                                round(z_m, 3)],
                'bearing_rad': round(bearing_rad, 3),
                'distance_m': round(distance_m, 3),
            })

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = x_m
            pose.pose.position.y = y_m
            pose.pose.position.z = z_m
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)

            cv2.putText(
                out,
                f"XYZ=({x_m:.2f},{y_m:.2f},{z_m:.2f})m "
                f"bearing={bearing_rad:.2f}rad",
                (x1, min(h - 8, y2 + 22)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        else:
            cv2.putText(out, 'NO HAND', (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        self.debug_pub.publish(
            String(data=json.dumps(debug, ensure_ascii=False)))

        if self.publish_debug_image:
            msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.image_pub.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        try:
            self.hands.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
