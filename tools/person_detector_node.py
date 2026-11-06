#!/usr/bin/env python3
"""YOLOv8 + 深度相机的简单人像检测（ROS 2, ai_host）。

订阅：
    /camera/color/image_raw/compressed   sensor_msgs/CompressedImage
    /camera/depth/image_raw              sensor_msgs/Image (16UC1)
    /camera/depth/camera_info            sensor_msgs/CameraInfo

发布：
    /perception/person_detections        std_msgs/String (JSON)
    /perception/person_pose              geometry_msgs/PoseStamped（最近的人）
    /perception/person_image/compressed  sensor_msgs/CompressedImage（标注图）

用法：
    python3 tools/person_detector_node.py \
        --model /home/pi/models/yolov8n.onnx --max-fps 3

说明：
- 模型为 YOLOv8n ONNX（输入 640×640，OpenCV DNN 推理）。
- 彩色和深度是同一台 Astra Pro 的两路流，未做严格标定；这里用归一化
  坐标把彩色框中心映射到深度图，取 ROI 中值作为距离。
  如果发现距离偏差，用 --depth-offset-x / --depth-offset-y 微调。
"""

import argparse
from collections import deque
import json
import math
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from person_depth import bbox_depth
from std_msgs.msg import String
from person_inference import ConfidenceGate, PersonInference

PERSON_CLASS_ID = 0


class PersonDetector(Node):
    def __init__(self, args):
        super().__init__('person_detector')
        self.args = args

        cv2.setNumThreads(args.cv_threads)
        if args.runtime_dir:
            sys.path.insert(0, args.runtime_dir)
        self.runner = PersonInference(args.model, backend=args.backend,
            width=args.input_size, height=args.input_height, threads=args.cv_threads,
            confidence=min(args.conf,args.support_conf), iou=args.iou)
        self.confidence_gate = ConfidenceGate(args.conf,args.support_conf,args.support_seconds)
        self.get_logger().info(f'YOLO model loaded: {args.model} backend={args.backend}')

        # OpenCV DNN 的 Net 不是线程安全的；定时器和回调可能并发。
        self._infer_lock = threading.Lock()
        self._frame_lock = threading.Lock()
        self._frame_ready = threading.Event()
        self._stop_worker = threading.Event()
        self._worker = None
        self.color_msg = None
        self.color_seq = 0
        self.processed_seq = -1
        self.depth_msg = None
        self.depth_history = deque(maxlen=12)
        # Orbbec 的 camera_info 在本机可能是 NaN，先用 Astra 640×480 的
        # 近似内参；收到有效值再覆盖。
        self.fx = float(args.fx)
        self.fy = float(args.fy)
        self.cx = float(args.cx)
        self.cy = float(args.cy)
        # 默认内参按 640×480 给出；深度流若降到 320×240，需要按比例缩放。
        self._default_intrinsics = True
        self._bad_intrinsics_logged = False
        self._last_depth_warn = 0.0
        self.last_log = 0.0
        self.last_annotation_at = 0.0

        # 只保留最新一帧；推理来不及时不排队，也不解码会被丢弃的帧。
        latest_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            CompressedImage, args.color_topic, self.color_cb, latest_qos)
        self.create_subscription(
            Image, args.depth_topic, self.depth_cb, latest_qos)
        self.create_subscription(
            CameraInfo, args.depth_info_topic, self.info_cb,
            latest_qos)

        self.det_pub = self.create_publisher(String, args.det_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, args.pose_topic, 10)
        self.img_pub = self.create_publisher(
            CompressedImage, args.image_topic, 1)

        self.get_logger().info(
            f'person detector ready: color={args.color_topic} '
            f'depth={args.depth_topic} max_fps={args.max_fps} '
            f'cv_threads={cv2.getNumThreads()} latest_frame_only=true')

    # ---------------- callbacks ----------------
    def color_cb(self, msg):
        with self._frame_lock:
            self.color_msg = msg
            self.color_seq += 1
        self._frame_ready.set()

    def start_worker(self):
        self._worker = threading.Thread(target=self._run_worker,
                                        name='person_inference', daemon=True)
        self._worker.start()

    def _run_worker(self):
        next_start = 0.0
        while not self._stop_worker.is_set():
            if not self._frame_ready.wait(.5):
                continue
            self._frame_ready.clear()
            if self._stop_worker.wait(max(0.0,next_start-time.monotonic())):
                return
            started = time.monotonic()
            try:
                self.tick()
            except Exception as error:
                self.get_logger().error(f'person inference worker failed: {error}')
                self._stop_worker.set()
                rclpy.try_shutdown()
                return
            next_start = started + 1.0/self.args.max_fps

    def stop_worker(self):
        self._stop_worker.set()
        self._frame_ready.set()
        if self._worker is not None:
            self._worker.join()

    def depth_cb(self, msg):
        if msg.encoding not in ('16UC1', 'mono16'):
            return
        with self._frame_lock:
            self.depth_msg = msg
            stamp = msg.header.stamp
            self.depth_history.append((stamp.sec + stamp.nanosec/1e9, msg))

    def info_cb(self, msg):
        k = msg.k
        fx, cx, fy, cy = float(k[0]), float(k[2]), float(k[4]), float(k[5])
        if all(math.isfinite(v) and v > 0 for v in (fx, fy, cx, cy)):
            self.fx, self.cx, self.fy, self.cy = fx, cx, fy, cy
            self._default_intrinsics = False
        elif not self._bad_intrinsics_logged:
            self._bad_intrinsics_logged = True
            self.get_logger().warn(
                'depth camera_info intrinsics invalid; using defaults '
                f'fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    # ---------------- inference ----------------
    def tick(self):
        if not self._infer_lock.acquire(blocking=False):
            return
        try:
            self._tick_impl()
        finally:
            self._infer_lock.release()

    def _tick_impl(self):
        with self._frame_lock:
            if self.color_msg is None or self.color_seq == self.processed_seq:
                return
            self.processed_seq = self.color_seq
            color_msg = self.color_msg
            source_stamp = color_msg.header.stamp
            source_seconds = source_stamp.sec + source_stamp.nanosec/1e9
            depth_msg = min(self.depth_history,
                key=lambda sample:abs(sample[0]-source_seconds))[1] if self.depth_history else None
            if depth_msg is not None:
                stamp = depth_msg.header.stamp
                skew_ms = abs(source_seconds-stamp.sec-stamp.nanosec/1e9)*1000
                if skew_ms > self.args.max_depth_skew_ms:
                    depth_msg = None

        # 保留消息引用可保证底层缓冲在推理期间有效；回调只替换引用。
        decode_start = time.perf_counter()
        img = cv2.imdecode(
            np.frombuffer(color_msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        depth = None
        if depth_msg is not None:
            try:
                dtype = np.dtype('>u2' if depth_msg.is_bigendian else '<u2')
                depth = np.ndarray(
                    (depth_msg.height, depth_msg.width), dtype=dtype,
                    buffer=depth_msg.data, strides=(depth_msg.step, 2))
            except (ValueError, TypeError):
                pass
        decode_ms = (time.perf_counter() - decode_start) * 1000.0
        source_stamp = color_msg.header.stamp
        source_seconds = source_stamp.sec + source_stamp.nanosec / 1e9
        input_age_ms = (time.time() - source_seconds) * 1000.0
        depth_seconds = None
        if depth_msg is not None:
            stamp = depth_msg.header.stamp
            depth_seconds = stamp.sec + stamp.nanosec / 1e9

        t0 = time.perf_counter()
        persons = self.detect(img)
        infer_ms = (time.perf_counter() - t0) * 1000.0

        h, w = img.shape[:2]
        for p in persons:
            self.attach_depth(p, w, h, depth)

        nearest = min(
            (p for p in persons if p.get('depth_m') is not None),
            key=lambda p: p['depth_m'], default=None)

        data = {
            'stamp': round(time.time(), 3),
            'source_stamp': round(source_seconds, 6),
            'depth_source_stamp': round(depth_seconds, 6) if depth is not None else None,
            'count': len(persons),
            'image_size': [w, h],
            'frame_id': color_msg.header.frame_id,
            'infer_ms': round(infer_ms, 1),
            'decode_ms': round(decode_ms, 1),
            'backend': self.args.backend,
            'input_shape': [self.runner.height, self.runner.width],
            'preprocess_ms': round(self.runner.last_timing.get('preprocess_ms', 0), 2),
            'forward_ms': round(self.runner.last_timing.get('forward_ms', 0), 2),
            'postprocess_ms': round(self.runner.last_timing.get('postprocess_ms', 0), 2),
            'input_age_ms': round(input_age_ms, 1),
            'result_age_ms': round((time.time() - source_seconds) * 1000.0, 1),
            'color_depth_skew_ms': (
                round((source_seconds - depth_seconds) * 1000.0, 1)
                if depth_seconds is not None else None),
            'nearest': nearest,
            'persons': persons,
        }
        self.det_pub.publish(String(data=json.dumps(
            data, ensure_ascii=False)))

        if nearest is not None:
            frame_id = depth_msg.header.frame_id or 'camera_depth_optical_frame'
            self.publish_pose(nearest, source_stamp, frame_id)
        self.publish_annotated(img, persons, color_msg.header)

        now = time.time()
        if now - self.last_log > 2.0:
            self.last_log = now
            nd = None if nearest is None else nearest.get('depth_m')
            self.get_logger().info(
                f'persons={len(persons)} infer={infer_ms:.0f}ms '
                f'input_age={input_age_ms:.0f}ms '
                f'result_age={data["result_age_ms"]:.0f}ms nearest={nd}')

    def detect(self, img):
        return self.confidence_gate.filter(self.runner.predict(img))

    # ---------------- depth / geometry ----------------
    def attach_depth(self, p, color_w, color_h, depth):
        p['depth_m'] = None
        p['depth_valid_pixels'] = 0
        p['bearing_deg'] = None
        p['xyz'] = None
        if depth is None:
            return
        dh, dw = depth.shape[:2]
        u, v = p['center']
        nx = (u / float(color_w)) + self.args.depth_offset_x
        ny = (v / float(color_h)) + self.args.depth_offset_y
        dx = int(round(nx * dw))
        dy = int(round(ny * dh))
        if not (0 <= dx < dw and 0 <= dy < dh):
            return

        z, count = bbox_depth(depth, p['bbox'], (color_w, color_h),
                              (self.args.depth_offset_x, self.args.depth_offset_y),
                              self.args.depth_min_m, self.args.depth_max_m)
        p['depth_valid_pixels'] = count
        if z is None:
            now = time.time()
            if now - self._last_depth_warn > 5.0:
                self._last_depth_warn = now
                self.get_logger().warn(
                    f'person bbox has no valid depth (>{self.args.depth_min_m*1000:.0f}mm): '
                    'depth lens may be blocked or too close')
            return
        p['depth_m'] = round(z, 3)

        fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
        if self._default_intrinsics:
            # 默认内参按 640×480 标定；深度流降到 320×240 时按比例缩放。
            sx, sy = dw / 640.0, dh / 480.0
            fx, cx = fx * sx, cx * sx
            fy, cy = fy * sy, cy * sy
        if fx and fy:
            xm = (dx - cx) * z / fx
            ym = (dy - cy) * z / fy
            p['xyz'] = [round(xm, 3), round(ym, 3), round(z, 3)]
            p['bearing_deg'] = round(np.degrees(np.arctan2(xm, z)), 1)

    @staticmethod
    def sample_depth(depth, cx, cy, radius):
        h, w = depth.shape[:2]
        x0, x1 = max(0, cx - radius), min(w, cx + radius + 1)
        y0, y1 = max(0, cy - radius), min(h, cy + radius + 1)
        roi = depth[y0:y1, x0:x1].astype(np.float32)
        valid = roi[(roi > 100.0) & (roi < 8000.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid)) / 1000.0

    def publish_pose(self, p, source_stamp, frame_id):
        xyz = p.get('xyz')
        if not xyz:
            return
        msg = PoseStamped()
        msg.header.stamp = source_stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    def publish_annotated(self, img, persons, source_header):
        if self.img_pub.get_subscription_count() == 0:
            return
        now = time.monotonic()
        if now-self.last_annotation_at < 1.0/self.args.annotation_fps:
            return
        self.last_annotation_at = now
        for p in persons:
            x, y, bw, bh = [int(v) for v in p['bbox']]
            cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            label = f"person {p['conf']:.2f}"
            if p.get('depth_m') is not None:
                label += f' {p["depth_m"]:.2f}m'
            if p.get('bearing_deg') is not None:
                label += f' {p["bearing_deg"]:+.1f}deg'
            cv2.putText(img, label, (x, max(18, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        ok, buf = cv2.imencode(
            '.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        msg = CompressedImage()
        msg.header = source_header
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.img_pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='/home/pi/models/yolov8n.onnx')
    ap.add_argument('--input-size', type=int, default=640)
    ap.add_argument('--input-height', type=int, default=None,
                    help='Model input height; defaults to --input-size (square)')
    ap.add_argument('--backend', choices=('opencv','ncnn','onnxruntime'), default='opencv')
    ap.add_argument('--runtime-dir', default='', help='Optional isolated inference dependency directory')
    ap.add_argument('--conf', type=float, default=0.35)
    ap.add_argument('--support-conf', type=float, default=0.2,
                    help='Weak current detection threshold, only with recent strong overlapping detection')
    ap.add_argument('--support-seconds', type=float, default=0.6,
                    help='Maximum age of strong evidence; weak boxes never extend this')
    ap.add_argument('--iou', type=float, default=0.45)
    ap.add_argument('--max-fps', type=float, default=3.0)
    ap.add_argument('--annotation-fps', type=float, default=3.0,
                    help='Diagnostic JPEG rate; independent of actual detection result rate')
    ap.add_argument('--max-depth-skew-ms', type=float, default=150.0)
    ap.add_argument('--cv-threads', type=int, default=2,
                    help='OpenCV CPU threads (default: 2; independent of Nav2)')
    ap.add_argument('--color-topic',
                    default='/camera/color/image_raw/compressed')
    ap.add_argument('--depth-topic', default='/camera/depth/image_raw')
    ap.add_argument('--depth-info-topic',
                    default='/camera/depth/camera_info')
    ap.add_argument('--det-topic', default='/perception/person_detections')
    ap.add_argument('--pose-topic', default='/perception/person_pose')
    ap.add_argument('--image-topic',
                    default='/perception/person_image/compressed')
    ap.add_argument('--depth-offset-x', type=float, default=0.0)
    ap.add_argument('--depth-offset-y', type=float, default=0.0)
    ap.add_argument('--depth-min-m', type=float, default=0.30,
                    help='Minimum accepted depth; visual mode uses 0.10 for a 0.20m target')
    ap.add_argument('--depth-max-m', type=float, default=8.0)
    ap.add_argument('--fx', type=float, default=570.0)
    ap.add_argument('--fy', type=float, default=570.0)
    ap.add_argument('--cx', type=float, default=319.5)
    ap.add_argument('--cy', type=float, default=239.5)
    args, _ = ap.parse_known_args()
    if args.cv_threads < 1:
        ap.error('--cv-threads must be at least 1')
    if not 0 < args.depth_min_m < args.depth_max_m:
        ap.error('depth range must satisfy 0 < min < max')
    if args.annotation_fps <= 0 or args.max_fps <= 0 or args.max_depth_skew_ms <= 0:
        ap.error('frame rates and depth matching limit must be positive')
    if args.input_size < 32 or (args.input_height is not None and args.input_height < 32):
        ap.error('model dimensions must be at least 32')
    if not 0 <= args.support_conf <= args.conf <= 1 or args.support_seconds < 0:
        ap.error('confidence thresholds must satisfy 0 <= support <= conf <= 1; support time >= 0')

    rclpy.init()
    node = PersonDetector(args)
    node.start_worker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_worker()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
