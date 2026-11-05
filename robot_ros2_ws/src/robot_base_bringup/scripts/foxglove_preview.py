#!/usr/bin/env python3
"""Small viewer-only images; never changes sensor or detector input topics.

Only foxglove_bridge remaps its image subscriptions to these outputs. Existing
Foxglove layouts keep their topic names; ROS consumers still receive originals.
"""
import argparse
from collections import deque
from dataclasses import dataclass
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Float32, String


class FrameRate:
    """Rate of completed local deliveries over a 5s monotonic time window."""
    def __init__(self, window=5.0):
        self.window = window
        self.times = deque()

    def observe(self, now=None):
        now = time.monotonic() if now is None else now
        self.times.append(now)
        self.prune(now)

    def prune(self, now):
        while self.times and self.times[0] < now-self.window:
            self.times.popleft()

    def value(self, now):
        self.prune(now)
        if len(self.times) < 2 or now-self.times[-1] > 3.0:
            return 0.0
        elapsed = self.times[-1]-self.times[0]
        return (len(self.times)-1)/elapsed if elapsed > 0 else 0.0


@dataclass
class PreviewStream:
    source: str
    destination: str
    kind: type
    fps: float
    label: str = ''
    publisher: object = None
    latest: object = None
    sequence: int = 0
    published_sequence: int = -1
    next_publish: float = 0.0


class FoxglovePreview(Node):
    def __init__(self, args):
        super().__init__('foxglove_preview')
        self.args = args
        cv2.setNumThreads(1)
        self.streams = [
            PreviewStream('/camera/color/image_raw/compressed', '/foxglove_preview/color/compressed', CompressedImage, args.fps, 'color'),
            PreviewStream('/camera/depth/image_raw', '/foxglove_preview/depth/image_raw', Image, args.fps, 'depth'),
            PreviewStream('/perception/person_image/compressed', '/foxglove_preview/person/compressed', CompressedImage, args.fps, 'person'),
            PreviewStream('/perception/hand_image/compressed', '/foxglove_preview/hand/compressed', CompressedImage, args.fps, 'hand'),
        ]
        self.rates = {name: FrameRate() for name in (
            'local_color', 'local_depth',
            'person_detection', 'preview_color', 'preview_depth',
            'preview_person', 'preview_hand')}
        self.metric_publishers = {
            name: self.create_publisher(Float32, '/performance/fps/'+name, 1)
            for name in self.rates}
        self.inference_publishers = {
            name: self.create_publisher(Float32, '/performance/'+name, 1)
            for name in ('person_infer_ms', 'person_input_age_ms', 'person_result_age_ms')}
        self.last_detection = None
        self.last_detection_at = None
        self.detection_input = None
        self.metrics_watched = False
        self.inputs = {}
        for stream in self.streams:
            # Reliable publisher also supports best-effort viewers.
            stream.publisher = self.create_publisher(stream.kind, stream.destination, 1)
        self.info_publisher = self.create_publisher(CameraInfo, '/foxglove_preview/depth/camera_info', 1)
        self.depth_info = None
        self.info_input = None
        self.create_timer(.1, self.refresh_inputs)
        self.create_timer(.05, self.publish_previews)
        self.create_timer(1.0, self.publish_frame_rates)
        self.get_logger().info(
            f'viewer previews ready: max_width={args.width} fps={args.fps:g} '
            f'jpeg_quality={args.quality}; local inputs unchanged')

    def refresh_inputs(self):
        # Statistics reuse lightweight callbacks, not extra video encoders.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for stream in self.streams:
            wanted = stream.publisher.get_subscription_count() > 0
            wanted = wanted or (self.metrics_watched and stream.label in ('color', 'depth'))
            if wanted and stream.label not in self.inputs:
                self.inputs[stream.label] = self.create_subscription(
                    stream.kind, stream.source,
                    lambda message, target=stream: self.on_image(target, message), qos)
            elif not wanted and stream.label in self.inputs:
                self.destroy_subscription(self.inputs.pop(stream.label))
                stream.latest = None
        info_wanted = self.info_publisher.get_subscription_count() > 0
        if info_wanted and self.info_input is None:
            self.info_input = self.create_subscription(
                CameraInfo, '/camera/depth/camera_info', self.on_info, qos)
        elif not info_wanted and self.info_input is not None:
            self.destroy_subscription(self.info_input)
            self.info_input = None
        if self.metrics_watched and self.detection_input is None:
            self.detection_input = self.create_subscription(
                String, '/perception/person_detections', self.on_detection, qos)
        elif not self.metrics_watched and self.detection_input is not None:
            self.destroy_subscription(self.detection_input)
            self.detection_input = None

    def on_image(self, stream, message):
        stream.latest = message
        stream.sequence += 1
        if 'local_'+stream.label in self.rates:
            self.rates['local_'+stream.label].observe()

    def on_detection(self, message):
        # Every result counts, including count=0 (no person in the frame).
        self.rates['person_detection'].observe()
        try:
            self.last_detection = json.loads(message.data)
            self.last_detection_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def publish_frame_rates(self):
        self.metrics_watched = any(
            publisher.get_subscription_count()
            for publisher in (*self.metric_publishers.values(), *self.inference_publishers.values()))
        if not self.metrics_watched:
            return
        now = time.monotonic()
        for name, publisher in self.metric_publishers.items():
            publisher.publish(Float32(data=self.rates[name].value(now)))
        for name, publisher in self.inference_publishers.items():
            field = name.removeprefix('person_')
            value = float('nan')
            if (self.last_detection is not None and self.last_detection_at is not None
                    and now-self.last_detection_at < 3.0):
                value = float(self.last_detection.get(field, float('nan')))
            publisher.publish(Float32(data=value))

    def on_info(self, message):
        self.depth_info = message

    def resize(self, image):
        height, width = image.shape[:2]
        if width <= self.args.width:
            return image
        return cv2.resize(image, (self.args.width, max(1, round(height*self.args.width/width))), interpolation=cv2.INTER_AREA)

    def publish_previews(self):
        now = time.monotonic()
        for stream in self.streams:
            if (not stream.publisher.get_subscription_count() or stream.latest is None
                    or stream.sequence == stream.published_sequence or now < stream.next_publish):
                continue
            message = stream.latest
            stream.published_sequence = stream.sequence
            stream.next_publish = now + 1.0/stream.fps
            try:
                if stream.kind is Image:
                    sent = self.publish_depth(stream, message)
                else:
                    sent = self.publish_jpeg(stream, message)
                if sent:
                    self.rates['preview_'+stream.label].observe()
            except (ValueError, TypeError, cv2.error) as error:
                self.get_logger().warning(f'preview decode failed: {stream.source}: {error}', throttle_duration_sec=5)

    def publish_jpeg(self, stream, source):
        image = cv2.imdecode(np.frombuffer(source.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        image = self.resize(image)
        success, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.args.quality])
        if not success:
            return
        preview = CompressedImage()
        preview.header = source.header
        preview.format = 'jpeg'
        preview.data = encoded.tobytes()
        stream.publisher.publish(preview)
        return True

    def publish_depth(self, stream, source):
        if source.encoding not in ('16UC1', 'mono16'):
            return
        dtype = np.dtype('>u2' if source.is_bigendian else '<u2')
        depth = np.ndarray((source.height, source.width), dtype=dtype,
                           buffer=source.data, strides=(source.step, 2))
        # Preserve actual millimetre values in the viewer too. Sampling by stride
        # avoids inventing interpolated ranges across foreground/background edges.
        stride = max(1, int(np.ceil(source.width/self.args.width)))
        small = np.ascontiguousarray(depth[::stride, ::stride], dtype='<u2')
        preview = Image()
        preview.header = source.header
        preview.height, preview.width = small.shape
        preview.encoding = '16UC1'
        preview.is_bigendian = 0
        preview.step = preview.width*2
        preview.data = small.tobytes()
        stream.publisher.publish(preview)
        if self.depth_info is not None and self.info_publisher.get_subscription_count():
            info = CameraInfo()
            info.header = source.header
            info.height, info.width = preview.height, preview.width
            info.distortion_model = self.depth_info.distortion_model
            info.d = list(self.depth_info.d)
            scale = 1.0/stride
            info.k = [float(v)*scale if i < 6 else float(v) for i,v in enumerate(self.depth_info.k)]
            info.r = list(self.depth_info.r)
            info.p = [float(v)*scale if i < 8 else float(v) for i,v in enumerate(self.depth_info.p)]
            self.info_publisher.publish(info)
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--width', type=int, default=240)
    parser.add_argument('--fps', type=float, default=3)
    parser.add_argument('--quality', type=int, default=40)
    args, _ = parser.parse_known_args()
    if args.width < 1 or args.fps <= 0 or not 1 <= args.quality <= 100:
        parser.error('width/fps must be positive and quality must be 1..100')
    rclpy.init()
    node = FoxglovePreview(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
