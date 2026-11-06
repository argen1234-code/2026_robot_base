"""Standalone RGB/depth -> remote velocity controller with a small web panel."""
import argparse
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int8, String

from .visual_follow import VisualFollower, validate_config


class VisualFollowNode(Node):
    def __init__(self, args):
        super().__init__('visual_follow')
        config = json.loads(Path(args.config).read_text()) if args.config else {}
        if args.distance is not None:
            config['target_distance_m'] = args.distance
        self.controller = VisualFollower(config)
        self.execute = args.execute
        self.packet, self.received_at, self.image = {}, 0., b''
        self.snapshot, self.commands = {}, queue.Queue()
        self.velocity_pub = self.create_publisher(Twist, '/remote_cmd_vel', 1)
        self.mode_pub = self.create_publisher(Int8, '/robot_mode', 1)
        self.status_pub = self.create_publisher(String, '/visual_follow/status', 1)
        self.create_subscription(String, '/perception/person_detections', self.on_detection, 1)
        self.create_subscription(CompressedImage, '/perception/person_image/compressed',
            self.on_image, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Bool, '/stop_request', self.on_stop, 1)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 1)
        self.create_subscription(Int8, '/robot_mode', self.on_mode, 1)
        self.create_timer(.05, self.tick)
        self.http = None
        self.http_thread = None
        if args.http_port:
            self.http = ThreadingHTTPServer((args.http_host, args.http_port),
                                            partial(WebHandler, node=self))
            self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
            self.http_thread.start()
        if args.auto:
            self.controller.start(args.follow)
        self.get_logger().info(
            f'visual follow ready: execute={self.execute} distance='
            f'{self.controller.cfg["target_distance_m"]:.2f}m http={args.http_port}')

    def on_detection(self, message):
        try:
            def reject_constant(value):
                raise ValueError(f'Nonfinite JSON value: {value}')
            packet = json.loads(message.data, parse_constant=reject_constant)
            if not isinstance(packet, dict):
                raise ValueError('Detection must be an object')
            self.packet, self.received_at = packet, time.monotonic()
        except (ValueError, TypeError):
            self.packet, self.received_at = {}, 0.
        # Evaluate each result in its callback so a single arrival sample
        # stops motion before the next 50ms timer and cannot be overwritten.
        if self.controller.active:
            self.update_control()

    def on_image(self, message):
        self.image = bytes(message.data)

    def on_stop(self, message):
        if message.data:
            self.halt('stopped')

    def on_goal(self, message):
        self.halt('manual_override')

    def on_mode(self, message):
        if self.execute and message.data != 2:
            self.halt('mode_changed')

    def publish_velocity(self, linear=0., angular=0.):
        if self.execute and rclpy.ok(context=self.context):
            message = Twist()
            message.linear.x, message.angular.z = float(linear), float(angular)
            self.velocity_pub.publish(message)

    def halt(self, reason='stopped'):
        # Discard web starts queued before a global stop/manual override.
        while not self.commands.empty():
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break
        self.controller.stop(reason)
        self.publish_velocity()

    def tick(self):
        while not self.commands.empty():
            action, fields = self.commands.get_nowait()
            if action == 'stop':
                self.controller.stop()
            else:
                self.publish_velocity()
                self.controller.cfg = validate_config(dict(self.controller.cfg, **fields))
                self.controller.start(follow=(action == 'follow'))
        self.update_control()

    def update_control(self):
        result = self.controller.step(self.packet, time.monotonic()-self.received_at, time.time())
        if self.execute:
            # Mode 2 is the existing direct USB velocity path. Never use the
            # navigation velocity topic or person_nav centering overlay.
            if self.controller.active:
                self.mode_pub.publish(Int8(data=2))
            self.publish_velocity(result['linear_mps'], result['angular_rps'])
        result.update(execute=self.execute, mode='visual', timestamp=time.time(),
                      distance_reference='camera_depth_optical_z',
                      output_linear_mps=result['linear_mps'] if self.execute else 0.,
                      output_angular_rps=result['angular_rps'] if self.execute else 0.,
                      detection_age_ms=round((time.monotonic()-self.received_at)*1000)
                      if self.received_at else None,
                      image_size=self.packet.get('image_size', [640, 480]),
                      persons=self.packet.get('persons', []))
        self.snapshot = result
        self.status_pub.publish(String(data=json.dumps(result, allow_nan=False)))

    def close(self):
        self.halt()
        if self.http is not None:
            self.http.shutdown()
            self.http.server_close()
            self.http_thread.join(timeout=2.)


class WebHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, node, **kwargs):
        self.node = node
        super().__init__(*args, **kwargs)

    def log_message(self, *args):
        pass

    def send(self, status, body, kind='application/json'):
        self.send_response(status)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/':
            self.send(200, Path(__file__).with_name('visual_follow.html').read_bytes(),
                      'text/html; charset=utf-8')
        elif path == '/status':
            self.send(200, json.dumps(self.node.snapshot, allow_nan=False).encode())
        elif path == '/image.jpg':
            self.send(200 if self.node.image else 503, self.node.image, 'image/jpeg')
        else:
            self.send(404, b'{}')

    def do_POST(self):
        action = self.path.lstrip('/')
        if action not in ('start', 'follow', 'stop'):
            self.send(404, b'{}')
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 <= length <= 1024:
                raise ValueError('Invalid request size')
            fields = json.loads(self.rfile.read(length) or b'{}')
            if not isinstance(fields, dict) or set(fields)-{'target_distance_m'}:
                raise ValueError('Only target_distance_m is accepted')
            validate_config(dict(self.node.controller.cfg, **fields))
            self.node.commands.put((action, fields))
            self.send(202, b'{"accepted":true}')
        except (ValueError, TypeError) as exc:
            self.send(400, json.dumps({'error': str(exc)}).encode())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='')
    parser.add_argument('--distance', type=float)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--auto', action='store_true')
    parser.add_argument('--follow', action='store_true')
    parser.add_argument('--http-host', default='0.0.0.0')
    parser.add_argument('--http-port', type=int, default=8879)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = VisualFollowNode(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
