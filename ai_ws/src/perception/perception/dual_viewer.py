#!/usr/bin/env python3
"""
Single-page dual-camera viewer on port 80.

Left:  USB hand camera (MediaPipe hand detection + Cartesian estimate)
Right: target/depth camera (ring detector debug image, or raw depth,
       or a placeholder until the Orbbec camera is connected)

HTTP endpoints:
    /                    HTML page
    /stream/hand         MJPEG hand feed
    /stream/target       MJPEG target/depth feed
    /snapshot/hand       single JPEG
    /snapshot/target     single JPEG
    /status/hand         last hand debug JSON
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class FrameStore:
    def __init__(self, placeholder_text):
        self._lock = threading.Lock()
        self._jpeg = None
        self._last_update = 0.0
        self._placeholder_text = placeholder_text
        self._placeholder_jpeg = None

    def update(self, frame):
        ok, jpeg = cv2.imencode('.jpg', frame,
                                [cv2.IMWRITE_JPEG_QUALITY, 80])
        with self._lock:
            if ok:
                self._jpeg = jpeg.tobytes()
                self._last_update = time.time()

    def age(self):
        with self._lock:
            if self._jpeg is None:
                return 1e9
            return time.time() - self._last_update

    def snapshot(self):
        with self._lock:
            if self._jpeg is not None:
                return self._jpeg
        return self.placeholder()

    def placeholder(self):
        with self._lock:
            if self._placeholder_jpeg is not None:
                return self._placeholder_jpeg
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(img, self._placeholder_text, (30, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (180, 180, 180), 2)
            ok, jpeg = cv2.imencode('.jpg', img)
            self._placeholder_jpeg = jpeg.tobytes() if ok else b''
            return self._placeholder_jpeg


class DualStore:
    def __init__(self):
        self.hand = FrameStore('WAITING FOR HAND CAMERA')
        self.debug = FrameStore('TARGET CAMERA NOT CONNECTED')
        self.depth = FrameStore('DEPTH CAMERA NOT CONNECTED')
        self.hand_status = ''

    def target_jpeg(self):
        # Prefer the annotated ring-detector image; otherwise show depth.
        if self.debug.age() < 2.0:
            return self.debug.snapshot()
        if self.depth.age() < 2.0:
            return self.depth.snapshot()
        return self.debug.placeholder()


class ViewerHandler(BaseHTTPRequestHandler):
    store = None
    shutdown_event = None

    def log_message(self, fmt, *args):
        return

    def _html(self):
        body = (
            b'<!doctype html><html><head><meta charset="utf-8">'
            b'<title>Robot Cameras</title>'
            b'<style>'
            b'body{margin:0;padding:12px;background:#111;color:#eee;'
            b'font-family:Arial,sans-serif}'
            b'.row{display:flex;flex-wrap:wrap;gap:12px}'
            b'.card{flex:1;min-width:320px;background:#1b1b1b;'
            b'border-radius:8px;padding:8px}'
            b'img{width:100%;height:auto;background:#000;border-radius:4px}'
            b'pre{white-space:pre-wrap;color:#8f8;font-size:13px}'
            b'</style></head><body>'
            b'<h2>Robot Cameras</h2>'
            b'<div class="row">'
            b'<div class="card"><h3>USB Hand Camera</h3>'
            b'<img src="/stream/hand"></div>'
            b'<div class="card"><h3>Target / Depth Camera</h3>'
            b'<img src="/stream/target"></div>'
            b'</div>'
            b'<h3>Hand status</h3><pre id="hand_status">waiting...</pre>'
            b'<script>setInterval(function(){'
            b'fetch("/status/hand").then(r=>r.text()).then(t=>{'
            b'document.getElementById("hand_status").textContent=t;});'
            b'},500);</script>'
            b'</body></html>'
        )
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _snapshot(self, jpeg):
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(jpeg)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(jpeg)

    def _stream(self, get_jpeg):
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        try:
            while not self.shutdown_event.is_set():
                jpeg = get_jpeg()
                if jpeg:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(
                        ('Content-Length: %d\r\n\r\n' % len(jpeg)).encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
                time.sleep(1.0 / 15.0)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/':
            self._html()
        elif path == '/stream/hand':
            self._stream(self.store.hand.snapshot)
        elif path == '/stream/target':
            self._stream(self.store.target_jpeg)
        elif path == '/snapshot/hand':
            self._snapshot(self.store.hand.snapshot())
        elif path == '/snapshot/target':
            self._snapshot(self.store.target_jpeg())
        elif path == '/status/hand':
            body = (self.store.hand_status or '{}').encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()


class DualViewer(Node):
    def __init__(self):
        super().__init__('dual_viewer')

        self.declare_parameter('hand_image_topic', '/perception/hand_image')
        self.declare_parameter('hand_status_topic', '/perception/hand_debug')
        self.declare_parameter('target_image_topic',
                               '/perception/debug_image')
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('http_port', 80)

        hand_image_topic = self.get_parameter('hand_image_topic').value
        hand_status_topic = self.get_parameter('hand_status_topic').value
        target_image_topic = self.get_parameter('target_image_topic').value
        depth_image_topic = self.get_parameter('depth_image_topic').value
        self.http_port = int(self.get_parameter('http_port').value)

        self.store = DualStore()
        self.bridge = CvBridge()
        self.shutdown_event = threading.Event()

        self.create_subscription(Image, hand_image_topic,
                                 self._hand_image_cb, 10)
        self.create_subscription(String, hand_status_topic,
                                 self._hand_status_cb, 10)
        self.create_subscription(Image, target_image_topic,
                                 self._target_image_cb, 10)
        self.create_subscription(Image, depth_image_topic,
                                 self._depth_cb, 10)

        ViewerHandler.store = self.store
        ViewerHandler.shutdown_event = self.shutdown_event
        self.server = ThreadingHTTPServer(('0.0.0.0', self.http_port),
                                          ViewerHandler)
        self.thread = threading.Thread(
            target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'Dual viewer: http://0.0.0.0:{self.http_port}/ '
            f'(hand + target/depth)')

    def _hand_image_cb(self, msg):
        try:
            self.store.hand.update(
                self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        except Exception as exc:
            self.get_logger().warning(f'hand image: {exc}')

    def _hand_status_cb(self, msg):
        self.store.hand_status = msg.data

    def _target_image_cb(self, msg):
        try:
            self.store.debug.update(
                self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        except Exception as exc:
            self.get_logger().warning(f'target image: {exc}')

    def _depth_cb(self, msg):
        try:
            depth = np.asarray(
                self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))
            valid = depth[np.isfinite(depth) & (depth > 0)]
            if valid.size:
                lo, hi = np.percentile(valid, [5, 95])
                if hi <= lo:
                    hi = lo + 1
                vis = np.clip((depth.astype(np.float32) - lo) /
                              (hi - lo) * 255, 0, 255).astype(np.uint8)
            else:
                vis = np.zeros(depth.shape, np.uint8)
            color = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
            cv2.putText(color, 'DEPTH', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            self.store.depth.update(color)
        except Exception as exc:
            self.get_logger().warning(f'depth image: {exc}')

    def destroy_node(self):
        self.shutdown_event.set()
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualViewer()
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
