#!/usr/bin/env python3
"""
Temporary target viewer.

Subscribes to the annotated debug image published by ring_detector_node and
serves it as an MJPEG stream, so it can be opened in a browser without
depending on the container's Wayland/X11 setup:

    http://<pi-ip>:8088/
    http://<pi-ip>:8088/snapshot   (single JPEG)

An optional native OpenCV window can be enabled with -p use_gui:=true.
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
    def __init__(self):
        self._lock = threading.Lock()
        self._jpeg = None
        self._status = ''
        self._frame = None

    def update_frame(self, frame):
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with self._lock:
            self._frame = frame
            if ok:
                self._jpeg = jpeg.tobytes()

    def update_status(self, status):
        with self._lock:
            self._status = status

    def snapshot(self):
        with self._lock:
            return self._jpeg

    def frame(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def status(self):
        with self._lock:
            return self._status


class ViewerHandler(BaseHTTPRequestHandler):
    store = None
    shutdown_event = None

    def log_message(self, fmt, *args):
        return

    def _send_html(self):
        body = (b'<!doctype html><html><head><meta charset="utf-8">'
                b'<title>Ring Target Viewer</title></head>'
                b'<body style="margin:0;background:#111;color:#eee;'
                b'font-family:sans-serif">'
                b'<h3 style="margin:8px 12px">Ring Target Viewer</h3>'
                b'<img src="/stream" style="max-width:100%;height:auto">'
                b'</body></html>')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_snapshot(self):
        jpeg = self.store.snapshot()
        if jpeg is None:
            self.send_response(503)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(jpeg)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(jpeg)

    def _send_stream(self):
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        try:
            while not self.shutdown_event.is_set():
                jpeg = self.store.snapshot()
                if jpeg is not None:
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
            self._send_html()
        elif path == '/snapshot':
            self._send_snapshot()
        elif path == '/stream':
            self._send_stream()
        else:
            self.send_response(404)
            self.end_headers()


class TargetViewer(Node):
    def __init__(self):
        super().__init__('target_viewer')

        self.declare_parameter('image_topic', '/perception/debug_image')
        self.declare_parameter('status_topic', '/perception/target_debug')
        self.declare_parameter('http_port', 8088)
        self.declare_parameter('use_gui', False)
        self.declare_parameter('window_name', 'Ring Target Viewer')

        image_topic = self.get_parameter('image_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.http_port = int(self.get_parameter('http_port').value)
        self.use_gui = bool(self.get_parameter('use_gui').value)
        self.window_name = self.get_parameter('window_name').value

        self.store = FrameStore()
        self.bridge = CvBridge()
        self.last_status = ''
        self.shutdown_event = threading.Event()
        self.server = None

        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.create_subscription(String, status_topic, self._status_cb, 10)

        ViewerHandler.store = self.store
        ViewerHandler.shutdown_event = self.shutdown_event
        self.server = ThreadingHTTPServer(('0.0.0.0', self.http_port),
                                          ViewerHandler)
        self.http_thread = threading.Thread(
            target=self.server.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(
            f'Target viewer: http://0.0.0.0:{self.http_port}/ '
            f'(snapshot: /snapshot)')

        if self.use_gui:
            self.create_timer(1.0 / 30.0, self._gui_tick)
            self.get_logger().info('Native OpenCV window requested')

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Cannot convert debug image: {exc}')
            return
        self.store.update_frame(frame)

    def _status_cb(self, msg):
        self.last_status = msg.data
        self.store.update_status(msg.data)

    def _gui_tick(self):
        frame = self.store.frame()
        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, 'waiting for /perception/debug_image',
                        (30, 240), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (200, 200, 200), 2)
        if self.last_status:
            cv2.putText(frame, self.last_status[:110], (10, 465),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
        try:
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                rclpy.shutdown()
        except Exception as exc:
            self.get_logger().error(
                f'OpenCV GUI failed ({exc}); use the browser viewer instead')
            self.use_gui = False

    def destroy_node(self):
        self.shutdown_event.set()
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TargetViewer()
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
