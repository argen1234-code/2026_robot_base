#!/usr/bin/env python3
"""
Velocity mux / safety gate for the target-following task.

Only one of nav / search / follow may drive the base at a time.  The active
mode is selected by /task_mode (String):
    "nav"    -> /cmd_vel_nav
    "search" -> /cmd_vel_search
    "follow" -> /cmd_vel_follow
    "idle" / "stop" -> zero velocity

The node always publishes /cmd_vel at a fixed rate.  It zeroes the output on
stop, on a stale source, or when the front LiDAR sector is too close.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class VelocityMux(Node):
    def __init__(self):
        super().__init__('velocity_mux')

        self.declare_parameter('nav_topic', 'cmd_vel_nav')
        self.declare_parameter('search_topic', 'cmd_vel_search')
        self.declare_parameter('follow_topic', 'cmd_vel_follow')
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('mode_topic', 'task_mode')
        self.declare_parameter('stop_topic', 'stop_request')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_s', 0.4)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('lidar_safety', True)
        self.declare_parameter('require_scan', False)
        self.declare_parameter('scan_timeout_s', 1.0)
        self.declare_parameter('safety_sector_deg', 50.0)
        self.declare_parameter('safety_center_angle_rad', 0.0)
        self.declare_parameter('safety_distance_m', 0.25)
        self.declare_parameter('min_valid_range_m', 0.12)

        nav_topic = self.get_parameter('nav_topic').value
        search_topic = self.get_parameter('search_topic').value
        follow_topic = self.get_parameter('follow_topic').value
        output_topic = self.get_parameter('output_topic').value
        mode_topic = self.get_parameter('mode_topic').value
        stop_topic = self.get_parameter('stop_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        self.publish_rate = max(
            1.0, float(self.get_parameter('publish_rate_hz').value))
        self.cmd_timeout_s = float(
            self.get_parameter('cmd_timeout_s').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.lidar_safety = bool(self.get_parameter('lidar_safety').value)
        self.require_scan = bool(self.get_parameter('require_scan').value)
        self.scan_timeout_s = float(
            self.get_parameter('scan_timeout_s').value)
        self.safety_sector = math.radians(
            float(self.get_parameter('safety_sector_deg').value))
        self.safety_center = float(
            self.get_parameter('safety_center_angle_rad').value)
        self.safety_distance = float(
            self.get_parameter('safety_distance_m').value)
        self.min_valid_range = float(
            self.get_parameter('min_valid_range_m').value)

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(
            Twist, nav_topic, lambda m: self._twist_cb('nav', m), 10)
        self.create_subscription(
            Twist, search_topic, lambda m: self._twist_cb('search', m), 10)
        self.create_subscription(
            Twist, follow_topic, lambda m: self._twist_cb('follow', m), 10)
        self.create_subscription(String, mode_topic, self._mode_cb, 10)
        self.create_subscription(Bool, stop_topic, self._stop_cb, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)

        self.mode = 'idle'
        self.commands = {
            'nav': (None, 0),
            'search': (None, 0),
            'follow': (None, 0),
        }
        self.latest_scan = None
        self.latest_scan_ns = 0
        self._last_unsafe_log_ns = 0

        self.create_timer(1.0 / self.publish_rate, self._tick)
        self.get_logger().info(
            f'VelocityMux ready: mode={self.mode}, dry_run={self.dry_run}, '
            f'lidar_safety={self.lidar_safety}')

    def _twist_cb(self, source, msg):
        self.commands[source] = (
            msg, self.get_clock().now().nanoseconds)

    def _mode_cb(self, msg):
        mode = (msg.data or '').strip().lower()
        if mode == self.mode:
            return
        self.mode = mode
        for key in self.commands:
            self.commands[key] = (None, 0)
        self.pub.publish(Twist())
        self.get_logger().info(f'Task mode -> {mode}')

    def _stop_cb(self, msg):
        if not msg.data:
            return
        self.mode = 'stop'
        for key in self.commands:
            self.commands[key] = (None, 0)
        self.pub.publish(Twist())
        self.get_logger().warn('STOP latched: zero velocity')

    def _scan_cb(self, msg):
        self.latest_scan = msg
        self.latest_scan_ns = self.get_clock().now().nanoseconds

    def _scan_is_fresh(self):
        if self.latest_scan is None:
            return False
        age = (self.get_clock().now().nanoseconds
               - self.latest_scan_ns) / 1e9
        return age <= self.scan_timeout_s

    def _lidar_safe(self):
        if not self.lidar_safety:
            return True
        if not self._scan_is_fresh():
            return not self.require_scan

        half = self.safety_sector * 0.5
        min_range = float('inf')
        try:
            scan = self.latest_scan
            for i, value in enumerate(scan.ranges):
                try:
                    r = float(value)
                except (TypeError, ValueError):
                    continue
                if not math.isfinite(r) or r < self.min_valid_range:
                    continue
                angle = scan.angle_min + i * scan.angle_increment
                offset = angle - self.safety_center
                if abs(math.atan2(math.sin(offset),
                                  math.cos(offset))) <= half:
                    min_range = min(min_range, r)
        except Exception:
            return not self.require_scan

        if min_range < self.safety_distance:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_unsafe_log_ns > 1_000_000_000:
                self._last_unsafe_log_ns = now_ns
                self.get_logger().warn(
                    f'Obstacle at {min_range:.2f} m; holding zero velocity')
            return False
        return True

    def _tick(self):
        if self.dry_run:
            self.pub.publish(Twist())
            return

        if self.mode not in self.commands:
            self.pub.publish(Twist())
            return

        msg, stamp_ns = self.commands[self.mode]
        if msg is None:
            self.pub.publish(Twist())
            return

        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age > self.cmd_timeout_s or not self._lidar_safe():
            self.pub.publish(Twist())
            return

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
