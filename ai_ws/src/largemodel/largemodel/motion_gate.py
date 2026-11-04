"""
Velocity gate between Nav2 and the mobile base.

Safety properties:

* Nav2 writes to ``cmd_vel_nav``; only this node publishes ``cmd_vel``.
* When disabled (stopped, no goal, stale command), it continuously publishes
  zero velocity instead of relying on Nav2's asynchronous cancel.
* A new goal cannot re-enable motion by accident: ``nav_task`` explicitly
  publishes ``/nav_enable`` only after validating a goal.
* Optional front-sector LiDAR guard stops the base before a crash.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class MotionGate(Node):
    def __init__(self):
        super().__init__('motion_gate')

        self.declare_parameter('input_topic', 'cmd_vel_nav')
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('stop_topic', 'stop_request')
        self.declare_parameter('enable_topic', 'nav_enable')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('lidar_safety', False)
        self.declare_parameter('require_scan', False)
        self.declare_parameter('scan_timeout_s', 1.0)
        self.declare_parameter('safety_sector_deg', 50.0)
        self.declare_parameter('safety_center_angle_rad', 0.0)
        self.declare_parameter('safety_distance_m', 0.22)
        self.declare_parameter('min_valid_range_m', 0.12)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        stop_topic = self.get_parameter('stop_topic').value
        enable_topic = self.get_parameter('enable_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)
        publish_rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.lidar_safety = bool(self.get_parameter('lidar_safety').value)
        self.require_scan = bool(self.get_parameter('require_scan').value)
        self.scan_timeout_s = float(self.get_parameter('scan_timeout_s').value)
        self.safety_sector = math.radians(
            float(self.get_parameter('safety_sector_deg').value)
        )
        self.safety_center = float(
            self.get_parameter('safety_center_angle_rad').value
        )
        self.safety_distance = float(
            self.get_parameter('safety_distance_m').value
        )
        self.min_valid_range = float(
            self.get_parameter('min_valid_range_m').value
        )

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.cmd_callback, 10)
        self.create_subscription(Bool, stop_topic, self.stop_callback, 10)
        self.create_subscription(Bool, enable_topic, self.enable_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        self.enabled = False
        self.stop_latched = True
        self.last_cmd = Twist()
        self.last_cmd_ns = 0
        self.latest_scan = None

        self._last_unsafe_log_ns = 0
        self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(
            f"MotionGate {input_topic} -> {output_topic}, "
            f"lidar_safety={self.lidar_safety}, disabled at startup"
        )

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_ns = self.get_clock().now().nanoseconds

    def enable_callback(self, msg: Bool):
        if msg.data:
            # Only a validated new task enables motion.
            self.enabled = True
            self.stop_latched = False
            self.get_logger().info("Motion enabled by nav_task")
        else:
            self.enabled = False
            self.stop_latched = True
            self.pub.publish(Twist())
            self.get_logger().info("Motion disabled")

    def stop_callback(self, msg: Bool):
        if not msg.data:
            return
        self.enabled = False
        self.stop_latched = True
        self.pub.publish(Twist())
        self.get_logger().warn("STOP latched: zero velocity published")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def _scan_is_fresh(self) -> bool:
        if self.latest_scan is None:
            return False
        try:
            stamp = self.latest_scan.header.stamp
            scan_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            age = (self.get_clock().now().nanoseconds - scan_ns) / 1e9
            return age <= self.scan_timeout_s
        except Exception:
            return False

    def _lidar_safe(self) -> bool:
        if not self.lidar_safety:
            return True
        if not self._scan_is_fresh():
            return not self.require_scan

        scan = self.latest_scan
        half = self.safety_sector * 0.5
        min_range = float('inf')

        try:
            for i, value in enumerate(scan.ranges):
                try:
                    r = float(value)
                except (TypeError, ValueError):
                    continue
                if not math.isfinite(r) or r < self.min_valid_range:
                    continue
                angle = scan.angle_min + i * scan.angle_increment
                offset = angle - self.safety_center
                if abs(math.atan2(math.sin(offset), math.cos(offset))) <= half:
                    min_range = min(min_range, r)
        except Exception:
            return not self.require_scan

        if min_range < self.safety_distance:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_unsafe_log_ns > 1_000_000_000:
                self._last_unsafe_log_ns = now_ns
                self.get_logger().warn(
                    f"Obstacle at {min_range:.2f} m in front sector; "
                    "holding zero velocity"
                )
            return False

        return True

    def _cmd_is_fresh(self) -> bool:
        if self.last_cmd_ns == 0:
            return False
        age = (self.get_clock().now().nanoseconds - self.last_cmd_ns) / 1e9
        return age <= self.cmd_timeout_s

    def timer_callback(self):
        if self.enabled and self._cmd_is_fresh() and self._lidar_safe():
            self.pub.publish(self.last_cmd)
        else:
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MotionGate()
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
