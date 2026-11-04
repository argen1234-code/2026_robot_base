#!/usr/bin/env python3
"""
Search state machine: rotate in place until the A4 two-ring target is seen.

States:
    IDLE -> SEARCHING -> FOUND
                       -> SEARCH_TIMEOUT
    any  -> STOPPED

The node publishes a slow angular velocity on /cmd_vel_search.  The
velocity_mux forwards it to /cmd_vel only when the task mode is "search"
and the LiDAR safety check passes.
"""

import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String


class TargetSearcher(Node):
    def __init__(self):
        super().__init__('target_searcher')

        self.declare_parameter('search_speed_rad_s', 0.35)
        self.declare_parameter('search_direction', 1.0)
        self.declare_parameter('search_timeout_s', 20.0)
        self.declare_parameter('valid_frames_required', 3)
        self.declare_parameter('approach_enabled', True)
        self.declare_parameter('base_standoff_m', 1.0)
        self.declare_parameter('camera_forward_offset_m', 0.12)
        self.declare_parameter('camera_height_m', 0.50)
        self.declare_parameter('target_height_m', 1.20)
        self.declare_parameter('bearing_tolerance_rad', 0.08)
        self.declare_parameter('max_linear_m_s', 0.15)
        self.declare_parameter('max_angular_rad_s', 0.40)
        self.declare_parameter('linear_gain', 0.60)
        self.declare_parameter('angular_gain', 1.00)
        self.declare_parameter('target_lost_timeout_s', 0.60)
        self.declare_parameter('command_topic', 'task_command')
        self.declare_parameter('summon_topic', 'summon_request')
        self.declare_parameter('stop_topic', 'stop_request')
        self.declare_parameter('mode_topic', 'task_mode')
        self.declare_parameter('search_cmd_topic', 'cmd_vel_search')
        self.declare_parameter('target_debug_topic', '/perception/target_debug')
        self.declare_parameter('target_pose_topic', '/perception/target_pose')
        self.declare_parameter('state_topic', 'task_state')

        self.search_speed = float(
            self.get_parameter('search_speed_rad_s').value)
        self.search_direction = float(
            self.get_parameter('search_direction').value)
        self.search_timeout_s = float(
            self.get_parameter('search_timeout_s').value)
        self.valid_frames_required = max(
            1, int(self.get_parameter('valid_frames_required').value))
        self.approach_enabled = bool(
            self.get_parameter('approach_enabled').value)
        self.base_standoff = float(
            self.get_parameter('base_standoff_m').value)
        self.camera_forward_offset = float(
            self.get_parameter('camera_forward_offset_m').value)
        self.camera_height = float(
            self.get_parameter('camera_height_m').value)
        self.target_height = float(
            self.get_parameter('target_height_m').value)
        self.bearing_tolerance = float(
            self.get_parameter('bearing_tolerance_rad').value)
        self.max_linear = float(
            self.get_parameter('max_linear_m_s').value)
        self.max_angular = float(
            self.get_parameter('max_angular_rad_s').value)
        self.linear_gain = float(
            self.get_parameter('linear_gain').value)
        self.angular_gain = float(
            self.get_parameter('angular_gain').value)
        self.target_lost_timeout = float(
            self.get_parameter('target_lost_timeout_s').value)

        # The camera is forward of the base centre; the desired camera-to-target
        # distance is the base standoff minus the offset, combined with the
        # height difference between the chest target and the camera.
        horizontal_camera = max(
            0.3, self.base_standoff - self.camera_forward_offset)
        height_diff = self.target_height - self.camera_height
        self.stop_distance_camera = math.hypot(horizontal_camera, height_diff)

        command_topic = self.get_parameter('command_topic').value
        summon_topic = self.get_parameter('summon_topic').value
        stop_topic = self.get_parameter('stop_topic').value
        mode_topic = self.get_parameter('mode_topic').value
        search_cmd_topic = self.get_parameter('search_cmd_topic').value
        target_debug_topic = self.get_parameter('target_debug_topic').value
        target_pose_topic = self.get_parameter('target_pose_topic').value
        state_topic = self.get_parameter('state_topic').value

        self.mode_pub = self.create_publisher(String, mode_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, search_cmd_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)

        self.create_subscription(String, command_topic, self._command_cb, 10)
        self.create_subscription(String, summon_topic, self._summon_cb, 10)
        self.create_subscription(Bool, stop_topic, self._stop_cb, 10)
        self.create_subscription(
            String, target_debug_topic, self._target_debug_cb, 10)
        self.create_subscription(
            PoseStamped, target_pose_topic, self._target_pose_cb, 10)

        self.state = 'IDLE'
        self.valid_count = 0
        self.search_start_ns = 0
        self.approach_start_ns = 0
        self.last_state_pub_ns = 0
        self.last_approach_log_ns = 0
        self.found_pose = None
        self.target_pose = None
        self.target_pose_ns = 0

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'TargetSearcher ready: speed={self.search_speed} rad/s, '
            f'timeout={self.search_timeout_s}s, '
            f'valid_frames={self.valid_frames_required}')

    def _command_cb(self, msg):
        command = (msg.data or '').strip().lower()
        if command in ('summon', 'search', 'start'):
            self._start()
        elif command in ('stop', 'cancel', 'halt'):
            self._stop('STOPPED')

    def _summon_cb(self, msg):
        # model_service publishes this on the fast "过来" intent.
        self._start()

    def _stop_cb(self, msg):
        if msg.data:
            self._stop('STOPPED')

    def _target_debug_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            data = {}

        valid = bool(data.get('valid', False))
        if valid:
            self.valid_count += 1
        else:
            self.valid_count = 0

        if (valid and self.state == 'SEARCHING'
                and self.valid_count >= self.valid_frames_required):
            self._found()

    def _target_pose_cb(self, msg):
        self.found_pose = msg
        self.target_pose = msg
        self.target_pose_ns = self.get_clock().now().nanoseconds

    def _start(self):
        if self.state in ('SEARCHING', 'APPROACHING'):
            return
        self.state = 'SEARCHING'
        self.valid_count = 0
        self.found_pose = None
        self.target_pose = None
        self.target_pose_ns = 0
        self.approach_start_ns = 0
        self.search_start_ns = self.get_clock().now().nanoseconds
        self.mode_pub.publish(String(data='search'))
        self._publish_state('SEARCHING')
        self.get_logger().info('Search started')

    def _found(self):
        if self.target_pose is not None:
            p = self.target_pose.pose.position
            self.get_logger().info(
                f'Target found at camera frame '
                f'({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m')
        else:
            self.get_logger().info('Target found')

        if self.approach_enabled:
            self.state = 'APPROACHING'
            self.approach_start_ns = self.get_clock().now().nanoseconds
            self.mode_pub.publish(String(data='search'))
            self._publish_state('APPROACHING')
            return

        self.state = 'FOUND'
        self._stop_motion()
        self.mode_pub.publish(String(data='idle'))
        self._publish_state('FOUND')

    def _stop(self, state):
        self.state = state
        self._stop_motion()
        self.mode_pub.publish(String(data='idle'))
        self._publish_state(state)
        self.get_logger().info(f'Search stopped: {state}')

    def _stop_motion(self):
        self.cmd_pub.publish(Twist())

    def _publish_state(self, state):
        self.state_pub.publish(String(data=state))
        self.last_state_pub_ns = self.get_clock().now().nanoseconds

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.state == 'SEARCHING':
            self._search_tick(now_ns)
        elif self.state == 'APPROACHING':
            self._approach_tick(now_ns)

    def _search_tick(self, now_ns):
        if self.search_start_ns == 0:
            self.search_start_ns = now_ns

        elapsed = (now_ns - self.search_start_ns) / 1e9
        if elapsed > self.search_timeout_s:
            self._stop('SEARCH_TIMEOUT')
            return

        cmd = Twist()
        cmd.angular.z = self.search_direction * self.search_speed
        self.cmd_pub.publish(cmd)

        if now_ns - self.last_state_pub_ns > 1_000_000_000:
            self.mode_pub.publish(String(data='search'))
            self._publish_state('SEARCHING')

    def _approach_tick(self, now_ns):
        if self.target_pose is None:
            elapsed = ((now_ns - self.approach_start_ns) / 1e9
                       if self.approach_start_ns else 0.0)
            if elapsed > self.target_lost_timeout:
                self._stop('TARGET_LOST')
            else:
                self.cmd_pub.publish(Twist())
                self.mode_pub.publish(String(data='search'))
            return

        age = (now_ns - self.target_pose_ns) / 1e9
        if age > self.target_lost_timeout:
            self._stop('TARGET_LOST')
            return

        p = self.target_pose.pose.position
        x, y, z = float(p.x), float(p.y), float(p.z)
        distance = math.sqrt(x * x + y * y + z * z)
        bearing = math.atan2(x, z)

        if distance <= self.stop_distance_camera:
            self._stop('ARRIVED')
            return

        cmd = Twist()
        cmd.angular.z = max(-self.max_angular,
                            min(self.max_angular,
                                self.angular_gain * bearing))
        if abs(bearing) > self.bearing_tolerance:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = max(
                0.0,
                min(self.max_linear,
                    self.linear_gain * (distance - self.stop_distance_camera)))

        self.cmd_pub.publish(cmd)

        if now_ns - self.last_approach_log_ns > 1_000_000_000:
            self.last_approach_log_ns = now_ns
            self.get_logger().info(
                f'Approach: distance={distance:.2f} m, '
                f'bearing={bearing:.2f} rad, '
                f'v={cmd.linear.x:.2f} m/s, w={cmd.angular.z:.2f} rad/s')

        if now_ns - self.last_state_pub_ns > 1_000_000_000:
            self.mode_pub.publish(String(data='search'))
            self._publish_state('APPROACHING')


def main(args=None):
    rclpy.init(args=args)
    node = TargetSearcher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
