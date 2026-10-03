#!/usr/bin/env python3
"""Fail-closed ROS Twist to STM32 USB CDC bridge."""

import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int8
import serial

from chassis_protocol import speed_frame


class ChassisBridge(Node):
    def __init__(self):
        super().__init__('chassis_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('enabled', False)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('max_linear', 0.20)
        self.declare_parameter('max_angular', 0.45)
        self.port = self.get_parameter('port').value
        self.timeout = self.get_parameter('command_timeout').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        if (os.path.realpath(self.port) == os.path.realpath('/dev/ydlidar') or
                self.timeout <= 0 or self.timeout >= 0.5):
            raise ValueError('invalid port or timeout (must be shorter than MCU 500 ms watchdog)')
        self.enabled = self.get_parameter('enabled').value
        self.device = None
        self.last_command = None
        self.command = (0.0, 0.0)
        self.remote_command = (0.0, 0.0)
        self.last_remote = None
        self.mode = 3
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
        self.create_subscription(Twist, '/remote_cmd_vel', self.on_remote, 10)
        self.create_subscription(Int8, '/robot_mode', self.on_mode, 10)
        self.mode_pub = self.create_publisher(Int8, '/robot_mode', 10)
        self.mode_pub.publish(Int8(data=self.mode))
        self.create_timer(0.05, self.tick)
        if not self.enabled:
            self.get_logger().warn('Motor output disabled; set motor_enable:=true explicitly')

    def on_command(self, msg):
        if (abs(msg.linear.y) > 1e-6 or abs(msg.linear.z) > 1e-6 or
                abs(msg.angular.x) > 1e-6 or abs(msg.angular.y) > 1e-6):
            self.last_command = None
            self.get_logger().warn('Unsupported velocity axis; stopping')
            return
        try:
            speed_frame(msg.linear.x, msg.angular.z, self.max_linear, self.max_angular)
        except ValueError:
            self.last_command = None
            self.get_logger().warn('Invalid/reverse velocity; stopping')
            return
        self.command = (msg.linear.x, msg.angular.z)
        self.last_command = time.monotonic()

    def on_mode(self, msg):
        if msg.data not in (2, 3):
            self.get_logger().warn('Unsupported mode; stopping')
            self.mode = 2
        elif msg.data != self.mode:
            self.mode = msg.data
        else:
            return
        self.last_command = None
        self.last_remote = None

    def on_remote(self, msg):
        axes = (msg.linear.x, msg.angular.z, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y)
        if not all(math.isfinite(v) for v in axes) or any(abs(v) > 1e-6 for v in axes[2:]):
            self.last_remote = None
            self.get_logger().warn('Invalid remote velocity; stopping')
            return
        self.remote_command = (msg.linear.x, msg.angular.z)
        self.last_remote = time.monotonic()

    def tick(self):
        if not self.enabled:
            return
        if self.device is None:
            try:
                # The MCU enumerates as USB CDC (ttyACM), not the lidar's CP2102 ttyUSB.
                self.device = serial.Serial(self.port, baudrate=115200, timeout=0,
                                            write_timeout=0.1, exclusive=True)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'Cannot open MCU {self.port}: {exc}', throttle_duration_sec=5.0)
                return
        if self.mode == 2:
            fresh = self.last_remote is not None and time.monotonic() - self.last_remote < self.timeout
            linear, angular = self.remote_command if fresh else (0.0, 0.0)
            # Firmware remote mode uses the verified WeChat turn sign.
            angular = -angular
        else:
            fresh = self.last_command is not None and time.monotonic() - self.last_command < self.timeout
            linear, angular = self.command if fresh else (0.0, 0.0)
        try:
            self.device.write(speed_frame(linear, angular, self.max_linear, self.max_angular,
                                          mode=self.mode))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'MCU write failed: {exc}')
            self.device.close()
            self.device = None
            self.last_command = None

    def stop(self):
        if self.device is not None:
            try:
                self.device.write(speed_frame(0.0, 0.0, mode=self.mode))
                self.device.flush()
            except (serial.SerialException, OSError):
                pass
            self.device.close()


def main():
    rclpy.init()
    node = ChassisBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
