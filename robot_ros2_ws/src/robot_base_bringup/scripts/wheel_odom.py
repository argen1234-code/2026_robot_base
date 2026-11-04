#!/usr/bin/env python3
"""轮式里程计：/chassis/wheel_speed -> /wheel/odom（不发布 TF）。

左右轮速来自 STM32 遥测（rad/s，前进为正）：
  v   = r * (wL + wR) / 2
  yaw_rate = r * (wR - wL) / wheel_separation

只用于 robot_localization 的 EKF 松耦合：EKF 融合本节点的 vx/vyaw，
再由 EKF 统一发布 odom -> base_footprint。本节点不发 TF。
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')
        self.declare_parameter('wheel_topic', '/chassis/wheel_speed')
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('wheel_radius', 0.041713)
        self.declare_parameter('wheel_separation', 0.148)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('wheel_timeout', 0.50)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.radius = float(self.get_parameter('wheel_radius').value)
        self.separation = float(self.get_parameter('wheel_separation').value)
        self.timeout = float(self.get_parameter('wheel_timeout').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.left = 0.0
        self.right = 0.0
        self.have_wheel = False
        self.last_wheel_time = None
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_report = self.get_clock().now()

        self.pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 20)
        self.create_subscription(
            Float32MultiArray, self.get_parameter('wheel_topic').value,
            self.on_wheel, 20)
        self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(
            f'wheel_odom: r={self.radius:.6f} m track={self.separation:.3f} m')

    def on_wheel(self, msg):
        if len(msg.data) < 2:
            return
        left = float(msg.data[0])
        right = float(msg.data[1])
        if not (math.isfinite(left) and math.isfinite(right)):
            return
        self.left = left
        self.right = right
        self.have_wheel = True
        self.last_wheel_time = self.get_clock().now()

    def publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.2:
            return
        if not self.have_wheel:
            return

        fresh = ((now - self.last_wheel_time).nanoseconds * 1e-9
                 <= self.timeout)
        if fresh:
            v = self.radius * (self.left + self.right) * 0.5
            w = self.radius * (self.right - self.left) / self.separation
        else:
            v = 0.0
            w = 0.0

        # 小幅更新，直接用当前 yaw 积分。
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw = wrap_pi(self.yaw + w * dt)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        # 只给 EKF 用的相关项；其余保持 0。
        cov = [0.0] * 36
        cov[0] = 0.02      # x
        cov[7] = 0.02      # y
        cov[35] = 0.05     # yaw
        msg.pose.covariance = cov
        cov = [0.0] * 36
        cov[0] = 0.01      # vx
        cov[35] = 0.02     # vyaw
        msg.twist.covariance = cov
        self.pub.publish(msg)

        if (now - self.last_report).nanoseconds > 10_000_000_000:
            self.last_report = now
            self.get_logger().info(
                f'wheel_odom pose=({self.x:+.2f},{self.y:+.2f},'
                f'{math.degrees(self.yaw):+.1f}°) v={v:+.3f} w={w:+.3f}',
                throttle_duration_sec=10.0)


def main():
    rclpy.init()
    node = WheelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
