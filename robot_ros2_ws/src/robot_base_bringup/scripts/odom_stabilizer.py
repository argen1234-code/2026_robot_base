#!/usr/bin/env python3
"""Freeze laser odometry while the wheels are stationary (ZUPT).

rf2o 在完全静止时仍会因扫描噪声持续漂移（实测 35 s 漂 12° yaw、3–4 cm xy），
代价地图跟着 odom 走，静态障碍看起来就在“漂”。

本节点不替代 rf2o，而是在它外面加一层零速保持：
  · /chassis/wheel_speed 的左右轮速都小于阈值并持续 hold 秒 -> 冻结输出位姿；
  · 轮速恢复 -> 用冻结位姿与当前 rf2o 位姿算一个偏置，之后
    output = offset ∘ rf2o，保证解冻瞬间不跳变；
  · 没有轮速数据（例如只跑建图、底盘桥没起）-> 原样透传 rf2o。

TF 归属：rf2o 必须 publish_tf:=false、odom_topic:=/odom_rf2o。
EKF 模式下本节点只输出 /odom_stabilized、publish_tf:=false；
odom -> base_footprint 由 ekf_filter_node 唯一发布。
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


def _compose(a, b):
    """2D 位姿复合 a ∘ b，位姿为 (x, y, yaw)。"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1],
            a[1] + s * b[0] + c * b[1],
            a[2] + b[2])


def _inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1],
            s * a[0] - c * a[1],
            -a[2])


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomStabilizer(Node):
    def __init__(self):
        super().__init__('odom_stabilizer')
        self.declare_parameter('odom_in', '/odom_rf2o')
        self.declare_parameter('odom_out', '/odom')
        self.declare_parameter('wheel_topic', '/chassis/wheel_speed')
        self.declare_parameter('target_topic', '/chassis/wheel_target')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        # 轮速实测静止噪声约 ±0.12 rad/s，所以进入阈值要高于它。
        self.declare_parameter('stationary_speed_rad_s', 0.20)
        self.declare_parameter('release_speed_rad_s', 0.30)
        self.declare_parameter('stationary_hold_s', 0.30)
        self.declare_parameter('release_hold_s', 0.15)
        self.declare_parameter('target_motion_threshold', 0.05)
        self.declare_parameter('target_timeout_s', 0.50)
        self.declare_parameter('wheel_timeout_s', 0.50)
        self.declare_parameter('publish_rate', 20.0)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.stationary_speed = float(
            self.get_parameter('stationary_speed_rad_s').value)
        self.release_speed = float(
            self.get_parameter('release_speed_rad_s').value)
        self.stationary_hold = float(
            self.get_parameter('stationary_hold_s').value)
        self.release_hold = float(
            self.get_parameter('release_hold_s').value)
        self.target_motion_threshold = float(
            self.get_parameter('target_motion_threshold').value)
        self.target_timeout = float(
            self.get_parameter('target_timeout_s').value)
        self.wheel_timeout = float(self.get_parameter('wheel_timeout_s').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.in_pose = None          # 最近一帧 rf2o 位姿
        self.in_twist = None
        self.in_cov = None
        self.in_stamp = None
        self.last_wheel = None       # (monotonic, left, right)
        self.last_target = None      # (monotonic, left, right)
        self.output_pose = (0.0, 0.0, 0.0)
        self.offset = (0.0, 0.0, 0.0)
        self.frozen = False
        self.stationary_since = None
        self.release_since = None
        self.freeze_pose = None

        self.pub = self.create_publisher(
            Odometry, self.get_parameter('odom_out').value, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter('odom_in').value, self.on_odom, 20)
        self.create_subscription(
            Float32MultiArray, self.get_parameter('wheel_topic').value,
            self.on_wheel, 20)
        self.create_subscription(
            Float32MultiArray, self.get_parameter('target_topic').value,
            self.on_target, 20)
        self.create_timer(1.0 / rate, self.publish)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.in_pose = (p.x, p.y, _yaw(q))
        self.in_twist = msg.twist.twist
        self.in_cov = msg.pose.covariance
        self.in_stamp = msg.header.stamp

    def on_wheel(self, msg):
        if len(msg.data) < 2:
            return
        self.last_wheel = (self.get_clock().now().nanoseconds * 1e-9,
                           float(msg.data[0]), float(msg.data[1]))

    def on_target(self, msg):
        if len(msg.data) < 2:
            return
        self.last_target = (self.get_clock().now().nanoseconds * 1e-9,
                            float(msg.data[0]), float(msg.data[1]))

    def _motion_evidence(self, now):
        """True=明确在动，False=明确静止，None=证据不足，保持原状态。

        目标轮速是上层运动意图：目标非零就说明 nav/遥控要求运动，即使
        实测轮速还很小（起步）也不能冻结。静止时目标为 0，此时再用实测
        轮速；实测噪声约 ±0.12 rad/s，所以进入静止用 0.20、退出用 0.30。
        """
        target = None
        if self.last_target is not None:
            stamp, left, right = self.last_target
            if now - stamp <= self.target_timeout:
                target = max(abs(left), abs(right)) > self.target_motion_threshold

        speed = None
        if self.last_wheel is not None:
            stamp, left, right = self.last_wheel
            if now - stamp <= self.wheel_timeout:
                speed = max(abs(left), abs(right))

        if target is True:
            return True
        if target is False and (speed is None or speed <= self.stationary_speed):
            return False
        if speed is not None and speed > self.release_speed:
            return True
        if target is None and speed is not None and speed <= self.stationary_speed:
            return False
        return None

    def _release(self):
        """解冻：output 在 freeze_pose 处无缝续接 rf2o。"""
        self.offset = _compose(self.freeze_pose, _inverse(self.in_pose))
        self.frozen = False
        self.release_since = None

    def publish(self):
        if self.in_pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9

        evidence = self._motion_evidence(now)
        if evidence is True:
            self.stationary_since = None
            if self.frozen:
                if self.release_since is None:
                    self.release_since = now
                if now - self.release_since >= self.release_hold:
                    self._release()
            else:
                self.release_since = None
        elif evidence is False:
            self.release_since = None
            if self.stationary_since is None:
                self.stationary_since = now
            if (not self.frozen and
                    now - self.stationary_since >= self.stationary_hold):
                self.frozen = True
                self.freeze_pose = self.output_pose
        else:
            # 证据不足：不改变冻结状态，只清掉未完成的计时。
            self.stationary_since = None
            self.release_since = None

        if self.frozen:
            pose = self.freeze_pose
        else:
            pose = _compose(self.offset, self.in_pose)
        self.output_pose = pose

        out = Odometry()
        out.header.stamp = self.in_stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = pose[0]
        out.pose.pose.position.y = pose[1]
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        out.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        if self.in_cov is not None:
            out.pose.covariance = [float(v) for v in self.in_cov]
        if self.in_twist is not None:
            out.twist.twist = self.in_twist
            if self.frozen:
                out.twist.twist.linear.x = 0.0
                out.twist.twist.linear.y = 0.0
                out.twist.twist.angular.z = 0.0
        self.pub.publish(out)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = self.in_stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = pose[0]
            tf.transform.translation.y = pose[1]
            tf.transform.rotation.z = math.sin(pose[2] / 2.0)
            tf.transform.rotation.w = math.cos(pose[2] / 2.0)
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomStabilizer()
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
