#!/usr/bin/env python3
"""在 map 坐标系中，沿机器人当前朝向发送一个前向目标点。

默认目标 = 雷达正前方 1.0 m：
  base_footprint +X 是车头；雷达在 base_footprint +X 约 0.2225 m，
  所以 goal_x = robot_x + 0.2225 + distance。

发布到 /goal_pose（Nav2 bt_navigator 直接订阅），不依赖 RViz。
用法：
  python3 tools/send_goal_ahead.py --distance 1.0
  python3 tools/send_goal_ahead.py --distance 1.5 --frame map
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GoalAhead(Node):
    def __init__(self, frame: str, base_frame: str, distance: float,
                 lidar_offset: float, timeout: float):
        super().__init__('goal_ahead')
        self.frame = frame
        self.base_frame = base_frame
        self.distance = distance
        self.lidar_offset = lidar_offset
        self.timeout = timeout
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def run(self) -> bool:
        deadline = time.time() + self.timeout
        tf = None
        while time.time() < deadline:
            # 必须自旋，TransformListener 才能收到 /tf。
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.buffer.lookup_transform(
                    self.frame, self.base_frame, Time())
                break
            except Exception as exc:  # noqa: BLE001
                self.get_logger().info(
                    f'等待 TF {self.frame}->{self.base_frame} ... ({exc})',
                    throttle_duration_sec=2.0)
                time.sleep(0.1)
        if tf is None:
            self.get_logger().error(
                f'TF {self.frame}->{self.base_frame} 超时；SLAM 是否已启动？')
            return False

        t = tf.transform.translation
        yaw = yaw_from_quat(tf.transform.rotation)
        ahead = self.lidar_offset + self.distance
        gx = t.x + ahead * math.cos(yaw)
        gy = t.y + ahead * math.sin(yaw)

        msg = PoseStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'robot=({t.x:+.3f},{t.y:+.3f},yaw={math.degrees(yaw):+.1f}°) '
            f'lidar+{self.distance:.2f}m -> goal=({gx:+.3f},{gy:+.3f}) '
            f'frame={self.frame}')
        # 等 bt_navigator 发现发布者，然后只发一次。
        # 重复发 /goal_pose 会让每个新目标 preempt 上一个，控制器只跑一小段。
        deadline = time.time() + 5.0
        while (self.pub.get_subscription_count() == 0
               and time.time() < deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.2)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        # 自旋一会儿，确保消息被 DDS 送出。
        spin_end = time.time() + 0.5
        while time.time() < spin_end:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(
            f'已发布一次 /goal_pose，订阅者={self.pub.get_subscription_count()}')
        return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--distance', type=float, default=1.0,
                        help='从雷达到目标的距离 m（默认 1.0）')
    parser.add_argument('--lidar-offset', type=float, default=0.2225,
                        help='雷达相对 base_footprint 的前向偏移 m')
    parser.add_argument('--frame', default='map')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--timeout', type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init(args=None)
    node = GoalAhead(args.frame, args.base_frame, args.distance,
                     args.lidar_offset, args.timeout)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
