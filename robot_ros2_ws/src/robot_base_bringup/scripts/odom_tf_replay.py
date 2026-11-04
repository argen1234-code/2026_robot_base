#!/usr/bin/env python3
"""把 rosbag 里的 /odom 转成 TF，用于纯离线 SLAM 回放。

订阅 nav_msgs/Odometry，广播 odom -> base_footprint。
时间戳直接使用消息 header.stamp，保留 bag 的仿真时间，不依赖系统时钟。
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfReplay(Node):
    def __init__(self):
        super().__init__('odom_tf_replay')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tf = TransformBroadcaster(self)
        self.count = 0
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.on_odom, 30)
        self.get_logger().info(
            f'odom_tf_replay: {self.get_parameter("odom_topic").value} '
            f'-> TF {self.odom_frame}->{self.base_frame}')

    def on_odom(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id or self.odom_frame
        t.child_frame_id = msg.child_frame_id or self.base_frame
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        self.tf.sendTransform(t)
        self.count += 1
        if self.count % 300 == 0:
            self.get_logger().info(f'odom_tf_replay frames={self.count}')


def main():
    rclpy.init()
    node = OdomTfReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
