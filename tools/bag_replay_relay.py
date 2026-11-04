#!/usr/bin/env python3
"""把 rosbag 回放的输入话题重新打上“当前时间”后转发。

为什么需要它：
    bag 里的 /tf、/scan、/goal_pose 带的是录制时刻的时间戳（本 bag 是
    2026-11-05 05:54）。如果直接回放给 use_sim_time=false 的 Nav2，TF 查询
    会认为数据“已经过期”，DWB 会持续报 Could not get robot pose / TF 失败。

用法（配合 ros2 bag play --remap）：
    ros2 bag play <bag> \
      --topics /scan_filtered /odom /tf /tf_static /map /goal_pose \
      --remap /scan_filtered:=/bag/scan_filtered /odom:=/bag/odom \
              /tf:=/bag/tf /goal_pose:=/bag/goal_pose

    本节点订阅 /bag/*，以当前墙钟时间重发到 /scan_filtered、/odom、/tf、
    /goal_pose。/tf_static 与 /map 是 transient_local，直接回放即可。
"""

import signal

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def reliable_qos(depth):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class BagReplayRelay(Node):
    def __init__(self):
        super().__init__('bag_replay_relay')
        self.counts = {'scan': 0, 'odom': 0, 'tf': 0, 'goal': 0}

        self.pub_scan = self.create_publisher(LaserScan, '/scan_filtered',
                                              reliable_qos(5))
        self.pub_odom = self.create_publisher(Odometry, '/odom',
                                              reliable_qos(20))
        self.pub_tf = self.create_publisher(TFMessage, '/tf', reliable_qos(100))
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose',
                                              reliable_qos(10))

        sub_qos = reliable_qos(50)
        self.create_subscription(LaserScan, '/bag/scan_filtered',
                                 self.on_scan, sub_qos)
        self.create_subscription(Odometry, '/bag/odom', self.on_odom, sub_qos)
        self.create_subscription(TFMessage, '/bag/tf', self.on_tf, sub_qos)
        self.create_subscription(PoseStamped, '/bag/goal_pose',
                                 self.on_goal, sub_qos)
        self.get_logger().info('relay ready: /bag/{scan_filtered,odom,tf,'
                               'goal_pose} -> current-stamped topics')

    def stamp_now(self):
        return self.get_clock().now().to_msg()

    def _tick(self, key):
        self.counts[key] += 1
        if self.counts[key] % 200 == 0:
            self.get_logger().info(
                f"relayed scan={self.counts['scan']} odom={self.counts['odom']} "
                f"tf={self.counts['tf']} goal={self.counts['goal']}")

    def on_scan(self, msg):
        msg.header.stamp = self.stamp_now()
        self.pub_scan.publish(msg)
        self._tick('scan')

    def on_odom(self, msg):
        msg.header.stamp = self.stamp_now()
        self.pub_odom.publish(msg)
        self._tick('odom')

    def on_tf(self, msg):
        now = self.stamp_now()
        for tr in msg.transforms:
            tr.header.stamp = now
        self.pub_tf.publish(msg)
        self._tick('tf')

    def on_goal(self, msg):
        msg.header.stamp = self.stamp_now()
        self.pub_goal.publish(msg)
        self._tick('goal')
        self.get_logger().info(
            f'goal relayed frame={msg.header.frame_id} '
            f'({msg.pose.position.x:+.2f},{msg.pose.position.y:+.2f})')


def main():
    rclpy.init()
    node = BagReplayRelay()

    def _stop(*_args):
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _stop)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().info(f'relay totals: {node.counts}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
