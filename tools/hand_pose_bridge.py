#!/usr/bin/env python3
"""把主机 MediaPipe 手部检测（UDP）桥接成 ROS 2 话题。

    /perception/hand_pose   geometry_msgs/PoseStamped
    /perception/hand_debug  std_msgs/String (JSON)
"""

import json
import socket

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class HandPoseBridge(Node):
    def __init__(self):
        super().__init__('hand_pose_bridge')
        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('frame_id', 'hand_camera_optical_frame')
        port = int(self.get_parameter('udp_port').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/perception/hand_pose', 10)
        self.debug_pub = self.create_publisher(
            String, '/perception/hand_debug', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port))
        self.sock.setblocking(False)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'hand bridge UDP :{port}')

    def tick(self):
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(8192)
            except BlockingIOError:
                break
            try:
                latest = json.loads(data.decode('utf-8'))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
        if latest is None:
            return
        self.debug_pub.publish(
            String(data=json.dumps(latest, ensure_ascii=False)))
        if not latest.get('valid'):
            return
        xyz = latest.get('cartesian_m') or []
        if len(xyz) != 3:
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = latest.get('frame_id') or self.frame_id
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = HandPoseBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
