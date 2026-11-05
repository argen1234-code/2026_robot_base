#!/usr/bin/env python3
"""MQTT -> ROS 2 导航目标桥（导航侧，ROS_DOMAIN_ID=1）。

只做一件事：订阅 MQTT `robot/ai/nav_goal`，转成 `/goal_pose`
（PoseStamped, map 系, 当前 stamp, RELIABLE），交给现有的 goal_sanitizer。
不修改导航栈。

MQTT 消息格式：
    {"frame_id": "map", "x": 1.2, "y": 0.3, "yaw": 0.0,
     "request_id": "optional"}
也接受 {"pose": {"position": {"x":...}, ...}} 形式。

状态回传 `robot/nav/status`：
    {"type":"ack", "state":"accepted|rejected", "request_id":...}
    {"type":"sanitizer", "detail":"..."}
    {"type":"nav_result", "status":"SUCCESS|FAILURE"}
"""

import json
import math
import os
import signal
import sys
import uuid

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import BehaviorTreeLog
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

ENV_FILE = os.environ.get(
    'ENV_FILE', '/home/pi/2026_robot_base/.env')


def load_env(path):
    env = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#') or '=' not in line:
                    continue
                key, value = line.split('=', 1)
                env[key] = value
    except OSError:
        pass
    return env


def get(env, key, default=None):
    return os.environ.get(key, env.get(key, default))


def reliable_qos(depth=10):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)


class MqttNavIngress(Node):
    def __init__(self):
        super().__init__('mqtt_nav_ingress')
        env = load_env(ENV_FILE)
        self.broker = get(env, 'NAV_MQTT_BROKER')
        self.port = int(get(env, 'NAV_MQTT_PORT', '8883'))
        self.user = get(env, 'NAV_MQTT_USERNAME')
        self.password = get(env, 'NAV_MQTT_PASSWORD')
        self.ca = get(env, 'NAV_MQTT_CA_CERTS')
        self.in_topic = get(env, 'NAV_MQTT_GOAL_TOPIC', 'robot/ai/nav_goal')
        self.status_topic = get(env, 'NAV_MQTT_STATUS_TOPIC',
                                'robot/nav/status')
        if not all((self.broker, self.user, self.password)):
            raise ValueError('NAV_MQTT_BROKER/USERNAME/PASSWORD 未配置')

        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', reliable_qos(10))
        self.create_subscription(
            String, '/goal_sanitizer_status', self.on_sanitizer_status,
            reliable_qos(10))
        self.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log', self.on_bt_log,
            reliable_qos(10))

        cid = f'nav_ingress_{os.getpid()}'
        self.mqtt = mqtt.Client(client_id=cid, clean_session=True)
        self.mqtt.username_pw_set(self.user, self.password)
        if self.ca and os.path.exists(self.ca):
            self.mqtt.tls_set(ca_certs=self.ca)
        else:
            self.get_logger().warn(f'CA 文件不存在，使用系统 CA: {self.ca}')
            self.mqtt.tls_set()
        self.mqtt.tls_insecure_set(False)
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_message
        self.mqtt.on_disconnect = self.on_disconnect
        self.mqtt.reconnect_delay_set(min_delay=1, max_delay=10)
        self.mqtt.connect_async(self.broker, self.port, keepalive=30)
        self.mqtt.loop_start()
        self.get_logger().info(
            f'MQTT ingress: {self.broker}:{self.port} '
            f'{self.in_topic} -> /goal_pose')

    def on_connect(self, client, _userdata, _flags, rc):
        if rc == 0:
            client.subscribe(self.in_topic, qos=1)
            self.get_logger().info(
                f'MQTT connected, subscribed {self.in_topic}')
        else:
            self.get_logger().error(f'MQTT connect failed rc={rc}')

    def on_disconnect(self, _client, _userdata, rc):
        if rc != 0:
            self.get_logger().warn(f'MQTT disconnected rc={rc}, reconnecting')

    def publish_status(self, payload):
        self.mqtt.publish(self.status_topic,
                          json.dumps(payload, separators=(',', ':')),
                          qos=1, retain=False)

    def on_sanitizer_status(self, msg):
        self.publish_status({'type': 'sanitizer', 'detail': msg.data})

    def on_bt_log(self, msg):
        for event in msg.event_log:
            if (event.node_name == 'NavigateWithReplanning'
                    and event.current_status in ('SUCCESS', 'FAILURE')):
                self.publish_status({
                    'type': 'nav_result',
                    'status': event.current_status,
                })

    @staticmethod
    def _parse_goal(data):
        if not isinstance(data, dict):
            raise ValueError('payload 不是 JSON object')
        if 'pose' in data:
            pose = data.get('pose') or {}
            pos = pose.get('position') or {}
            ori = pose.get('orientation') or {}
            x, y = pos.get('x'), pos.get('y')
            yaw = data.get('yaw')
            if yaw is None and all(k in ori for k in ('x', 'y', 'z', 'w')):
                qx, qy, qz, qw = (float(ori[k]) for k in ('x', 'y', 'z', 'w'))
                yaw = math.atan2(2 * (qw * qz + qx * qy),
                                 1 - 2 * (qy * qy + qz * qz))
        else:
            x, y, yaw = data.get('x'), data.get('y'), data.get('yaw', 0.0)
        if x is None or y is None:
            raise ValueError('缺少 x/y')
        x, y = float(x), float(y)
        yaw = float(yaw or 0.0)
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('x/y/yaw 非有限值')
        frame = str(data.get('frame_id') or 'map')
        return frame, x, y, yaw

    def on_message(self, _client, _userdata, msg):
        request_id = None
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            request_id = data.get('request_id')
            frame, x, y, yaw = self._parse_goal(data)
            goal = PoseStamped()
            goal.header.frame_id = frame
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.z = math.sin(yaw * 0.5)
            goal.pose.orientation.w = math.cos(yaw * 0.5)
            self.goal_pub.publish(goal)
            self.publish_status({
                'type': 'ack', 'state': 'accepted',
                'request_id': request_id, 'frame_id': frame,
            })
            self.get_logger().info(
                f'MQTT goal -> /goal_pose: {frame} '
                f'({x:+.2f},{y:+.2f}) yaw={math.degrees(yaw):+.1f}deg')
        except Exception as exc:  # noqa: BLE001
            self.publish_status({
                'type': 'ack', 'state': 'rejected',
                'request_id': request_id, 'detail': str(exc),
            })
            self.get_logger().warn(f'Rejected MQTT goal: {exc}')

    def close(self):
        try:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    try:
        node = MqttNavIngress()
    except Exception as exc:  # noqa: BLE001
        print(f'无法启动 MQTT ingress: {exc}', file=sys.stderr)
        rclpy.shutdown()
        return 1

    def _stop(*_args):
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _stop)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
