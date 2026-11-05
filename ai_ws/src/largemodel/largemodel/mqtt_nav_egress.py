#!/usr/bin/env python3
"""ROS 2 -> MQTT 导航目标桥（AI 侧，ROS_DOMAIN_ID=2）。

订阅 AI 容器发布的 `/nav_goal`（PoseStamped），发布 MQTT
`robot/ai/nav_goal`；同时把 `robot/nav/status` 转回 ROS `/nav_status`。

本脚本可以跑在主机（ros_jazzy + domain 2，和 AI 容器同域），也可以
放到 AI 容器里运行；AI 镜像需要 paho-mqtt。它不修改导航栈。
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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

ENV_FILE = os.environ.get(
    'ENV_FILE', '/home/pi/2026_robot_base/.env')
HOST_CA_FALLBACK = (
    '/home/pi/2026_robot_base/ai_ws/src/largemodel/config/emqxsl-ca.crt')


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


class MqttNavEgress(Node):
    def __init__(self):
        super().__init__('mqtt_nav_egress')
        env = load_env(ENV_FILE)
        self.broker = get(env, 'AI_MQTT_BROKER', get(env, 'NAV_MQTT_BROKER'))
        self.port = int(get(env, 'AI_MQTT_PORT', '8883'))
        self.user = get(env, 'AI_MQTT_USERNAME')
        self.password = get(env, 'AI_MQTT_PASSWORD')
        self.ca = get(env, 'AI_MQTT_CA_CERTS')
        if not (self.ca and os.path.exists(self.ca)):
            self.ca = get(env, 'NAV_MQTT_CA_CERTS')
        if not (self.ca and os.path.exists(self.ca)):
            self.ca = HOST_CA_FALLBACK
        self.out_topic = get(env, 'AI_MQTT_GOAL_TOPIC', 'robot/ai/nav_goal')
        self.status_topic = get(env, 'AI_MQTT_STATUS_TOPIC',
                                'robot/nav/status')
        if not all((self.broker, self.user, self.password)):
            raise ValueError('AI_MQTT_BROKER/USERNAME/PASSWORD 未配置')

        self.nav_status_pub = self.create_publisher(
            String, '/nav_status', reliable_qos(10))
        self.create_subscription(
            PoseStamped, '/nav_goal', self.on_nav_goal, reliable_qos(10))

        cid = f'ai_egress_{os.getpid()}'
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
            f'MQTT egress: /nav_goal -> {self.out_topic} '
            f'({self.broker}:{self.port})')

    def on_connect(self, client, _userdata, _flags, rc):
        if rc == 0:
            client.subscribe(self.status_topic, qos=1)
            self.get_logger().info(
                f'MQTT connected, subscribed {self.status_topic}')
        else:
            self.get_logger().error(f'MQTT connect failed rc={rc}')

    def on_disconnect(self, _client, _userdata, rc):
        if rc != 0:
            self.get_logger().warn(f'MQTT disconnected rc={rc}, reconnecting')

    def on_message(self, _client, _userdata, msg):
        try:
            text = msg.payload.decode('utf-8')
        except UnicodeDecodeError:
            text = msg.payload.decode('utf-8', errors='replace')
        status = String()
        status.data = text
        self.nav_status_pub.publish(status)
        self.get_logger().info(f'MQTT status -> /nav_status: {text[:160]}')

    def on_nav_goal(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        payload = {
            'frame_id': msg.header.frame_id or 'map',
            'x': float(msg.pose.position.x),
            'y': float(msg.pose.position.y),
            'yaw': float(yaw),
            'request_id': str(uuid.uuid4()),
            'stamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec),
            },
        }
        self.mqtt.publish(self.out_topic,
                          json.dumps(payload, separators=(',', ':')),
                          qos=1, retain=False)
        self.get_logger().info(
            f'/nav_goal -> MQTT: {payload["frame_id"]} '
            f'({payload["x"]:+.2f},{payload["y"]:+.2f}) '
            f'yaw={math.degrees(yaw):+.1f}deg')

    def close(self):
        try:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    try:
        node = MqttNavEgress()
    except Exception as exc:  # noqa: BLE001
        print(f'无法启动 MQTT egress: {exc}', file=sys.stderr)
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
