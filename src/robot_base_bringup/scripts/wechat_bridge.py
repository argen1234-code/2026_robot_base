#!/usr/bin/env python3
"""Optional TLS MQTT adapter for the reference WeChat mini-program schema."""

import json
import math
import os
import queue
import time

import paho.mqtt.client as mqtt
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8
from tf2_ros import Buffer, TransformListener, TransformException

from wechat_protocol import parse_command
from wechat_map import map_payload


class WechatBridge(Node):
    def __init__(self):
        super().__init__('wechat_bridge')
        self.declare_parameter('broker', os.environ.get(
            'WECHAT_MQTT_BROKER', 'i6130f30.ala.cn-hangzhou.emqxsl.cn'))
        self.declare_parameter('port', int(os.environ.get('WECHAT_MQTT_PORT', '8883')))
        self.declare_parameter('client_id', os.environ.get('WECHAT_MQTT_CLIENT_ID', 'robot_001_jetson'))
        self.declare_parameter('username', os.environ.get('WECHAT_MQTT_USERNAME', ''))
        self.declare_parameter('ca_certs', os.environ.get('WECHAT_MQTT_CA_CERTS', ''))
        self.declare_parameter('sub_topic', '/k1ck5t83zdZ/test/user/get')
        self.declare_parameter('pub_topic', '/k1ck5t83zdZ/test/user/robot')
        self.declare_parameter('map_pub_topic', '/k1ck5t83zdZ/test/user/map')
        self.declare_parameter('path_pub_topic', '/k1ck5t83zdZ/test/user/path')
        self.declare_parameter('mission_pub_topic', '/k1ck5t83zdZ/test/user/mission')
        self.declare_parameter('max_linear', 0.20)
        self.declare_parameter('max_angular', 0.45)
        get = lambda key: self.get_parameter(key).value
        password = os.environ.get('WECHAT_MQTT_PASSWORD', '')
        if not get('broker') or not get('username') or not password:
            raise ValueError('WECHAT_MQTT_BROKER, USERNAME and PASSWORD are required')
        self.robot_id = get('client_id').removesuffix('_jetson')
        self.topics = {key: get(key) for key in
                       ('sub_topic', 'pub_topic', 'map_pub_topic', 'path_pub_topic', 'mission_pub_topic')}
        self.max_linear = get('max_linear')
        self.max_angular = get('max_angular')
        self.events = queue.Queue(maxsize=64)
        self.mode = 3
        self.emergency = False
        self.mission = None
        self.goal_handle = None
        self.generation = 0
        self.last_map = None
        self.map_revision = 0
        self.map_signature = None
        self.seen_requests = {}
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.action = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.remote_pub = self.create_publisher(Twist, '/remote_cmd_vel', 10)
        self.mode_pub = self.create_publisher(Int8, '/robot_mode', 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(Path, '/plan', self.on_path, 10)
        self.create_timer(0.05, self.process_events)
        self.create_timer(1.0, self.publish_pose)
        self.client = mqtt.Client(client_id=get('client_id'), clean_session=True)
        self.client.username_pw_set(get('username'), password)
        self.client.tls_set(ca_certs=get('ca_certs') or None)
        self.client.tls_insecure_set(False)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        self.connected = False
        self.client.connect_async(get('broker'), int(get('port')), keepalive=30)
        self.client.loop_start()
        self.get_logger().info('WeChat MQTT bridge started (TLS; motors remain separately gated)')

    def on_connect(self, client, _userdata, _flags, rc):
        self.connected = rc == 0
        if self.connected:
            client.subscribe(self.topics['sub_topic'], qos=1)
        else:
            self.get_logger().error(f'MQTT connection rejected: {rc}')

    def on_disconnect(self, _client, _userdata, _rc):
        self.connected = False
        # Last remote Twist expires in chassis_bridge after 0.25s.

    def on_message(self, _client, _userdata, msg):
        if msg.retain:
            return
        try:
            self.events.put_nowait((time.monotonic(), msg.payload))
        except queue.Full:
            self.get_logger().warn('MQTT command queue full; dropping command')

    def publish(self, topic, data, retain=False):
        if self.connected:
            self.client.publish(self.topics[topic], json.dumps(data, separators=(',', ':')),
                                qos=1, retain=retain)

    def set_mode(self, mode):
        if self.mode != mode:
            self.remote_pub.publish(Twist())
            self.mode = mode
            self.mode_pub.publish(Int8(data=mode))

    def cancel_mission(self, state='CANCELLED'):
        self.generation += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.mission = None
        self.publish('mission_pub_topic', {'state': state}, retain=True)

    def process_events(self):
        for _ in range(16):
            try:
                stamp, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if time.monotonic() - stamp > 0.5:
                continue
            try:
                data = json.loads(payload)
                command, arg = parse_command(payload)
                request_id = str(data.get('request_id', ''))[:128]
                if request_id:
                    key = (command, request_id)
                    now = time.monotonic()
                    self.seen_requests = {k: t for k, t in self.seen_requests.items() if now - t < 30}
                    if key in self.seen_requests:
                        continue
                    self.seen_requests[key] = now
                self.handle_command(command, arg, data)
            except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                self.get_logger().warn(f'Rejected MQTT command: {exc}')

    def handle_command(self, command, arg, data):
        if command == 'EMERGENCY':
            self.emergency = True
            self.cancel_mission('EMERGENCY')
            self.set_mode(2)
            self.remote_pub.publish(Twist())
            return
        if command == 'RESET_EMERGENCY':
            self.emergency = False
            return
        if self.emergency:
            return
        if command in ('REMOTE', 'INDOOR', 'LINE'):
            if command == 'REMOTE':
                if self.mission is not None or self.goal_handle is not None:
                    self.cancel_mission()
            self.set_mode(2 if command == 'REMOTE' else 3)
        elif command in ('FORWARD', 'BACKWARD', 'LEFT', 'RIGHT', 'STOP'):
            if self.mission is not None or self.goal_handle is not None:
                self.cancel_mission()
            self.set_mode(2)
            tw = Twist()
            if command == 'FORWARD':
                tw.linear.x = arg * self.max_linear
            elif command == 'BACKWARD':
                tw.linear.x = -arg * self.max_linear
            elif command == 'LEFT':
                tw.angular.z = arg * self.max_angular
            elif command == 'RIGHT':
                tw.angular.z = -arg * self.max_angular
            self.remote_pub.publish(tw)
        elif command == 'INDOOR_MISSION_CANCEL':
            self.cancel_mission()
            self.set_mode(2)
        elif command == 'INDOOR_MISSION_START':
            if not self.action.server_is_ready():
                self.publish('mission_pub_topic', {'state': 'ERROR', 'message': 'Nav2 unavailable'})
                return
            self.cancel_mission()
            self.set_mode(3)
            self.mission = str(data.get('mission_id', ''))[:128]
            gen = self.generation
            goal = NavigateThroughPoses.Goal()
            for x, y, yaw in arg:
                from geometry_msgs.msg import PoseStamped
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x, pose.pose.position.y = x, y
                pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.orientation.w = math.cos(yaw / 2)
                goal.poses.append(pose)
            future = self.action.send_goal_async(goal)
            future.add_done_callback(lambda result: self.on_goal(gen, result))
            self.publish('mission_pub_topic', {'state': 'STARTED', 'mission_id': self.mission}, True)

    def on_goal(self, gen, future):
        try:
            handle = future.result()
            if gen != self.generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.publish('mission_pub_topic', {'state': 'ERROR', 'message': 'Goal rejected'}, True)
                self.mission = None
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(lambda result: self.on_result(gen, result))
        except Exception as exc:
            self.get_logger().error(f'Nav2 goal failed: {exc}')
            if gen == self.generation:
                self.mission = None
                self.publish('mission_pub_topic', {'state': 'ERROR', 'message': 'Nav2 goal failed'}, True)

    def on_result(self, gen, future):
        if gen != self.generation:
            return
        try:
            state = 'SUCCEEDED' if future.result().status == GoalStatus.STATUS_SUCCEEDED else 'FAILED'
        except Exception:
            state = 'FAILED'
        self.publish('mission_pub_topic', {'state': state, 'mission_id': self.mission}, True)
        self.goal_handle = None
        self.mission = None

    def on_map(self, msg):
        self.last_map = msg

    def publish_pose(self):
        payload = {'robot_id': self.robot_id, 'online': True, 'mode': self.mode,
                   'requested_mode': self.mode, 'timestamp': time.time()}
        try:
            trans = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            q = trans.transform.rotation
            payload.update(x=round(trans.transform.translation.x, 4),
                           y=round(trans.transform.translation.y, 4),
                           yaw=round(math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z))), 2))
        except TransformException:
            pass
        self.publish('pub_topic', payload)
        if self.connected and self.last_map is not None:
            msg = self.last_map
            signature = (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.info.width,
                         msg.info.height, hash(tuple(msg.data)))
            if signature != self.map_signature:
                try:
                    data = map_payload(msg, self.map_revision + 1)
                    self.publish('map_pub_topic', data, True)
                    self.map_revision += 1
                    self.map_signature = signature
                except ValueError:
                    pass

    def on_path(self, msg):
        if not self.connected:
            return
        step = max(1, math.ceil(len(msg.poses) / 120))
        points = [{'x': round(p.pose.position.x, 4), 'y': round(p.pose.position.y, 4)}
                  for p in msg.poses[::step]]
        if msg.poses:
            last = {'x': round(msg.poses[-1].pose.position.x, 4),
                    'y': round(msg.poses[-1].pose.position.y, 4)}
            if not points or points[-1] != last:
                points.append(last)
        self.publish('path_pub_topic', {'type': 'indoor_path', 'frame_id': msg.header.frame_id,
                                        'points': points})

    def stop(self):
        self.client.disconnect()
        self.client.loop_stop()


def main():
    rclpy.init()
    node = WechatBridge()
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
