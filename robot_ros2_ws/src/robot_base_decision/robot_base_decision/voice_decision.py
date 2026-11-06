#!/usr/bin/env python3
"""Voice-stack decision layer: turn ASR text into confirmed intents.

Task phrases are not executed directly.  The layer asks a second confirmation
question, then publishes a confirmed DecisionIntent for the top-level task
coordinator.  It speaks through /decision/speak (model_service TTS).
"""

import argparse
import json
import re
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from robot_base_msgs.msg import DecisionIntent, TaskStatus
from robot_base_msgs.srv import SetTask

from .priority import CONFIRMED_VOICE, STOP


APPROACH_WORDS = (
    '丢垃圾', '扔垃圾', '来找我', '过来', '到我身边', '到我这里',
    '来我这边', '需要你', '过来一下', '来一下',
)
MOVE_RE = re.compile(
    r'(向前|向后|向左|向右|往前|往后|往左|往右|前|后|左|右)'
    r'\s*(?:走|移动|走个|走大约)?\s*'
    r'([0-9]+(?:\.[0-9]+)?|[一二三四五六七八九十两]+)\s*米')
POSITIVE = ('好的', '好', '是', '可以', '需要', '确认', '对', '行', '嗯')
NEGATIVE = ('不用', '不要', '不好', '取消', '停', '算了', '不是', '别')
STOP_WORDS = ('停止', '停下', '停下来', '别动', '不要动', '取消任务')
DIRECTION = {
    '前': ('forward', '前方'), '向前': ('forward', '前方'),
    '往前': ('forward', '前方'),
    '后': ('backward', '后方'), '向后': ('backward', '后方'),
    '往后': ('backward', '后方'),
    '左': ('left', '左侧'), '向左': ('left', '左侧'),
    '往左': ('left', '左侧'),
    '右': ('right', '右侧'), '向右': ('right', '右侧'),
    '往右': ('right', '右侧'),
}
CN_NUM = {'一': 1, '二': 2, '两': 2, '三': 3, '四': 4, '五': 5,
          '六': 6, '七': 7, '八': 8, '九': 9, '十': 10}


def _number(text):
    if text in CN_NUM:
        return float(CN_NUM[text])
    try:
        return float(text)
    except ValueError:
        return None


class VoiceDecision(Node):
    def __init__(self, execute=False, preview=True):
        super().__init__('voice_decision')
        self.declare_parameter('confirmation_timeout_s', 10.0)
        self.timeout = float(
            self.get_parameter('confirmation_timeout_s').value)
        self.execute = bool(execute)
        self.preview = bool(preview)

        self.pending = None
        self.last_state = 'idle'
        self.last_detail = ''
        self.request_id = ''

        self.create_subscription(String, '/asr', self.on_asr, 5)
        self.intent_pub = self.create_publisher(
            DecisionIntent, '/decision/intent', 5)
        self.coordinator_client = self.create_client(
            SetTask, '/decision/set_task')
        # Transient-local so a prompt published before model_service has
        # matched the subscription is still delivered on match.
        self.speak_pub = self.create_publisher(
            String, '/decision/speak',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub = self.create_publisher(
            TaskStatus, '/decision/voice_status', 5)
        self.create_timer(0.2, self.tick)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f'voice decision ready (confirmation timeout {self.timeout:g}s)')

    def speak(self, text):
        msg = String()
        msg.data = text

        # Repeat briefly: a single cross-environment one-shot can be lost
        # during DDS discovery. model_service de-duplicates identical prompts.
        def _send():
            for _ in range(4):
                self.speak_pub.publish(msg)
                time.sleep(0.25)
        threading.Thread(target=_send, daemon=True).start()

    def status(self, state, detail=''):
        self.last_state, self.last_detail = state, detail
        self.publish_status()

    def publish_status(self):
        msg = TaskStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.owner = 'voice'
        msg.task = self.pending[0] if self.pending else 'none'
        msg.state = self.last_state
        msg.detail = self.last_detail
        msg.request_id = self.request_id
        self.status_pub.publish(msg)

    def parse_task(self, text):
        if any(word in text for word in APPROACH_WORDS):
            return 'approach_user', {}, '好的，那现在需要我去到你身边吗？'
        match = MOVE_RE.search(text)
        if match:
            direction = DIRECTION.get(match.group(1), ('forward', '前方'))
            distance = _number(match.group(2))
            if direction and distance and 0.1 <= distance <= 5.0:
                params = {'direction': direction[0],
                          'distance_m': distance, 'yaw_deg': 0.0}
                question = (f'确认向{direction[1]}走{distance:g}米吗？')
                return 'move_relative', params, question
        return None

    def on_asr(self, msg):
        text = (msg.data or '').strip()
        self.get_logger().info(f'ASR: {text!r}')
        if not text:
            return
        # Stop is always immediate, never confirmed.
        if any(word in text for word in STOP_WORDS):
            self.pending = None
            self.request_id = f'{int(time.time()*1000)}'
            intent = DecisionIntent()
            intent.intent = 'stop'
            intent.source = 'voice'
            intent.confirmed = True
            intent.priority = STOP
            intent.request_id = self.request_id
            intent.params_json = '{}'
            self.intent_pub.publish(intent)
            self.speak('好的，已停止。')
            self.status('cancelled', 'voice stop')
            return

        parsed = self.parse_task(text)
        if self.pending is not None:
            if parsed is not None:
                # A new task replaces the old proposal.
                self.propose(*parsed)
                return
            if any(word in text for word in NEGATIVE):
                self.pending = None
                self.speak('好的，已取消。')
                self.status('cancelled', 'user declined')
                return
            if any(word in text for word in POSITIVE):
                intent, params, _ = self.pending
                self.request_id = f'{int(time.time()*1000)}'
                # Use the coordinator's service for a reliable dispatch; a
                # one-shot topic publish can be missed during DDS discovery.
                if self.coordinator_client.service_is_ready():
                    req = SetTask.Request()
                    req.task = intent
                    req.params_json = json.dumps(params, ensure_ascii=False)
                    req.execute = self.execute
                    req.preview = self.preview
                    self.coordinator_client.call_async(req)
                else:
                    self.get_logger().warn(
                        'coordinator /decision/set_task unavailable')
                self.speak('好的，开始执行。')
                self.status('confirmed', intent)
                self.pending = None
                return
            return
        if parsed is not None:
            self.propose(*parsed)

    def propose(self, intent, params, question):
        self.pending = (intent, params, time.monotonic())
        self.request_id = f'{int(time.time()*1000)}'
        self.speak(question)
        self.status('confirming', question)

    def tick(self):
        if self.pending is not None:
            if time.monotonic() - self.pending[2] > self.timeout:
                self.pending = None
                self.speak('没有听到确认，已取消。')
                self.status('timeout', 'confirmation timeout')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--preview', action='store_true')
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = VoiceDecision(execute=args.execute,
                         preview=args.preview or not args.execute)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
