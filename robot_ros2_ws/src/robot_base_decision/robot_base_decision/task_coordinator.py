#!/usr/bin/env python3
"""Top-level decision layer.

Schedules the navigation/voice/vision stack decision layers.  It receives
confirmed intents, dispatches them to the owning stack, and publishes one
aggregate status.  Stop and manual Foxglove goals preempt stack tasks.
"""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from geometry_msgs.msg import PoseStamped
from robot_base_msgs.msg import DecisionIntent, TaskStatus
from robot_base_msgs.srv import SetTask

from .priority import MANUAL_GOAL, STOP


def vision_task_for(task):
    """Map a user-facing task to the vision layer's mode.

    'approach_user' must be a tracking task ('follow'): the person may move
    while the robot is on its way, and freezing the first standoff point makes
    the robot stop short of the person.
    """
    return 'follow' if task == 'approach_user' else task


class TaskCoordinator(Node):
    def __init__(self):
        super().__init__('task_coordinator')
        self.declare_parameter('initial_task', 'none')
        self.declare_parameter('execute', False)
        self.declare_parameter('preview', False)
        # DDS service discovery on the Pi can lag node startup.  Wait longer
        # before dispatching the initial task, and let call() wait as well.
        self.declare_parameter('initial_delay_s', 10.0)
        self.initial_task = str(
            self.get_parameter('initial_task').value).strip().lower()
        self.execute = bool(self.get_parameter('execute').value)
        self.preview = bool(self.get_parameter('preview').value)
        delay = float(self.get_parameter('initial_delay_s').value)

        self.owner = 'idle'
        self.task = 'none'
        self.state = 'idle'
        self.detail = ''
        self.request_id = ''
        self.voice_state = 'idle'
        self.manual_until = 0.0

        self.vision_client = self.create_client(
            SetTask, '/decision/vision_task')
        self.nav_client = self.create_client(
            SetTask, '/decision/nav_task')
        self.create_service(SetTask, '/decision/set_task', self.on_set_task)
        self.create_subscription(
            DecisionIntent, '/decision/intent', self.on_intent, 10)
        self.create_subscription(
            TaskStatus, '/decision/vision_status', self.on_vision_status, 10)
        self.create_subscription(
            TaskStatus, '/decision/nav_status', self.on_nav_status, 10)
        self.create_subscription(
            TaskStatus, '/decision/voice_status', self.on_voice_status, 10)
        self.create_subscription(Bool, '/stop_request', self.on_stop, 1)
        self.create_subscription(
            PoseStamped, '/goal_pose_safe', self.on_manual_goal, 10)
        self.status_pub = self.create_publisher(
            TaskStatus, '/decision/status', 5)
        self.create_timer(0.5, self.publish_status)
        self._initial_timer = None
        if delay > 0 and self.initial_task not in ('none', ''):
            self._initial_timer = self.create_timer(
                delay, self.start_initial_task)
        self.get_logger().info(
            f'task coordinator ready (initial_task={self.initial_task}, '
            f'execute={self.execute}, preview={self.preview})')

    def set_state(self, owner, task, state, detail='', request_id=''):
        self.owner, self.task, self.state = owner, task, state
        self.detail = detail
        if request_id:
            self.request_id = request_id
        self.publish_status()
        self.get_logger().info(
            f'decision: owner={owner} task={task} state={state} {detail}')

    def publish_status(self):
        msg = TaskStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.owner, msg.task, msg.state = self.owner, self.task, self.state
        msg.detail, msg.request_id = self.detail, self.request_id
        self.status_pub.publish(msg)

    def call(self, client, task, params=None, request_id='', execute=None,
             preview=None, wait_s=5.0):
        if not client.service_is_ready():
            # Service discovery can take a few seconds on the Pi, especially
            # for the initial task right after all decision nodes start.
            self.get_logger().info(
                f'Waiting up to {wait_s:g}s for {client.srv_name}')
            if wait_s <= 0 or not client.wait_for_service(timeout_sec=wait_s):
                self.set_state(self.owner, task, 'failed',
                               f'{client.srv_name} unavailable')
                return
        req = SetTask.Request()
        req.task = task
        req.params_json = json.dumps(params or {}, ensure_ascii=False)
        req.execute = self.execute if execute is None else bool(execute)
        req.preview = self.preview if preview is None else bool(preview)
        future = client.call_async(req)

        def done(fut):
            try:
                res = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.set_state(self.owner, task, 'failed', str(exc))
                return
            if not res.success:
                self.set_state(self.owner, task, 'failed', res.message)
            else:
                self.set_state(self.owner, task, 'dispatching', res.message,
                               request_id)
        future.add_done_callback(done)

    def dispatch(self, intent):
        request_id = intent.request_id
        if intent.intent == 'stop':
            self.request_id = request_id
            self.set_state('coordinator', 'stop', 'stopping', 'stop', request_id)
            self.call(self.vision_client, 'stop', request_id=request_id,
                      execute=False, preview=False, wait_s=0.5)
            self.call(self.nav_client, 'stop', request_id=request_id,
                      execute=False, preview=False, wait_s=0.5)
            return
        if intent.intent == 'approach_user':
            self.request_id = request_id
            # Voice approach is a tracking task: the vision layer keeps the
            # person locked and re-plans as they move, instead of freezing the
            # first standoff point.
            task = vision_task_for('approach_user')
            self.set_state('coordinator', task, 'dispatching',
                           'approach_user -> follow', request_id)
            self.call(self.vision_client, task, {}, request_id,
                      execute=self.execute, preview=self.preview)
            return
        if intent.intent == 'move_relative':
            self.request_id = request_id
            params = json.loads(intent.params_json or '{}')
            self.set_state('coordinator', 'move_relative', 'dispatching',
                           intent.params_json, request_id)
            self.call(self.nav_client, 'move_relative', params, request_id)
            return
        self.set_state(self.owner, intent.intent, 'ignored',
                       'unsupported intent')

    def on_intent(self, msg):
        if not msg.confirmed:
            return
        if time.monotonic() < self.manual_until and msg.intent != 'stop':
            self.set_state(self.owner, msg.intent, 'ignored',
                           'manual goal owns motion; retry shortly')
            return
        self.dispatch(msg)

    def on_manual_goal(self, msg):
        self.manual_until = time.monotonic() + 2.0
        self.set_state('manual', 'manual_goal', 'manual_override',
                       'Foxglove/manual goal owns navigation')
        # Release a person task; bt_navigator receives the manual goal itself.
        self.call(self.vision_client, 'stop', execute=False, preview=False,
                      wait_s=0.5)

    def on_stop(self, msg):
        if msg.data:
            self.set_state('coordinator', 'stop', 'stopping', 'stop_request')
            self.call(self.vision_client, 'stop', execute=False, preview=False,
                      wait_s=0.5)
            self.call(self.nav_client, 'stop', execute=False, preview=False,
                      wait_s=0.5)

    def on_vision_status(self, msg):
        if msg.state in ('failed', 'arrived', 'stopped',
                         'target_reacquire_timeout'):
            self.set_state('vision', msg.task, msg.state, msg.detail,
                           msg.request_id)

    def on_nav_status(self, msg):
        if msg.owner == 'nav' and msg.state in (
                'arrived', 'failed', 'cancelled', 'manual_override'):
            self.set_state('nav', msg.task, msg.state, msg.detail,
                           msg.request_id)

    def on_voice_status(self, msg):
        if msg.state == self.voice_state and self.owner == 'voice':
            return
        self.voice_state = msg.state
        if msg.state == 'confirming' and self.owner in ('idle', 'voice'):
            self.set_state('voice', msg.task, 'confirming', msg.detail,
                           msg.request_id)
        elif msg.state in ('timeout', 'cancelled') and self.owner == 'voice':
            self.set_state('voice', msg.task, msg.state, msg.detail,
                           msg.request_id)
        # 'confirmed' is followed by the /decision/set_task service dispatch;
        # do not overwrite the coordinator's follow state with it.

    def start_initial_task(self):
        if self._initial_timer is not None:
            self.destroy_timer(self._initial_timer)
            self._initial_timer = None
        if self.initial_task in ('follow', 'preview'):
            self.set_state('coordinator', 'follow', 'dispatching',
                           f'initial task {self.initial_task}')
            self.call(self.vision_client, 'follow', {},
                      execute=self.execute and self.initial_task == 'follow',
                      preview=self.initial_task == 'preview')
        elif self.initial_task not in ('none', ''):
            self.get_logger().warn(
                f'unknown initial_task {self.initial_task!r}')

    def on_set_task(self, request, response):
        task = (request.task or '').strip().lower()
        try:
            params = json.loads(request.params_json or '{}')
        except ValueError:
            response.success, response.message = False, 'bad params_json'
            return response
        if task in ('follow', 'preview', 'approach_user'):
            vision_task = vision_task_for(task)
            self.set_state('coordinator', task, 'dispatching',
                           f'set_task -> {vision_task}')
            self.call(self.vision_client, vision_task, params,
                      execute=request.execute, preview=request.preview)
            response.success, response.message = True, 'vision task queued'
        elif task in ('move_relative', 'approach_pose'):
            self.set_state('coordinator', task, 'dispatching', 'set_task')
            self.call(self.nav_client, task, params)
            response.success, response.message = True, 'nav task queued'
        elif task in ('stop', 'none', 'cancel'):
            self.set_state('coordinator', 'stop', 'stopping', 'set_task')
            self.call(self.vision_client, 'stop', execute=False, preview=False,
                      wait_s=0.5)
            self.call(self.nav_client, 'stop', execute=False, preview=False,
                      wait_s=0.5)
            response.success, response.message = True, 'stopping'
        else:
            response.success, response.message = False, f'unknown task {task!r}'
        return response


def main():
    rclpy.init()
    node = TaskCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
