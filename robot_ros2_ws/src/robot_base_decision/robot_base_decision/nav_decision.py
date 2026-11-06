#!/usr/bin/env python3
"""Navigation-stack decision layer.

Receives confirmed relative/absolute tasks from the top-level coordinator,
sanitizes the goal through goal_sanitizer, and sends a Nav2 NavigateToPose
action.  Manual /goal_pose_safe goals always take ownership and cancel the
decision-layer goal.
"""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from robot_base_msgs.msg import TaskStatus
from robot_base_msgs.srv import SanitizeGoal, SetTask

from .priority import MANUAL_GOAL


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class NavDecision(Node):
    def __init__(self):
        super().__init__('nav_decision')
        # Create the TF listener lazily: an idle decision layer should not pay
        # for a full TF subscription while no relative task is active.
        self.tf = None
        self.listener = None
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.sanitizer = self.create_client(
            SanitizeGoal, '/goal_sanitizer/sanitize_goal')

        self.state = 'idle'
        self.detail = ''
        self.task = 'none'
        self.request_id = ''
        self.action_handle = None
        self.manual_until = 0.0

        self.create_service(SetTask, '/decision/nav_task', self.on_task)
        self.create_subscription(
            PoseStamped, '/goal_pose_safe', self.on_manual_goal, 10)
        self.create_subscription(Bool, '/stop_request', self.on_stop, 1)
        self.status_pub = self.create_publisher(
            TaskStatus, '/decision/nav_status', 5)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info('nav decision ready')

    def set_state(self, state, detail=''):
        self.state, self.detail = state, detail
        self.publish_status()
        if state not in ('idle', 'arrived'):
            self.get_logger().info(f'nav decision: {state} {detail}')

    def publish_status(self):
        msg = TaskStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.owner = 'nav'
        msg.task = self.task
        msg.state = self.state
        msg.detail = self.detail
        msg.request_id = self.request_id
        self.status_pub.publish(msg)

    def cancel(self, reason):
        self.task = 'none'
        self.set_state('cancelled' if reason != 'manual_override' else reason,
                       reason)
        if self.action_handle is not None:
            self.action_handle.cancel_goal_async()
            self.action_handle = None

    def on_stop(self, msg):
        if msg.data:
            self.cancel('stopped')

    def on_manual_goal(self, msg):
        # bt_navigator receives this goal itself; we only release ownership.
        self.manual_until = time.monotonic() + 2.0
        self.cancel('manual_override')
        self.set_state('manual_override', 'manual goal owns navigation')

    def _ensure_tf(self):
        if self.tf is None:
            self.tf = Buffer(cache_time=Duration(seconds=5.0))
            self.listener = TransformListener(self.tf, self)
        return self.tf

    def move_relative_goal(self, params):
        direction = str(params.get('direction', 'forward')).lower()
        distance = float(params.get('distance_m', 1.0))
        yaw_deg = float(params.get('yaw_deg', 0.0) or 0.0)
        table = {'forward': (1.0, 0.0), 'backward': (-1.0, 0.0),
                 'left': (0.0, 1.0), 'right': (0.0, -1.0)}
        if direction not in table or not 0.1 <= distance <= 5.0:
            raise ValueError('direction must be forward/backward/left/right and '
                             'distance_m in [0.1, 5.0]')
        tf_buffer = self._ensure_tf()
        deadline = time.monotonic() + 1.0
        while True:
            try:
                tf = tf_buffer.lookup_transform(
                    'map', 'base_footprint', Time(),
                    timeout=Duration(seconds=0.2))
                break
            except Exception:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)
        x0 = tf.transform.translation.x
        y0 = tf.transform.translation.y
        yaw0 = yaw_from_quat(tf.transform.rotation)
        dx, dy = table[direction]
        ox, oy = dx * distance, dy * distance
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x0 + math.cos(yaw0) * ox - math.sin(yaw0) * oy
        goal.pose.position.y = y0 + math.sin(yaw0) * ox + math.cos(yaw0) * oy
        yaw = yaw0 + math.radians(yaw_deg)
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        return goal

    def on_task(self, request, response):
        task = (request.task or '').strip().lower()
        if time.monotonic() < self.manual_until and task not in ('stop', 'none'):
            response.success = False
            response.message = 'manual goal owns navigation; retry shortly'
            return response
        if task in ('stop', 'none', 'cancel'):
            self.cancel('stopped')
            response.success, response.message = True, 'stopped'
            return response
        try:
            params = json.loads(request.params_json or '{}')
            if task == 'move_relative':
                goal = self.move_relative_goal(params)
            elif task == 'approach_pose':
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = float(params['x'])
                goal.pose.position.y = float(params['y'])
                yaw = math.radians(float(params.get('yaw_deg', 0.0)))
                goal.pose.orientation.z = math.sin(yaw / 2)
                goal.pose.orientation.w = math.cos(yaw / 2)
            else:
                response.success = False
                response.message = f'unsupported task {task!r}'
                return response
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'invalid task: {exc}'
            return response
        self.task, self.request_id = task, request.request_id if hasattr(
            request, 'request_id') else ''
        response.success = True
        response.message = 'goal accepted; sanitizing'
        self.set_state('sanitizing', task)
        self.sanitize_and_send(goal, task)
        return response

    def sanitize_and_send(self, goal, task):
        if not self.sanitizer.service_is_ready():
            self.set_state('failed', 'goal_sanitizer service unavailable')
            return
        req = SanitizeGoal.Request()
        req.goal = goal
        future = self.sanitizer.call_async(req)

        def done(fut):
            try:
                res = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.set_state('failed', f'sanitize call failed: {exc}')
                return
            if not res.success:
                self.set_state('failed', f'sanitized rejected: {res.message}')
                return
            self.send_goal(res.goal, task)
        future.add_done_callback(done)

    def send_goal(self, goal, task):
        if not self.nav_client.server_is_ready():
            self.set_state('failed', 'navigate_to_pose server unavailable')
            return
        request = NavigateToPose.Goal()
        request.pose = goal
        self.set_state('navigating', task)
        future = self.nav_client.send_goal_async(request)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.set_state('failed', f'goal send failed: {exc}')
            return
        if not handle.accepted:
            self.set_state('failed', 'goal rejected by Nav2')
            return
        self.action_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self.set_state('failed', f'result failed: {exc}')
            return
        self.action_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.set_state('arrived', 'goal reached')
        elif status == GoalStatus.STATUS_CANCELED:
            self.set_state('cancelled', 'goal canceled')
        else:
            self.set_state('failed', f'goal status={status}')


def main():
    rclpy.init()
    node = NavDecision()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
