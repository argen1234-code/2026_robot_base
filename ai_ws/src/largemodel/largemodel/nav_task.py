"""
Navigation task node.

Subscribes to a map-frame goal produced by the summon pipeline and drives
Nav2 asynchronously.  It is deliberately a separate process from
``model_service``:

* model/VLM inference is slow and blocking;
* the stop command must remain responsive while a goal is executing;
* Nav2 may be unavailable while the rest of the voice stack still runs.

The node never forwards velocities itself.  Nav2 writes to ``cmd_vel_nav``
and ``motion_gate`` is the only publisher of ``cmd_vel``.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

try:
    from nav2_msgs.action import NavigateToPose
    NAV2_AVAILABLE = True
except ImportError:  # nav2_msgs is not installed yet
    NavigateToPose = None
    NAV2_AVAILABLE = False


class NavTaskNode(Node):
    def __init__(self):
        super().__init__('nav_task')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('goal_topic', 'nav_goal')
        self.declare_parameter('stop_topic', 'stop_request')
        self.declare_parameter('enable_topic', 'nav_enable')
        self.declare_parameter('status_topic', 'nav_status')
        self.declare_parameter('max_goal_age_s', 5.0)
        self.declare_parameter('max_goal_distance_m', 20.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.action_name = self.get_parameter('action_name').value
        goal_topic = self.get_parameter('goal_topic').value
        stop_topic = self.get_parameter('stop_topic').value
        enable_topic = self.get_parameter('enable_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.max_goal_age_s = float(self.get_parameter('max_goal_age_s').value)
        self.max_goal_distance_m = float(
            self.get_parameter('max_goal_distance_m').value
        )

        self.enable_pub = self.create_publisher(Bool, enable_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.enable_pub.publish(Bool(data=False))

        self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 1)
        self.create_subscription(Bool, stop_topic, self.stop_callback, 10)

        self._client = None
        if NAV2_AVAILABLE:
            self._client = ActionClient(self, NavigateToPose, self.action_name)
            self.get_logger().info(
                f"NavTask waiting for action server '{self.action_name}'"
            )
        else:
            self.get_logger().error(
                "nav2_msgs is not installed; navigation goals will be rejected. "
                "Install ros-humble-nav2-msgs / ros-humble-nav2-bringup."
            )

        # Monotonic task id: every callback checks it so a late result from an
        # old goal can never re-enable motion after a stop.
        self.task_id = 0
        self._goal_handle = None
        self._last_feedback_log_ns = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_timer(1.0, self._server_watchdog)
        self.get_logger().info(
            f"NavTask ready (frame={self.map_frame}, goal_topic={goal_topic}, "
            f"stop_topic={stop_topic})"
        )

    # ------------------------------------------------------------------
    # Goal handling
    # ------------------------------------------------------------------
    def goal_callback(self, msg: PoseStamped):
        if self._client is None:
            self._publish_status("导航模块还没准备好，我暂时不能移动。")
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self._publish_status("导航目标坐标系不正确，已拒绝移动。")
            self.get_logger().error(
                f"Rejecting goal in frame '{msg.header.frame_id}', "
                f"expected '{self.map_frame}'"
            )
            return

        p = msg.pose.position
        q = msg.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            self._publish_status("导航目标无效，已拒绝移动。")
            return

        if self._goal_is_stale(msg):
            self._publish_status("导航目标已过期，请再召唤一次。")
            return

        distance = self._distance_from_robot(msg)
        if distance is None:
            self._publish_status("定位系统还没准备好，我暂时不能移动。")
            return
        if self.max_goal_distance_m > 0.0 and distance > self.max_goal_distance_m:
            self._publish_status("目标距离太远，已拒绝移动。")
            self.get_logger().error(
                f"Rejecting goal {distance:.1f} m away "
                f"(limit {self.max_goal_distance_m:.1f} m)"
            )
            return

        # A new goal supersedes the old one.
        self.task_id += 1
        task_id = self.task_id
        self._cancel_current()
        self._goal_handle = None

        if not self._client.server_is_ready():
            self.get_logger().info(
                f"Action server '{self.action_name}' not ready, waiting..."
            )
            if not self._client.wait_for_server(timeout_sec=2.0):
                self.enable_pub.publish(Bool(data=False))
                self._publish_status("导航服务未启动，我暂时不能移动。")
                return

        self.enable_pub.publish(Bool(data=True))

        goal = NavigateToPose.Goal()
        goal.pose = msg

        self.get_logger().info(
            f"Sending Nav2 goal (task {task_id}): "
            f"x={p.x:.2f}, y={p.y:.2f}, distance={distance:.2f} m"
        )
        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._feedback_cb(task_id, feedback),
        )
        send_future.add_done_callback(
            lambda future: self._goal_response_cb(task_id, future)
        )

    def _goal_response_cb(self, task_id, future):
        if task_id != self.task_id:
            self.get_logger().info(f"Ignoring late goal response for task {task_id}")
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.enable_pub.publish(Bool(data=False))
            self._publish_status(f"导航目标发送失败：{exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.enable_pub.publish(Bool(data=False))
            self._publish_status("导航目标被拒绝，请检查目标是否可达。")
            return

        self._goal_handle = goal_handle
        self.get_logger().info(f"Nav2 accepted task {task_id}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self._result_cb(task_id, future)
        )

    def _result_cb(self, task_id, future):
        if task_id != self.task_id:
            self.get_logger().info(f"Ignoring late result for task {task_id}")
            return

        self._goal_handle = None
        self.enable_pub.publish(Bool(data=False))

        try:
            status = future.result().status
        except Exception as exc:
            self._publish_status(f"导航结果异常：{exc}")
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status("我已到达，请投放垃圾。")
            self.get_logger().info(f"Nav task {task_id} succeeded")
        elif status == GoalStatus.STATUS_CANCELED:
            # The stop path already acknowledged the user; do not speak twice.
            self.get_logger().info(f"Nav task {task_id} canceled")
        else:
            self._publish_status("我没能到达，请检查通道或再召唤一次。")
            self.get_logger().warning(f"Nav task {task_id} failed, status={status}")

    def _feedback_cb(self, task_id, feedback_msg):
        if task_id != self.task_id:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_feedback_log_ns < 2_000_000_000:
            return
        self._last_feedback_log_ns = now_ns

        try:
            remaining = feedback_msg.feedback.distance_remaining
            self.get_logger().info(f"Nav task {task_id}: {remaining:.2f} m remaining")
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Stop / validation
    # ------------------------------------------------------------------
    def stop_callback(self, msg: Bool):
        if not msg.data:
            return

        self.get_logger().info("Stop requested: disabling motion and canceling goal")
        self.task_id += 1
        self.enable_pub.publish(Bool(data=False))
        self._cancel_current()

    def _cancel_current(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f"Failed to cancel goal: {exc}")
            self._goal_handle = None

    def _goal_is_stale(self, msg: PoseStamped) -> bool:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return False
        try:
            goal_time = Time.from_msg(stamp)
            age = (self.get_clock().now() - goal_time).nanoseconds / 1e9
            return age > self.max_goal_age_s
        except Exception:
            return False

    def _distance_from_robot(self, msg: PoseStamped):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
            robot = transform.transform.translation
            dx = msg.pose.position.x - robot.x
            dy = msg.pose.position.y - robot.y
            return math.hypot(dx, dy)
        except Exception as exc:
            self.get_logger().warning(f"TF lookup for goal validation failed: {exc}")
            return None

    def _server_watchdog(self):
        if self._client is None:
            return
        if not self._client.server_is_ready():
            self.get_logger().debug(
                f"Waiting for Nav2 action server '{self.action_name}'"
            )

    def _publish_status(self, text: str):
        self.get_logger().info(f"Nav status: {text}")
        self.status_pub.publish(String(data=text))

    def shutdown(self):
        self.task_id += 1
        try:
            if rclpy.ok():
                self.enable_pub.publish(Bool(data=False))
        except Exception:
            pass
        self._cancel_current()


def main(args=None):
    rclpy.init(args=args)
    node = NavTaskNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
