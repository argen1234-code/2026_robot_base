#!/usr/bin/env python3
"""Nav2 目标点安全检查/修正。

订阅 /goal_pose（Foxglove/用户入口），发布 /goal_pose_safe 给 bt_navigator：
  · 终点在致命格/未知格：在 max_goal_adjust_m 内找最近自由格，保持朝向；
  · 起点在致命格：调用 local/global costmap 的 clear_entirely 服务，
    然后交给行为树的 spin/backup/clear 恢复链；
  · 终点在膨胀区但非致命格：不修改，交给规划器。

bt_navigator 在 nav2_navigation.launch.py 中把 /goal_pose 重映射到
/goal_pose_safe，因此用户仍只发 /goal_pose。
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def qrot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # 用四元数旋转向量
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class GoalSanitizer(Node):
    def __init__(self):
        super().__init__('goal_sanitizer')
        self.declare_parameter('goal_in_topic', '/goal_pose')
        self.declare_parameter('goal_out_topic', '/goal_pose_safe')
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('max_goal_adjust_m', 1.0)
        self.declare_parameter('free_threshold', 20)
        self.declare_parameter('lethal_threshold', 90)
        self.declare_parameter('clear_start_costmap', True)

        self.max_adjust = float(self.get_parameter('max_goal_adjust_m').value)
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.lethal_threshold = int(
            self.get_parameter('lethal_threshold').value)
        self.clear_start = bool(
            self.get_parameter('clear_start_costmap').value)

        self.costmap = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('global_costmap_topic').value,
            self.on_costmap, qos)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_in_topic').value,
            self.on_goal, 10)
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter('goal_out_topic').value, 10)
        self.status_pub = self.create_publisher(
            String, '/goal_sanitizer_status', 10)

        self.clear_local = self.create_client(
            ClearEntireCostmap,
            'local_costmap/clear_entirely_local_costmap')
        self.clear_global = self.create_client(
            ClearEntireCostmap,
            'global_costmap/clear_entirely_global_costmap')

        self.get_logger().info(
            f'goal_sanitizer: adjust<= {self.max_adjust:.2f} m, '
            f'free<={self.free_threshold}, lethal>={self.lethal_threshold}')

    def status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def on_costmap(self, msg):
        self.costmap = msg

    def cell_cost(self, x, y):
        m = self.costmap
        if m is None:
            return None
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        col = int(math.floor((x - ox) / res))
        row = int(math.floor((y - oy) / res))
        if col < 0 or row < 0 or col >= m.info.width or row >= m.info.height:
            return None
        return int(m.data[row * m.info.width + col])

    def call_clear(self, client, name):
        if not client.service_is_ready():
            self.get_logger().warn(f'{name} 服务未就绪，跳过清图')
            return
        future = client.call_async(ClearEntireCostmap.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(f'{name} 清图请求已发送'))

    def transform_goal(self, goal):
        frame = goal.header.frame_id or 'map'
        if frame == 'map':
            return goal
        # Foxglove 的 Fixed frame 是 base_link 时，goal 的 stamp 可能比
        # TF 缓存早/晚一点；先按 stamp 查，失败再回退到最新 TF。
        tf = None
        last_exc = None
        for stamp in (goal.header.stamp, Time()):
            try:
                tf = self.buffer.lookup_transform(
                    'map', frame, stamp, timeout=Duration(seconds=0.2))
                break
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
        if tf is None:
            self.get_logger().warn(
                f'goal 从 {frame} 转到 map 失败: {last_exc}')
            return None
        q = (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w)
        t = tf.transform.translation
        p = goal.pose.position
        rp = qrot(q, (p.x, p.y, p.z))
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.pose.position.x = t.x + rp[0]
        out.pose.position.y = t.y + rp[1]
        out.pose.position.z = t.z + rp[2]
        oq = (goal.pose.orientation.x, goal.pose.orientation.y,
              goal.pose.orientation.z, goal.pose.orientation.w)
        nq = qmul(q, oq)
        out.pose.orientation.x, out.pose.orientation.y = nq[0], nq[1]
        out.pose.orientation.z, out.pose.orientation.w = nq[2], nq[3]
        return out

    def find_free_goal(self, gx, gy):
        m = self.costmap
        if m is None:
            return None
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        max_cells = max(1, int(round(self.max_adjust / res)))
        gcol = int(math.floor((gx - ox) / res))
        grow = int(math.floor((gy - oy) / res))
        best = None
        best_d2 = None
        for dr in range(-max_cells, max_cells + 1):
            for dc in range(-max_cells, max_cells + 1):
                col, row = gcol + dc, grow + dr
                if col < 0 or row < 0 or col >= m.info.width \
                        or row >= m.info.height:
                    continue
                cost = int(m.data[row * m.info.width + col])
                if cost < 0 or cost > self.free_threshold:
                    continue
                x = ox + (col + 0.5) * res
                y = oy + (row + 0.5) * res
                d2 = (x - gx) ** 2 + (y - gy) ** 2
                if best_d2 is None or d2 < best_d2:
                    best_d2 = d2
                    best = (x, y)
        return best

    def on_goal(self, goal):
        original = goal
        goal = self.transform_goal(goal)
        if goal is None:
            # 变换失败时不要丢目标：/goal_pose_safe 的 bt_navigator
            # 自己也会做 TF 变换；原样转发，避免 Foxglove Fixed frame
            # 为 base_link 时目标被静默丢弃。
            self.status('goal 无法变换到 map，原样转发给 bt_navigator')
            original.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(original)
            return

        # 起点在致命格：清图（行为树随后还会 clear/spin/backup）
        if self.clear_start and self.costmap is not None:
            try:
                tf = self.buffer.lookup_transform(
                    'map', 'base_footprint', Time(), timeout=Duration(seconds=0.2))
                sx = tf.transform.translation.x
                sy = tf.transform.translation.y
                start_cost = self.cell_cost(sx, sy)
                if start_cost is not None and start_cost >= self.lethal_threshold:
                    self.status(
                        f'起点 cost={start_cost} 在致命格，清理局部/全局 costmap')
                    self.call_clear(self.clear_local, 'local_costmap')
                    self.call_clear(self.clear_global, 'global_costmap')
            except Exception:  # noqa: BLE001
                pass

        gx = goal.pose.position.x
        gy = goal.pose.position.y
        cost = self.cell_cost(gx, gy)
        if cost is None:
            self.status('终点在 global costmap 外，交给规划器/恢复链')
        elif cost >= self.lethal_threshold:
            free = self.find_free_goal(gx, gy)
            if free is None:
                self.status(
                    f'终点 cost={cost} 在致命格，且 {self.max_adjust:.1f} m '
                    '内无自由格，拒绝目标')
                return
            self.status(
                f'终点 cost={cost} 在致命格，修正到 '
                f'({free[0]:+.2f},{free[1]:+.2f})')
            goal.pose.position.x, goal.pose.position.y = free
        elif cost < 0:
            self.status('终点在未知格，保留原目标（planner allow_unknown）')
        else:
            self.status(f'终点 cost={cost}，保留原目标')

        goal.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(goal)


def main():
    rclpy.init()
    node = GoalSanitizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
