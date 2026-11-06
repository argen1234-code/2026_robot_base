#!/usr/bin/env python3
"""Fail-closed ROS Twist / STM32 USB CDC bridge.

下行：Nav2 速度 -> AA 55 速度帧（可选 ARM 后才会真正驱动电机）。
上行：CC 55 遥测 -> /chassis/imu（sensor_msgs/Imu）
                     /chassis/wheel_speed（Float32MultiArray，rad/s，左/右）
"""

import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int8, String
import serial
from tf2_ros import Buffer, TransformListener

# 与视觉决策层共用同一套居中/足迹安全逻辑。
try:
    from robot_base_decision.person_centering import FootprintGuard, centered_command
    _HAS_CENTERING = True
except Exception:  # noqa: BLE001 - centering is optional; raw Nav2 must still work
    # 兼容迁移前的 tools/ 布局。
    _TOOLS = Path(__file__).resolve().parents[4] / 'tools'
    if _TOOLS.is_dir():
        sys.path.insert(0, str(_TOOLS))
    try:
        from person_centering import FootprintGuard, centered_command
        _HAS_CENTERING = True
    except Exception:  # noqa: BLE001
        FootprintGuard = centered_command = None
        _HAS_CENTERING = False

from chassis_protocol import (
    FLAG_ARMED,
    FLAG_CMD_TIMEOUT,
    FLAG_FATAL,
    FLAG_IMU_ONLINE,
    FLAG_MOTORS_READY,
    SERVICE_ARM,
    SERVICE_DISARM,
    TelemetryStream,
    result_name,
    service_frame,
    speed_frame,
)


def _quaternion(roll, pitch, yaw):
    """欧拉角（rad，ZYX）-> 四元数 (x, y, z, w)。"""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class ChassisBridge(Node):
    def __init__(self):
        super().__init__('chassis_bridge')
        self.declare_parameter('port', '/dev/stm32')
        self.declare_parameter('enabled', False)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('max_linear', 0.20)
        self.declare_parameter('max_angular', 0.45)
        self.declare_parameter('imu_frame_id', 'base_link')
        self.declare_parameter('imu_topic', '/chassis/imu')
        self.declare_parameter('wheel_topic', '/chassis/wheel_speed')
        self.declare_parameter('target_topic', '/chassis/wheel_target')
        self.declare_parameter('current_topic', '/chassis/wheel_current')
        # 静止标定：从原始角度/陀螺中减去这些零偏（见 config/imu_calibration.yaml）。
        self.declare_parameter('imu_roll_zero_deg', 0.0)
        self.declare_parameter('imu_pitch_zero_deg', 0.0)
        self.declare_parameter('imu_yaw_zero_deg', 0.0)
        self.declare_parameter('imu_gyro_bias_dps', [0.0, 0.0, 0.0])
        self.port = self.get_parameter('port').value
        self.timeout = self.get_parameter('command_timeout').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.imu_roll_zero = float(self.get_parameter('imu_roll_zero_deg').value)
        self.imu_pitch_zero = float(self.get_parameter('imu_pitch_zero_deg').value)
        self.imu_yaw_zero = float(self.get_parameter('imu_yaw_zero_deg').value)
        imu_bias = list(self.get_parameter('imu_gyro_bias_dps').value)
        self.imu_gyro_bias = ([float(v) for v in imu_bias]
                              if len(imu_bias) == 3 else [0.0, 0.0, 0.0])
        if (os.path.realpath(self.port) == os.path.realpath('/dev/ydlidar') or
                self.timeout <= 0 or self.timeout >= 0.5):
            raise ValueError('invalid port or timeout (must be shorter than MCU 500 ms watchdog)')

        # launch 传进来的可能是字符串 'true'/'false'，统一解析。
        enabled = self.get_parameter('enabled').value
        if isinstance(enabled, str):
            enabled = enabled.strip().lower() in ('1', 'true', 'yes', 'on')
        self.enabled = bool(enabled)

        self.device = None
        self.stream = TelemetryStream()
        self.last_command = None
        self.command = (0.0, 0.0)
        self.remote_command = (0.0, 0.0)
        self.last_remote = None
        self.mode = 3

        self.armed = False
        self.armed_since = None
        self.motors_ready = False
        self.last_result = None
        self.last_arm_attempt = 0.0
        self.last_telemetry = None
        self.telemetry_count = 0

        # 找人任务的画面居中叠加：只在 /person_nav/status 表明正在靠近时生效。
        self.centering_cfg = {}
        self.center_enabled = False
        if _HAS_CENTERING:
            cfg_path = Path(__file__).resolve().parents[4] / 'config' / 'person_approach.json'
            try:
                self.centering_cfg = json.loads(cfg_path.read_text())
                self.center_enabled = bool(self.centering_cfg.get('center_person', False))
            except (OSError, ValueError) as exc:
                self.get_logger().warn(f'Person centering config unavailable: {exc}')
        self.person_status = {}
        self.person_status_at = 0.0
        self.centering_target = {}
        self.centering_at = 0.0
        self.local_guard = None
        self.local_frame = None
        self.local_grid_at = 0.0
        self.last_centering_yaw = 0.0
        self.last_centering_at = time.monotonic()
        self.center_block = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.imu_pub = self.create_publisher(
            Imu, self.get_parameter('imu_topic').value, 10)
        self.wheel_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('wheel_topic').value, 10)
        self.target_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('target_topic').value, 10)
        self.current_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('current_topic').value, 10)

        # ⚠️ 订阅的是 Nav2 整条速度链的【末端话题】，不是笼统的 cmd_vel。
        #
        # nav2_bringup/launch/navigation_launch.py 里给 controller_server 和
        # velocity_smoother 都加了 remappings + [('cmd_vel', 'cmd_vel_nav')]，
        # 于是实际链路是：
        #     controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
        # 没有任何节点会往 /cmd_vel 上发东西，订阅 /cmd_vel 会一帧都收不到，
        # 表现为 RViz 里点了目标点车却不动。
        self.create_subscription(Twist, 'cmd_vel_smoothed', self.on_command, 10)
        self.create_subscription(Twist, '/remote_cmd_vel', self.on_remote, 10)
        self.create_subscription(Int8, '/robot_mode', self.on_mode, 10)
        self.create_subscription(String, '/person_nav/status', self.on_person_status, 1)
        self.create_subscription(String, '/person_nav/centering_target', self.on_centering, 1)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.on_local_grid,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.mode_pub = self.create_publisher(Int8, '/robot_mode', 10)
        self.mode_pub.publish(Int8(data=self.mode))
        self.create_timer(0.05, self.tick)
        if not self.enabled:
            self.get_logger().warn(
                'Motor output disabled; telemetry stays on, set motor_enable:=true to ARM')
        if not _HAS_CENTERING:
            self.get_logger().warn('Person centering unavailable (person_centering import failed)')
        elif not self.center_enabled:
            self.get_logger().info('Person centering disabled by config')

    # ------------------------------------------------------------------ 输入
    def on_command(self, msg):
        if (abs(msg.linear.y) > 1e-6 or abs(msg.linear.z) > 1e-6 or
                abs(msg.angular.x) > 1e-6 or abs(msg.angular.y) > 1e-6):
            self.last_command = None
            self.get_logger().warn('Unsupported velocity axis; stopping')
            return
        try:
            speed_frame(msg.linear.x, msg.angular.z, self.max_linear, self.max_angular)
        except ValueError:
            self.last_command = None
            self.get_logger().warn('Invalid/reverse velocity; stopping')
            return
        self.command = (msg.linear.x, msg.angular.z)
        self.last_command = time.monotonic()

    def on_mode(self, msg):
        if msg.data not in (2, 3):
            self.get_logger().warn('Unsupported mode; stopping')
            self.mode = 2
        elif msg.data != self.mode:
            self.mode = msg.data
        else:
            return
        self.last_command = None
        self.last_remote = None

    def on_remote(self, msg):
        axes = (msg.linear.x, msg.angular.z, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y)
        if not all(math.isfinite(v) for v in axes) or any(abs(v) > 1e-6 for v in axes[2:]):
            self.last_remote = None
            self.get_logger().warn('Invalid remote velocity; stopping')
            return
        previous = self.remote_command
        self.remote_command = (msg.linear.x, msg.angular.z)
        self.last_remote = time.monotonic()
        # Stop the direct-control path in this callback, without waiting for
        # the next 50ms USB tick. Ordinary navigation keeps its own path.
        if (self.mode == 2 and self.remote_command == (0.0, 0.0)
                and previous != (0.0, 0.0) and self.device is not None):
            try:
                self.device.write(speed_frame(0.0, 0.0, self.max_linear,
                                              self.max_angular, mode=2))
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'MCU immediate stop failed: {exc}')
                self._close()

    # ------------------------------------------------------------- 遥测解析
    def on_telemetry(self, msg):
        now = time.monotonic()
        self.last_telemetry = now
        self.telemetry_count += 1

        was_armed = self.armed
        self.armed = bool(msg['flags'] & FLAG_ARMED)
        if self.armed and not was_armed:
            self.armed_since = now
        elif not self.armed:
            self.armed_since = None
        self.motors_ready = bool(msg['flags'] & FLAG_MOTORS_READY)
        if msg['flags'] & FLAG_FATAL:
            self.get_logger().error('MCU reports fatal fault', throttle_duration_sec=5.0)
        if msg['last_result'] != self.last_result:
            self.last_result = msg['last_result']
            self.get_logger().info(
                f"MCU result={result_name(self.last_result)} "
                f"motors_ready={self.motors_ready} armed={self.armed}")
        # ARM 的同一拍固件会把 cmd_tick 清零，因此刚使能时必然先报一次
        # CMD_TIMEOUT；等节点开始发零速帧就会清掉，这里只报持续超时。
        if (msg['flags'] & FLAG_CMD_TIMEOUT and self.enabled and self.armed and
                self.armed_since is not None and now - self.armed_since > 1.0):
            self.get_logger().warn(
                'MCU command watchdog expired (no fresh cmd_vel)', throttle_duration_sec=2.0)

        if self.enabled and not self.armed and self.last_result not in (None, 16, 0):
            self.get_logger().warn(
                f"ARM not accepted: {result_name(self.last_result)} "
                '(motors offline? check CAN/power)', throttle_duration_sec=3.0)

        if msg['flags'] & FLAG_IMU_ONLINE:
            self.publish_imu(msg)
        else:
            self.get_logger().warn('IMU offline; no /chassis/imu frame',
                                   throttle_duration_sec=5.0)

        wheel = Float32MultiArray()
        wheel.data = [msg['left_speed'], msg['right_speed']]
        self.wheel_pub.publish(wheel)

        # 目标轮速单独发一份：odom_stabilizer 用它判断"是否有运动意图"，
        # 避免把静止时的轮速噪声（实测 ±0.12 rad/s）误判为运动。
        target = Float32MultiArray()
        target.data = [msg['left_target'], msg['right_target']]
        self.target_pub.publish(target)

        # 左右轮相电流（A）：用于诊断原地大角度旋转时的扭矩/打滑。
        current = Float32MultiArray()
        current.data = [msg['left_current'], msg['right_current']]
        self.current_pub.publish(current)

    def publish_imu(self, msg):
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_frame_id

        roll = math.radians((msg['roll_deg'] - self.imu_roll_zero + 180.0) % 360.0 - 180.0)
        pitch = math.radians(msg['pitch_deg'] - self.imu_pitch_zero)
        yaw = math.radians((msg['yaw_deg'] - self.imu_yaw_zero + 180.0) % 360.0 - 180.0)
        qx, qy, qz, qw = _quaternion(roll, pitch, yaw)
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.orientation_covariance = [0.02, 0.0, 0.0,
                                      0.0, 0.02, 0.0,
                                      0.0, 0.0, 0.05]

        gx, gy, gz = msg['gyro_deg_s']
        imu.angular_velocity.x = math.radians(gx - self.imu_gyro_bias[0])
        imu.angular_velocity.y = math.radians(gy - self.imu_gyro_bias[1])
        imu.angular_velocity.z = math.radians(gz - self.imu_gyro_bias[2])
        imu.angular_velocity_covariance = [0.002, 0.0, 0.0,
                                           0.0, 0.002, 0.0,
                                           0.0, 0.0, 0.002]

        ax, ay, az = msg['accel_g']
        imu.linear_acceleration.x = ax * 9.80665
        imu.linear_acceleration.y = ay * 9.80665
        imu.linear_acceleration.z = az * 9.80665
        imu.linear_acceleration_covariance = [0.02, 0.0, 0.0,
                                              0.0, 0.02, 0.0,
                                              0.0, 0.0, 0.02]
        self.imu_pub.publish(imu)

    # ------------------------------------------------------------- 周期任务
    def _open(self):
        try:
            # The MCU enumerates as STM32 USB CDC (0483:5740) -> /dev/stm32.
            self.device = serial.Serial(self.port, baudrate=115200, timeout=0,
                                        write_timeout=0.1, exclusive=True)
            self.stream = TelemetryStream()
            self.armed = False
            self.last_result = None
            self.last_telemetry = None
            self.last_arm_attempt = 0.0
            self.get_logger().info(
                f'STM32 connected on {self.port} ({self.device.name}); '
                f'motor output {"ENABLED" if self.enabled else "disabled"}')
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Cannot open MCU {self.port}: {exc}',
                                    throttle_duration_sec=5.0)

    def _read_telemetry(self):
        try:
            for _ in range(8):
                data = self.device.read(256)
                if not data:
                    break
                for msg in self.stream.feed(data):
                    self.on_telemetry(msg)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'MCU read failed: {exc}')
            self._close()

    # ------------------------------------------------------- 找人画面居中
    def on_person_status(self, msg):
        try:
            self.person_status = json.loads(msg.data)
            self.person_status_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_centering(self, msg):
        try:
            self.centering_target = json.loads(msg.data)
            self.centering_at = time.monotonic()
        except (ValueError, TypeError):
            self.centering_target = {}

    def on_local_grid(self, msg):
        if not _HAS_CENTERING or msg.info.resolution <= 0 or not msg.header.frame_id:
            return
        q = msg.info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.local_guard = FootprintGuard(
            msg.data, msg.info.width, msg.info.height, msg.info.resolution,
            (msg.info.origin.position.x, msg.info.origin.position.y), yaw)
        self.local_frame = msg.header.frame_id
        self.local_grid_at = time.monotonic()

    def person_task_active(self, now):
        status = self.person_status
        return (self.center_enabled and now - self.person_status_at <= .75 and
                bool(status.get('execute')) and bool(status.get('active')) and
                not status.get('paused') and
                status.get('state') in ('navigating', 'target_temporarily_lost'))

    def centered_velocity(self, linear, angular, now):
        target = self.centering_target
        try:
            age = time.time() - float(target.get('source_stamp', 0.))
        except (TypeError, ValueError):
            age = 0.
        if (now - self.centering_at > .35 or not target.get('enabled') or
                not target.get('active') or
                not -.1 <= age <= self.centering_cfg.get('centering_max_result_age_s', .5)):
            self.center_block = 'centering_waiting_for_target'
            return None
        if self.local_guard is None or now - self.local_grid_at > .45:
            self.center_block = 'centering_waiting_for_costmap'
            return None
        try:
            v, w, _ = centered_command(linear, angular, target, self.centering_cfg)
            limit = (self.centering_cfg.get('centering_max_angular_accel', .6) *
                     min(.1, max(.001, now - self.last_centering_at)))
            w = min(self.last_centering_yaw + limit,
                    max(self.last_centering_yaw - limit, w))
            # A Nav2 zero command is never permission to turn on its own.
            if abs(linear) < .001 and abs(angular) < .01:
                v = w = 0.
            tf = self.tf_buffer.lookup_transform(
                self.local_frame, 'base_footprint', Time())
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            if not self.local_guard.clear(
                    (t.x, t.y, yaw), v, w,
                    front=self.centering_cfg.get('robot_front_m', .246),
                    half_width=self.centering_cfg.get('robot_half_width_m', .118),
                    horizon=self.centering_cfg.get('centering_horizon_s', 1.2)):
                self.center_block = 'centering_path_blocked'
                return None
            self.last_centering_yaw, self.last_centering_at = w, now
            self.center_block = None
            return v, w
        except (KeyError, TypeError, ValueError):
            self.center_block = 'centering_waiting_for_target'
        except Exception:
            self.center_block = 'centering_waiting_for_tf'
        return None

    def _nav_command(self):
        """返回 ROS 约定下的 (linear_x, angular_z)；失效时归零。"""
        if not self.enabled:
            return 0.0, 0.0
        if self.mode == 2:
            fresh = (self.last_remote is not None and
                     time.monotonic() - self.last_remote < self.timeout)
            return self.remote_command if fresh else (0.0, 0.0)
        fresh = (self.last_command is not None and
                 time.monotonic() - self.last_command < self.timeout)
        return self.command if fresh else (0.0, 0.0)

    def _command(self):
        linear, angular = self._nav_command()
        now = time.monotonic()
        if not self.person_task_active(now):
            self.last_centering_yaw = 0.0
            self.last_centering_at = now
            self.center_block = None
            return linear, angular
        centered = self.centered_velocity(linear, angular, now)
        if centered is None:
            self.get_logger().warn(
                f'Person centering blocked: {self.center_block}',
                throttle_duration_sec=2.0)
            return 0.0, 0.0
        return centered

    def tick(self):
        if self.device is None:
            self._open()
            if self.device is None:
                return

        self._read_telemetry()
        if self.device is None:
            return

        now = time.monotonic()
        if (self.last_telemetry is not None and
                now - self.last_telemetry > 1.0):
            self.get_logger().warn('No MCU telemetry for >1 s',
                                   throttle_duration_sec=5.0)

        linear, angular = self._command()

        try:
            # 只有显式 motor_enable:=true 才发 ARM；否则只发零速帧，
            # 保持下行链路可验证，但电机绝不会使能。
            if self.enabled and not self.armed and now - self.last_arm_attempt >= 1.0:
                self.last_arm_attempt = now
                self.device.write(service_frame(SERVICE_ARM))

            # ⚠️ 角速度取反：ROS REP-103 左转为正，而固件 app_usb.c 的
            #    约定是“下行 wz < 0 表示左转”。固件再取反一次得到 ccw。
            #    线速度同样由 speed_frame 取反（固件 forward = -vx）。
            self.device.write(speed_frame(linear, -angular, self.max_linear,
                                          self.max_angular, mode=self.mode))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'MCU write failed: {exc}')
            self._close()

    def _close(self):
        if self.device is not None:
            try:
                self.device.close()
            except (serial.SerialException, OSError):
                pass
            self.device = None

    def stop(self):
        if self.device is not None:
            try:
                # self.armed 来自固件遥测，只代表 USB 这条链路；即使本次
                # motor_enable=false，也要清掉上一次会话可能留下的 ARM。
                if self.enabled or self.armed:
                    self.device.write(service_frame(SERVICE_DISARM))
                self.device.write(speed_frame(0.0, 0.0, mode=self.mode))
                self.device.flush()
            except (serial.SerialException, OSError):
                pass
            self._close()


def main():
    rclpy.init()
    node = ChassisBridge()
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
