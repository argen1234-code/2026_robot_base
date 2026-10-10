#!/usr/bin/env python3
"""Fail-closed ROS Twist to STM32 USB CDC bridge (双向：下行速度 + 上行遥测).

除了原有的"把 ROS Twist 转成固件的 12 字节速度帧下发"之外，本节点现在还
【读取】固件的 183 字节遥测帧，把轮速与 IMU 发成 ROS 话题，供
robot_localization 的 EKF 融合：

    /wheel/odom   nav_msgs/Odometry  四轮计数算出的轮速里程计
    /imu/data     sensor_msgs/Imu    陀螺角速度(+加计，仅诊断)
    /imu/gyro_bias std_msgs/Float32  实时陀螺零偏估计（诊断用，可读出后硬写）

⚠️ 为什么读也放在这个节点里：`/dev/stm32` 被本节点以 exclusive=True 独占，
   一个进程只能有一个读者，否则两边各读到一半字节流、谁都解不出帧。
   （另一个方案是让 `motor_enable:=false` 把端口让给探针 —— 那样就没有
     /imu/data 与 /wheel/odom，EKF 会退化成"只有 rf2o"、静止漂移回归。）

⚠️ 收帧线程必须【绝对不干扰】下面 tick() 的 fail-closed 发帧通路：
   只读 self.device、绝不 open/close/替换它、绝不碰
   last_command/command/remote_command/mode、所有异常就地吞掉。
"""

import math
import os
import statistics
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int8
import serial

from chassis_protocol import speed_frame
# 遥测帧解析与单位换算的【唯一一份】实现，与探针共用。
# 该模块 docstring 记着两个坑：car_mode 是 JetsonMode_t；gyro 单位 °/s、acc 单位 g。
from chassis_telemetry import (
    FrameAssembler, FLAG_IMU_VALID, DEG2RAD, G_TO_MS2, WHEEL_TRACK_M,
    wheel_twist_from_counts)


class ChassisBridge(Node):
    def __init__(self):
        super().__init__('chassis_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('enabled', False)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('max_linear', 0.20)
        self.declare_parameter('max_angular', 0.45)
        self.port = self.get_parameter('port').value
        self.timeout = self.get_parameter('command_timeout').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        if (os.path.realpath(self.port) == os.path.realpath('/dev/ydlidar') or
                self.timeout <= 0 or self.timeout >= 0.5):
            raise ValueError('invalid port or timeout (must be shorter than MCU 500 ms watchdog)')
        self.enabled = self.get_parameter('enabled').value
        # ---- 上行遥测（轮速/IMU -> ROS）相关参数 ----
        self.declare_parameter('publish_sensors', True)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('wheel_odom_topic', '/wheel/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        # ⚠️ 未标定！单位是"每米行程累加多少个每周期计数"（Σ c_mean），与周期无关。
        #    占位值 12000 反推自固件【命令】尺度：ROS_LINE_MAX_SPEED=120 counts
        #    是"每 10 ms 周期"，1 秒 = 100 个周期 -> 120*100 = 12000。
        #    但实测控制环周期是 35~71 ms（不是 10 ms），所以占位值预计会偏约 3.5 倍，
        #    必须按 README 的直线标定步骤实测修正。
        self.declare_parameter('wheel_counts_per_mps', 12000.0)
        self.declare_parameter('wheel_track', WHEEL_TRACK_M)
        # 逐轮符号的整体翻转，默认 False（=按 wheel_twist_from_counts 里已经
        # 内置好的固件约定解释）。留着它只是台架排查用的逃生阀：
        # ⚠️ 不要用它去"修符号"—— 它会同时把 vx 和 wz 一起翻转，而本固件只有
        #    【平移】那一路需要取反（推导见 chassis_telemetry.wheel_twist_from_counts
        #    的注释）。平移的取反已经写在该函数里了，正常情况下这里保持 False。
        self.declare_parameter('wheel_invert', False)
        self.declare_parameter('wheel_rest_counts', 1.0)
        # 静止/运动两档协方差（ZUPT 的关键）：静止档必须远小于 rf2o 的 4e-4
        self.declare_parameter('cov_rest_vx', 1e-5)
        self.declare_parameter('cov_move_vx', 2.5e-3)
        self.declare_parameter('cov_vyaw', 1.0)
        # ---- 陀螺零偏（EKF 无零偏状态，必须在这里扣）----
        self.declare_parameter('subtract_gyro_bias', True)
        self.declare_parameter('gyro_bias', float('nan'))   # NaN = 自动估计
        self.declare_parameter('gyro_bias_alpha', 0.002)
        self.declare_parameter('gyro_bias_max_dps', 5.0)
        self.declare_parameter('bias_warmup', 50)
        self.declare_parameter('stationary_gyro_dps', 3.0)
        self.publish_sensors = self.get_parameter('publish_sensors').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.wheel_odom_topic = self.get_parameter('wheel_odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.wheel_counts_per_mps = self.get_parameter('wheel_counts_per_mps').value
        self.wheel_track = self.get_parameter('wheel_track').value
        self.wheel_invert = self.get_parameter('wheel_invert').value
        self.wheel_rest_counts = self.get_parameter('wheel_rest_counts').value
        self.cov_rest_vx = self.get_parameter('cov_rest_vx').value
        self.cov_move_vx = self.get_parameter('cov_move_vx').value
        self.cov_vyaw = self.get_parameter('cov_vyaw').value
        self.subtract_gyro_bias = self.get_parameter('subtract_gyro_bias').value
        self.gyro_bias_param = self.get_parameter('gyro_bias').value
        self.gyro_bias_alpha = self.get_parameter('gyro_bias_alpha').value
        self.gyro_bias_max_dps = self.get_parameter('gyro_bias_max_dps').value
        self.bias_warmup = self.get_parameter('bias_warmup').value
        self.stationary_gyro_dps = self.get_parameter('stationary_gyro_dps').value
        self._bias = None                # 收敛后的零偏（°/s）；None = 还没收敛
        self._bias_samples = []
        self._assembler = None
        self._reader = None
        self._reader_stop = threading.Event()

        self.device = None
        self.last_command = None
        self.command = (0.0, 0.0)
        self.remote_command = (0.0, 0.0)
        self.last_remote = None
        self.mode = 3
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
        self.mode_pub = self.create_publisher(Int8, '/robot_mode', 10)
        self.mode_pub.publish(Int8(data=self.mode))
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.wheel_pub = self.create_publisher(Odometry, self.wheel_odom_topic, 10)
        self.bias_pub = self.create_publisher(Float32, '/imu/gyro_bias', 10)
        self.create_timer(0.05, self.tick)
        if not self.enabled:
            self.get_logger().warn('Motor output disabled; set motor_enable:=true explicitly')

    # ===================== 上行遥测：轮速 + IMU =====================

    def _is_stationary(self, frame):
        """静止判据：本模式下【指令为 0】且四轮计数都在静息阈值内。

        用指令参与判断是因为它最可靠（没有指令就是不该动），四轮计数是佐证。
        """
        linear, angular = self.remote_command if self.mode == 2 else self.command
        commanded = abs(linear) > 1e-6 or abs(angular) > 1e-6
        counts = frame['motor_speed']
        still = all(abs(c) <= self.wheel_rest_counts for c in counts)   # <= ：±1 抖动算静止
        return (not commanded) and still

    def _gyro_bias_dps(self, frame):
        """静基座陀螺零偏（°/s）的估计与返回。

        ⚠️ robot_localization 的 EKF 【没有零偏状态】，不在这里扣掉的话，陀螺
        带来的 yaw 改善会被零偏吃掉。真机实测零偏中位数约 ±0.07 °/s（两次运行
        分别 +0.061 / -0.067，50 个静止样本的中位数；各次运行不同），虽然不大，
        但目标是把静止漂压到几个 0.01 °/s，加上噪声后不扣仍会明显漂。
        策略：静止时先攒 warmup 个样本取中位数，之后极慢泄漏自适应。
        """
        if not self.subtract_gyro_bias:
            return 0.0
        if math.isfinite(self.gyro_bias_param):
            return self.gyro_bias_param          # 用户硬写了值
        if not self._is_stationary(frame):
            return self._bias if self._bias is not None else 0.0
        gz = frame['gyro_z']
        if self._bias is None:
            # 未收敛：只在明显静止（|gz| 小）时攒样本，避免把"其实在转"采进去
            if abs(gz) < self.stationary_gyro_dps:
                self._bias_samples.append(gz)
                if len(self._bias_samples) >= self.bias_warmup:
                    self._bias = statistics.median(self._bias_samples)
                    self.get_logger().info(
                        f'陀螺零偏收敛: {self._bias:+.3f} °/s'
                        f'（{self.bias_warmup} 个静止样本的中位数）')
            return 0.0                            # 收敛前不扣，先用原始值
        # 已收敛：极慢自适应，并用量级守卫排除"其实在转"的样本
        if abs(gz - self._bias) < self.stationary_gyro_dps:
            self._bias += self.gyro_bias_alpha * (gz - self._bias)
            lim = self.gyro_bias_max_dps
            self._bias = max(-lim, min(lim, self._bias))
        return self._bias

    def _on_frame(self, frame, stamp):
        """把一帧遥测转成 ROS 消息。异常由调用方兜住。"""
        if not self.publish_sensors:
            return
        bias = self._gyro_bias_dps(frame)

        if frame['flags'] & FLAG_IMU_VALID:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self.base_frame
            # ⚠️ orientation_covariance[0] = -1 是 sensor_msgs/Imu 的约定：
            #    "姿态未知"。必须这么写 —— 磁力计是坏的（mag_* 恒为 0 而
            #    MAG_VALID 位照置），而 JY901S 自己的 imu_yaw 实测还在漂
            #    0.2 °/s（磁力计失效后退化成陀螺积分）。所以绝不能把 yaw
            #    当作姿态发给下游。
            imu.orientation_covariance[0] = -1.0
            imu.angular_velocity.x = frame['gyro_x'] * DEG2RAD
            imu.angular_velocity.y = frame['gyro_y'] * DEG2RAD
            imu.angular_velocity.z = (frame['gyro_z'] - bias) * DEG2RAD
            # 陀螺噪声由实测 ±1.5 °/s 抖动推得 ~0.005 rad/s（方差 2.5e-5）
            imu.angular_velocity_covariance[0] = 1.0
            imu.angular_velocity_covariance[4] = 1.0
            imu.angular_velocity_covariance[8] = 2.5e-5
            # 加计只发不复用：Z 轴安装方向与重力处理都没验证过，协方差给大
            imu.linear_acceleration.x = frame['acc_x'] * G_TO_MS2
            imu.linear_acceleration.y = frame['acc_y'] * G_TO_MS2
            imu.linear_acceleration.z = frame['acc_z'] * G_TO_MS2
            imu.linear_acceleration_covariance[0] = 1.0
            imu.linear_acceleration_covariance[4] = 1.0
            imu.linear_acceleration_covariance[8] = 1.0
            self.imu_pub.publish(imu)

        self._publish_wheel_odom(frame, stamp)
        self.bias_pub.publish(Float32(data=float(bias)))

    def _publish_wheel_odom(self, frame, stamp):
        """四轮计数 -> nav_msgs/Odometry。

        差速近似（左两轮 / 右两轮分别取平均）：纯前进时四轮同向、纯自转时
        左右两组反号，所以这样能把 vx 与 wz 分离出来，只丢掉横移自由度 ——
        而本车在 ROS 室内模式下从不横移（固件 Vy_set 恒 0、nav2 的
        max_vel_y=0），所以这个近似对本车是充分的。
        实测原地左转时四轮为 [-26,+26,-26,+26]（FL,FR,RL,RR），与 REP-103
        的"左转为正"一致。
        """
        counts = list(frame['motor_speed'])
        p_ms = frame.get('loop_period_ms') or 0
        dt = (p_ms / 1000.0) if p_ms > 0 else 0.035      # 兜底 35 ms
        vx, vy, wz, still = wheel_twist_from_counts(
            counts, dt, self.wheel_counts_per_mps, self.wheel_track,
            self.wheel_rest_counts, self.wheel_invert)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.twist.twist.linear.x = vx
        od.twist.twist.linear.y = vy
        od.twist.twist.angular.z = wz
        # ⭐ ZUPT：四轮为 0 时用【极小】协方差发零速，把 EKF 速度强拉向 0
        #    -> 静止时位姿被钉住，这就是"静止不再漂"的承载机制。
        #    运动时用【大】协方差，让 rf2o 主导平移、未标定的轮速无法污染它。
        cov_vx = self.cov_rest_vx if still else self.cov_move_vx
        tw = [0.0] * 36
        tw[0] = cov_vx
        tw[7] = cov_vx
        tw[14] = 1e-4
        tw[21] = 1.0
        tw[28] = 1.0
        tw[35] = self.cov_vyaw
        od.twist.covariance = tw
        # 位姿一律不积分（本节点不做位姿积分），协方差放大 = 明确告诉 EKF 别融位姿。
        # 这也是本路径只能融"速率"的原因：帧内没有 MCU 时间戳，只能用主机到达
        # 时刻打戳，存在几十 ms 的相位滞后 —— 速率能容忍，位姿不能。
        od.pose.covariance = [1e3 if i in (0, 7, 14, 21, 28, 35) else 0.0
                              for i in range(36)]
        self.wheel_pub.publish(od)

    def _read_loop(self):
        """收帧线程。⚠️ 只读：绝不 open/close/替换 self.device，绝不碰命令状态，
        所有异常就地吞掉 —— 唯一目的是不可能影响 tick() 的 fail-closed 发帧。
        """
        while not self._reader_stop.is_set():
            dev = self.device                     # 快照；tick() 可能在下一行置 None
            if dev is None:
                time.sleep(0.05)
                continue
            try:
                waiting = dev.in_waiting
                if not waiting:
                    # ⚠️ 不能紧循环空转：会抢 GIL 拖慢 50 ms 的发帧节奏，
                    #    固件 500 ms 看门狗一超时就把车停掉。
                    time.sleep(0.002)
                    continue
                chunk = dev.read(waiting)
            except (serial.SerialException, OSError):
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            try:
                frames = self._assembler.feed(chunk)
            except Exception:                     # 解析器自身的 bug 也不能杀掉线程
                continue
            for frame in frames:
                try:
                    self._on_frame(frame, self.get_clock().now().to_msg())
                except Exception as exc:          # noqa: BLE001
                    self.get_logger().warn(
                        f'telemetry frame handling failed: {exc}',
                        throttle_duration_sec=5.0)

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
        self.remote_command = (msg.linear.x, msg.angular.z)
        self.last_remote = time.monotonic()

    def tick(self):
        if not self.enabled:
            return
        if self.device is None:
            try:
                # The MCU enumerates as USB CDC (ttyACM), not the lidar's CP2102 ttyUSB.
                self.device = serial.Serial(self.port, baudrate=115200, timeout=0,
                                            write_timeout=0.1, exclusive=True)
                self.get_logger().info(f'STM32 connected on {self.port} ({self.device.name}); sending zero-safe frames')
                # 端口开成功后启动收帧线程（只启动一次；设备若被关闭重开，
                # 该线程每轮会重新快照 self.device，无需重启）。
                if self.publish_sensors and self._reader is None:
                    self._assembler = FrameAssembler()
                    self._reader = threading.Thread(
                        target=self._read_loop, daemon=True, name='telemetry_reader')
                    self._reader.start()
                    self.get_logger().info(
                        f'遥测收帧线程已启动 -> {self.imu_topic} / {self.wheel_odom_topic}'
                        f'（wheel_counts_per_mps={self.wheel_counts_per_mps:g}'
                        f' ⚠️ 未标定，见 README）')
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'Cannot open MCU {self.port}: {exc}', throttle_duration_sec=5.0)
                return
        if self.mode == 2:
            fresh = self.last_remote is not None and time.monotonic() - self.last_remote < self.timeout
            linear, angular = self.remote_command if fresh else (0.0, 0.0)
        else:
            fresh = self.last_command is not None and time.monotonic() - self.last_command < self.timeout
            linear, angular = self.command if fresh else (0.0, 0.0)

        # ⚠️ 角速度必须取反，两种模式都要。
        #
        # 固件的 Remote_WeChat_Update(mode 2) 与 Remote_ROS_Update(mode 3)
        # 对 wz 的处理【完全相同】，都是 out_wz = cmd_vel.vz * vz_scale，
        # 最后都写进同一个 chassis->Wz_set。也就是说固件这一侧的转向符号
        # 只有一种约定：正的 wz 会让车【右转】，与 ROS REP-103
        # （逆时针为正 / 左转为正）相反。
        #
        # 原来这行取反只写在 mode 2 分支里（注释 "the verified WeChat turn
        # sign" 就是当初调微信遥控时验证出来的），mode 3 漏了。后果很严重：
        #   DWB 要求左转 -> 车实际右转 -> 误差变大 -> DWB 加大左转指令
        #   -> 车转得更右 …… 正反馈一路加到满舵，车原地疯狂自旋、
        #   线速度恒为 0，进度检查必然失败，于是"中止->重规划->再自旋"死循环。
        #   表现为：到达目标点附近自旋、二次设置目标点无响应。
        #
        # 实测证据：指令 wz=+0.185 rad/s（左转）时，map 坐标系下 yaw 实际
        # 变化 -18.6°（右转），符号相反。
        angular = -angular
        try:
            self.device.write(speed_frame(linear, angular, self.max_linear, self.max_angular,
                                          mode=self.mode))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'MCU write failed: {exc}')
            self.device.close()
            self.device = None
            self.last_command = None

    def stop(self):
        # 先让收帧线程停下并等它退出，再关端口 —— 否则线程可能在 close 之后
        # 还去 read 同一个对象（虽然异常会被吞掉，但干净的顺序更可靠）。
        self._reader_stop.set()
        if self._reader is not None:
            self._reader.join(timeout=1.0)
            self._reader = None
        if self.device is not None:
            try:
                self.device.write(speed_frame(0.0, 0.0, mode=self.mode))
                self.device.flush()
            except (serial.SerialException, OSError):
                pass
            self.device.close()


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
