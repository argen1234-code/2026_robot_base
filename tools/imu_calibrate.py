#!/usr/bin/env python3
"""JY901S IMU 软件标定工具（直接读 STM32 USB CDC 遥测，不经过 ROS）。

两个阶段：

1. 静止标定（默认）
   小车放水平、完全静止，采集 --seconds 秒：
   - roll/pitch/yaw 的均值与噪声
   - 陀螺零偏（deg/s）
   - 加速度均值、模长、由重力反算的 roll/pitch
   - 建议的 roll/pitch/yaw 零偏和陀螺零偏

2. 轴向符号测试 --guided
   在静止结果之后，提示你做三个动作：
   - 车头抬起（nose up）
   - 车身向左倾斜（左侧压低）
   - 绕 Z 轴左转（从上往下看逆时针）
   每次动作采集几秒，打印 roll/pitch/yaw/accel 的变化量，
   用来判断 IMU 轴是否与 ROS REP-103（x 前、y 左、z 上）一致。

结果写入 --out（默认 robot_base_bringup/config/imu_calibration.yaml）。
注意：本工具只读遥测，不会 ARM 电机；请先停掉 ROS 导航栈，
确保没有 chassis_bridge 占用 /dev/stm32。
"""

from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import time
from datetime import datetime

import serial
import yaml

BRINGUP_SCRIPTS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'robot_ros2_ws', 'src', 'robot_base_bringup', 'scripts')
sys.path.insert(0, BRINGUP_SCRIPTS)

from chassis_protocol import TelemetryStream  # noqa: E402


def wrap180(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def read_frames(ser, stream, seconds: float, label: str):
    frames = []
    start = time.time()
    last_print = start
    while time.time() - start < seconds:
        data = ser.read(ser.in_waiting or 1)
        if data:
            frames.extend(stream.feed(data))
        now = time.time()
        if now - last_print >= 1.0:
            print(f'  {label}: {len(frames)} 帧 / {now - start:.1f}s')
            last_print = now
        time.sleep(0.005)
    return frames


def mean_std(values):
    if not values:
        return float('nan'), float('nan')
    return statistics.fmean(values), (statistics.pstdev(values) if len(values) > 1 else 0.0)


def summarize(frames):
    if not frames:
        raise RuntimeError('没有收到遥测帧：检查 /dev/stm32、电源和 IMU 在线状态')
    out = {}
    for key in ('roll_deg', 'pitch_deg', 'yaw_deg',
                'left_speed', 'right_speed'):
        m, s = mean_std([f[key] for f in frames])
        out[key] = (m, s)
    for axis in range(3):
        m, s = mean_std([f['gyro_deg_s'][axis] for f in frames])
        out[f'gyro_{axis}'] = (m, s)
        m, s = mean_std([f['accel_g'][axis] for f in frames])
        out[f'accel_{axis}'] = (m, s)
    accel = (out['accel_0'][0], out['accel_1'][0], out['accel_2'][0])
    out['accel_norm'] = math.sqrt(sum(v * v for v in accel))
    if out['accel_norm'] > 1e-6:
        ax, ay, az = accel
        out['gravity_roll_deg'] = math.degrees(math.atan2(ay, az))
        out['gravity_pitch_deg'] = math.degrees(
            math.atan2(-ax, math.hypot(ay, az)))
    else:
        out['gravity_roll_deg'] = float('nan')
        out['gravity_pitch_deg'] = float('nan')
    return out


def print_static(result):
    print('\n=== 静止结果 ===')
    for name, key in (('roll', 'roll_deg'), ('pitch', 'pitch_deg'),
                      ('yaw', 'yaw_deg')):
        m, s = result[key]
        print(f'{name:5s}: mean={m:+8.3f}°  std={s:6.3f}°')
    print('gyro (deg/s): '
          + ', '.join(f'{axis}={result[f"gyro_{axis}"][0]:+7.3f}'
                      f'±{result[f"gyro_{axis}"][1]:.3f}'
                      for axis in range(3)))
    print('accel (g):    '
          + ', '.join(f'{axis}={result[f"accel_{axis}"][0]:+7.3f}'
                      for axis in range(3))
          + f'  |a|={result["accel_norm"]:.4f}')
    print(f'重力反算: roll={result["gravity_roll_deg"]:+.2f}°  '
          f'pitch={result["gravity_pitch_deg"]:+.2f}°')
    print('\n建议零偏（如果此时小车确实水平且初始朝向作为 0°）：')
    print(f'  roll_offset_deg:  {-result["roll_deg"][0]:+.3f}')
    print(f'  pitch_offset_deg: {-result["pitch_deg"][0]:+.3f}')
    print(f'  yaw_offset_deg:   {-result["yaw_deg"][0]:+.3f}')
    print(f'  gyro_bias_dps:    '
          f'[{result["gyro_0"][0]:+.4f}, {result["gyro_1"][0]:+.4f}, '
          f'{result["gyro_2"][0]:+.4f}]')
    if result['accel_norm'] > 1e-6:
        print(f'  accel_scale:      '
              f'{1.0 / result["accel_norm"]:.5f}  (当前 |a|={result["accel_norm"]:.4f} g)')


def guided_test(ser, stream, base):
    print('\n=== 轴向符号测试 ===')
    print('每次提示后把小车摆到指定姿态并保持不动，然后按 Enter。')
    steps = [
        ('nose_up', '把车头抬起约 20°（车尾仍着地）'),
        ('roll_left', '把车身向左倾斜约 20°（左侧压低）'),
        ('yaw_left', '绕 Z 轴向左转约 45°（从上往下看逆时针）'),
    ]
    deltas = {}
    for key, prompt in steps:
        input(f'\n准备：{prompt}，摆好后按 Enter 开始采集...')
        frames = read_frames(ser, stream, 2.5, key)
        if not frames:
            print(f'  {key}: 没有数据')
            continue
        m, _ = summarize(frames), None
        d = {
            'roll': wrap180(m['roll_deg'][0] - base['roll_deg'][0]),
            'pitch': wrap180(m['pitch_deg'][0] - base['pitch_deg'][0]),
            'yaw': wrap180(m['yaw_deg'][0] - base['yaw_deg'][0]),
            'ax': m['accel_0'][0] - base['accel_0'][0],
            'ay': m['accel_1'][0] - base['accel_1'][0],
            'az': m['accel_2'][0] - base['accel_2'][0],
        }
        deltas[key] = d
        print(f'  变化: roll={d["roll"]:+.2f}° pitch={d["pitch"]:+.2f}° '
              f'yaw={d["yaw"]:+.2f}°  '
              f'accel=({d["ax"]:+.3f},{d["ay"]:+.3f},{d["az"]:+.3f}) g')

    print('\n=== 轴向判断 ===')
    if 'nose_up' in deltas:
        d = deltas['nose_up']
        axis = 'pitch' if abs(d['pitch']) >= abs(d['roll']) else 'roll'
        print(f'车头抬起主要改变 {axis}（{d[axis]:+.2f}°）。')
        print('ROS REP-103 下，x 前、y 左、z 上；pitch 绕 +y，'
              '车头抬起对应 pitch 负方向。')
    if 'roll_left' in deltas:
        d = deltas['roll_left']
        axis = 'roll' if abs(d['roll']) >= abs(d['pitch']) else 'pitch'
        print(f'左倾主要改变 {axis}（{d[axis]:+.2f}°）。')
        print('ROS REP-103 下，左侧压低对应 roll 负方向（绕 +x）。')
    if 'yaw_left' in deltas:
        d = deltas['yaw_left']
        print(f'左转 yaw 变化 {d["yaw"]:+.2f}°；'
              f'ROS 左转（逆时针）应为 yaw 增大。')
    return deltas


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/stm32')
    parser.add_argument('--seconds', type=float, default=8.0,
                        help='静止采集秒数（默认 8）')
    parser.add_argument('--guided', action='store_true',
                        help='静止后继续做抬车头/侧倾/左转轴符号测试')
    parser.add_argument(
        '--out',
        default=os.path.join(BRINGUP_SCRIPTS, '..', 'config',
                             'imu_calibration.yaml'))
    args = parser.parse_args()

    print('请先确认：ROS 导航栈已停止，/dev/stm32 没有被 chassis_bridge 占用。')
    print('阶段 1：小车放水平、完全静止，开始采集。')
    ser = serial.Serial(args.port, 115200, timeout=0)
    stream = TelemetryStream()
    try:
        frames = read_frames(ser, stream, args.seconds, 'static')
        result = summarize(frames)
        print_static(result)

        guided = guided_test(ser, stream, result) if args.guided else None
    finally:
        ser.close()

    calibration = {
        'source': 'JY901S via STM32 USB CDC',
        'measured_at': datetime.now().isoformat(timespec='seconds'),
        'static': {
            'samples': len(frames),
            'roll_deg': result['roll_deg'][0],
            'pitch_deg': result['pitch_deg'][0],
            'yaw_deg': result['yaw_deg'][0],
            'roll_std_deg': result['roll_deg'][1],
            'pitch_std_deg': result['pitch_deg'][1],
            'yaw_std_deg': result['yaw_deg'][1],
            'gyro_bias_dps': [result[f'gyro_{i}'][0] for i in range(3)],
            'accel_g': [result[f'accel_{i}'][0] for i in range(3)],
            'accel_norm_g': result['accel_norm'],
            'gravity_roll_deg': result['gravity_roll_deg'],
            'gravity_pitch_deg': result['gravity_pitch_deg'],
        },
        'suggested_offsets': {
            'roll_offset_deg': -result['roll_deg'][0],
            'pitch_offset_deg': -result['pitch_deg'][0],
            'yaw_offset_deg': -result['yaw_deg'][0],
            'gyro_bias_dps': [result[f'gyro_{i}'][0] for i in range(3)],
            'accel_scale': (1.0 / result['accel_norm']
                            if result['accel_norm'] > 1e-6 else 1.0),
        },
    }
    if guided is not None:
        calibration['guided_axis_test'] = guided

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(calibration, fh, sort_keys=False, allow_unicode=True)
    print(f'\n标定结果已写入: {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
