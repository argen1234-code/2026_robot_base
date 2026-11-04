#!/usr/bin/env python3
"""直连 STM32 的固定圈数/固定时长速度标定，并同时记录雷达前向距离。

流程：
  1. DISARM + 静止基线；
  2. ARM；
  3. 以恒定 body 线速度前进（或后退）固定时长；
  4. 同时记录 50 Hz 轮速/IMU 和 /scan 前向扇区距离；
  5. 停止、零速、DISARM，积分编码器轮速得到实际圈数和距离；
  6. 输出 YAML/CSV，供与卷尺距离、雷达距离变化交叉验证。

默认：5 圈/轮、0.10 m/s、按 5×2π×0.041/0.10≈12.9 s 运行。
需要先启动雷达驱动（另一个终端）：
  ros2 launch ydlidar_ros2_driver ydlidar_launch.py publish_static_tf:=false

用法：
  python3 tools/speed_calibrate.py --revs 5 --speed 0.10
跑完后测量从起点标记到终点同一参考点的地面距离，填入报告或告诉我。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import struct
import sys
import time
from datetime import datetime

import serial
import yaml

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
except Exception:  # pragma: no cover
    rclpy = None
    Node = object
    qos_profile_sensor_data = None
    LaserScan = None

SERVICE_ARM = 1
SERVICE_DISARM = 2
TELEMETRY_HEADER = b'\xcc\x55'
TELEMETRY_SIZE = 104
FLAG_MOTORS_READY = 1 << 2
FLAG_ARMED = 1 << 3

_RESULT_NAMES = {
    0: 'OK', 1: 'BAD_COMMAND', 2: 'RANGE', 3: 'STATE', 4: 'CONFIRM',
    5: 'OFFLINE', 6: 'TX', 7: 'READBACK', 8: 'MOTOR_FAULT', 9: 'TIMEOUT',
    10: 'BUSOFF', 11: 'PROTECTION', 13: 'HEALTH', 14: 'OVERSPEED',
    15: 'SERVICE_GAP', 16: 'PENDING', 17: 'BUSY',
}


def result_name(value: int) -> str:
    return _RESULT_NAMES.get(value, str(value))


def wrap180(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def mean(values):
    return sum(values) / len(values) if values else float('nan')


def service_frame(command: int) -> bytes:
    if command not in (SERVICE_ARM, SERVICE_DISARM):
        raise ValueError('bad service command')
    return b'\xbb\x55' + bytes([command, command])


def speed_frame_wire(vx: float, wz: float, mode: int = 3) -> bytes:
    payload = struct.pack('<Bff', mode, float(vx), float(wz))
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return b'\xaa\x55' + payload + bytes([checksum])


def parse_telemetry(frame: bytes) -> dict:
    if len(frame) != TELEMETRY_SIZE or frame[0:2] != TELEMETRY_HEADER:
        raise ValueError('bad frame')
    checksum = 0
    for byte in frame[2:-1]:
        checksum ^= byte
    if checksum != frame[-1]:
        raise ValueError('bad checksum')
    values = struct.unpack('<BBBBBHI19fIIIIB', frame)
    return {
        'version': values[2],
        'flags': values[3],
        'mode': values[4],
        'seq': values[5],
        'tick_ms': values[6],
        'cmd_vx': values[7],
        'cmd_vz': values[8],
        'left_speed': values[9],
        'right_speed': values[10],
        'left_target': values[11],
        'right_target': values[12],
        'left_current': values[13],
        'right_current': values[14],
        'roll_deg': values[15],
        'pitch_deg': values[16],
        'yaw_deg': values[17],
        'gyro_deg_s': values[18:21],
        'accel_g': values[21:24],
        'req_left': values[24],
        'req_right': values[25],
        'last_result': values[26],
        'rx_frames': values[27],
        'rx_bad': values[28],
        'tx_busy': values[29],
    }


class TelemetryStream:
    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes):
        self._buffer.extend(data)
        frames = []
        while True:
            start = self._buffer.find(TELEMETRY_HEADER)
            if start < 0:
                if len(self._buffer) > 1:
                    del self._buffer[:-1]
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < TELEMETRY_SIZE:
                break
            frame = bytes(self._buffer[:TELEMETRY_SIZE])
            try:
                frames.append(parse_telemetry(frame))
            except ValueError:
                del self._buffer[:2]
                continue
            del self._buffer[:TELEMETRY_SIZE]
        return frames


class ScanListener(Node if rclpy else object):
    def __init__(self, topic: str, forward_deg: float, sector_deg: float,
                 min_range: float, max_range: float, scan_writer):
        super().__init__('speed_calibration_scan')
        self.forward_rad = math.radians(forward_deg)
        self.sector_rad = math.radians(sector_deg)
        self.min_range = min_range
        self.max_range = max_range
        self.scan_writer = scan_writer
        self.last_range = None
        self.last_min = None
        self.last_stamp = None
        self.count = 0
        self.create_subscription(LaserScan, topic, self.cb, qos_profile_sensor_data)

    def cb(self, msg: LaserScan):
        vals = []
        min_val = None
        stamp = time.time()
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < self.min_range or r > self.max_range:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            # 归一到 [-pi, pi]
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if self.scan_writer is not None:
                self.scan_writer.writerow(
                    [f'{stamp:.6f}', f'{math.degrees(angle):.2f}', f'{r:.4f}'])
            diff = (angle - self.forward_rad + math.pi) % (2.0 * math.pi) - math.pi
            if abs(diff) <= self.sector_rad:
                vals.append(r)
                if min_val is None or r < min_val:
                    min_val = r
        if vals:
            self.last_range = statistics.median(vals)
        if min_val is not None:
            self.last_min = min_val
        self.last_stamp = stamp
        self.count += 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/stm32')
    parser.add_argument('--revs', type=float, default=5.0,
                        help='目标车轮圈数（默认 5）')
    parser.add_argument('--speed', type=float, default=0.10,
                        help='body 线速度 m/s（默认 0.10）')
    parser.add_argument('--wheel-radius', type=float, default=0.041)
    parser.add_argument('--direction', choices=('forward', 'backward'),
                        default='forward')
    parser.add_argument('--time', type=float, default=None,
                        help='覆盖运行秒数（默认按圈数/速度计算）')
    parser.add_argument('--settle', type=float, default=0.5,
                        help='ARM 后零速等待秒数')
    parser.add_argument('--stop-settle', type=float, default=0.6,
                        help='停止后继续记录秒数')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--forward-angle-deg', type=float, default=90.0,
                        help='laser_frame 中车头对应的扫描角（默认 +90°）')
    parser.add_argument('--sector-deg', type=float, default=10.0)
    parser.add_argument('--min-range', type=float, default=0.20,
                        help='忽略小于该距离的雷达点（滤掉自回波，默认 0.20 m）')
    parser.add_argument('--max-range', type=float, default=10.0)
    parser.add_argument('--safety-range', type=float, default=0.30,
                        help='前向中值小于该距离时提前停车（默认 0.30 m）')
    parser.add_argument('--no-lidar', action='store_true')
    parser.add_argument('--mode', type=int, default=3, choices=(2, 3))
    parser.add_argument('--csv', default='/tmp/speed_calibration.csv')
    parser.add_argument('--scan-csv', default='/tmp/speed_scan.csv',
                        help='整圈扫描点 (time,angle_deg,range) 输出')
    parser.add_argument('--report', default='/tmp/speed_calibration.yaml')
    args = parser.parse_args()

    sign = 1.0 if args.direction == 'forward' else -1.0
    run_time = args.time
    if run_time is None:
        run_time = abs(args.revs) * 2.0 * math.pi * args.wheel_radius / args.speed
    wire_vx = -sign * args.speed  # vx<0 前进

    scan_node = None
    scan_file = None
    scan_writer = None
    if not args.no_lidar and rclpy is not None:
        scan_file = open(args.scan_csv, 'w', newline='', encoding='utf-8')
        scan_writer = csv.writer(scan_file)
        scan_writer.writerow(['time', 'angle_deg', 'range_m'])
        rclpy.init(args=None)
        scan_node = ScanListener(args.scan_topic,
                                 args.forward_angle_deg, args.sector_deg,
                                 args.min_range, args.max_range, scan_writer)

    ser = serial.Serial(args.port, 115200, timeout=0, write_timeout=0.2)
    stream = TelemetryStream()
    telemetry = None
    samples = []
    csv_file = open(args.csv, 'w', newline='', encoding='utf-8')
    writer = csv.writer(csv_file)
    writer.writerow([
        'time', 'run_phase', 'wire_vx', 'wire_wz',
        'left_target', 'right_target', 'left_speed', 'right_speed',
        'roll', 'pitch', 'yaw', 'gyro_z',
        'accel_x', 'accel_y', 'accel_z',
        'lidar_median', 'lidar_min', 'scan_count'])

    def spin():
        if scan_node is not None:
            rclpy.spin_once(scan_node, timeout_sec=0.0)

    def drain(phase: str):
        nonlocal telemetry
        data = ser.read(ser.in_waiting or 1)
        if data:
            for frame in stream.feed(data):
                telemetry = frame
                samples.append((time.time(), phase, frame,
                                scan_node.last_range if scan_node else None,
                                scan_node.last_min if scan_node else None))
                writer.writerow([
                    f'{samples[-1][0]:.6f}', phase,
                    f'{wire_vx:.4f}', '0.0000',
                    f'{frame["left_target"]:.4f}',
                    f'{frame["right_target"]:.4f}',
                    f'{frame["left_speed"]:.4f}',
                    f'{frame["right_speed"]:.4f}',
                    f'{frame["roll_deg"]:.3f}',
                    f'{frame["pitch_deg"]:.3f}',
                    f'{frame["yaw_deg"]:.3f}',
                    f'{frame["gyro_deg_s"][2]:.4f}',
                    f'{frame["accel_g"][0]:.4f}',
                    f'{frame["accel_g"][1]:.4f}',
                    f'{frame["accel_g"][2]:.4f}',
                    '' if scan_node is None or scan_node.last_range is None
                    else f'{scan_node.last_range:.4f}',
                    '' if scan_node is None or scan_node.last_min is None
                    else f'{scan_node.last_min:.4f}',
                    scan_node.count if scan_node else 0])

    def send(vx: float, wz: float = 0.0):
        ser.write(speed_frame_wire(vx, wz, args.mode))

    try:
        # 先 DISARM，采一小段静止基线。
        ser.write(service_frame(SERVICE_DISARM))
        time.sleep(0.2)
        base_start = time.time()
        while time.time() - base_start < 0.8:
            drain('baseline')
            spin()
            time.sleep(0.005)

        arm_start = time.time()
        while time.time() - arm_start < 8.0:
            ser.write(service_frame(SERVICE_ARM))
            drain('arm')
            spin()
            if telemetry and (telemetry['flags'] & FLAG_ARMED):
                break
            time.sleep(0.1)
        else:
            print('ARM 超时')
            return 1
        print(f"ARM OK result={result_name(telemetry['last_result'])}")

        # 零速稳定
        settle_end = time.time() + args.settle
        while time.time() < settle_end:
            send(0.0)
            drain('settle')
            spin()
            time.sleep(0.02)

        print(f'开始移动：{args.direction} {args.speed:.3f} m/s，'
              f'目标 {args.revs:.2f} 圈，运行 {run_time:.2f} s')
        t_move = time.time()
        move_end = t_move + run_time
        while time.time() < move_end:
            send(wire_vx, 0.0)
            drain('move')
            spin()
            if (scan_node is not None and scan_node.last_range is not None
                    and scan_node.last_range < args.safety_range):
                print(f'雷达前向 {scan_node.last_range:.2f} m < '
                      f'{args.safety_range:.2f} m，提前停车')
                break
            time.sleep(0.02)
        t_stop = time.time()
        print(f'停止移动：t={t_stop - t_move:.2f}s，零速 {args.stop_settle:.1f}s')

        stop_end = time.time() + args.stop_settle
        while time.time() < stop_end:
            send(0.0)
            drain('stop')
            spin()
            time.sleep(0.02)

        # 再采一小段静止，便于 IMU 结束值。
        final_end = time.time() + 0.5
        while time.time() < final_end:
            send(0.0)
            drain('final')
            spin()
            time.sleep(0.02)

    finally:
        try:
            for _ in range(10):
                send(0.0)
                time.sleep(0.02)
            ser.write(service_frame(SERVICE_DISARM))
            ser.flush()
            time.sleep(0.1)
            drain('final')
        finally:
            ser.close()
            csv_file.close()
            if scan_file is not None:
                scan_file.close()
            if scan_node is not None:
                scan_node.destroy_node()
                rclpy.shutdown()

    # 只积分 move + stop 段；用左右轮速梯形积分。
    move_samples = [s for s in samples if s[1] in ('move', 'stop')]
    rev_l = 0.0
    rev_r = 0.0
    dist_l = 0.0
    dist_r = 0.0
    prev = None
    for ts, phase, f, _, _ in move_samples:
        if prev is not None:
            dt = ts - prev[0]
            if 0.0 < dt < 0.5:
                rev_l += 0.5 * (prev[1]['left_speed'] + f['left_speed']) * dt / (2 * math.pi)
                rev_r += 0.5 * (prev[1]['right_speed'] + f['right_speed']) * dt / (2 * math.pi)
                dist_l += 0.5 * (prev[1]['left_speed'] + f['left_speed']) * dt * args.wheel_radius
                dist_r += 0.5 * (prev[1]['right_speed'] + f['right_speed']) * dt * args.wheel_radius
        prev = (ts, f)

    rev_mean = 0.5 * (rev_l + rev_r)
    encoder_distance = 0.5 * (dist_l + dist_r)

    lidar_vals = [(ts, med, mn) for ts, phase, _, med, mn in samples
                  if phase in ('move', 'stop') and med is not None]
    lidar_start = lidar_vals[0][1] if lidar_vals else None
    lidar_end = lidar_vals[-1][1] if lidar_vals else None
    lidar_delta = (lidar_start - lidar_end
                   if lidar_start is not None and lidar_end is not None else None)

    first = next((s[2] for s in samples if s[1] == 'move'), None)
    last = next((s[2] for s in reversed(samples) if s[1] in ('stop', 'final')), None)
    imu_delta = None
    if first is not None and last is not None:
        imu_delta = {
            'roll_deg': wrap180(last['roll_deg'] - first['roll_deg']),
            'pitch_deg': wrap180(last['pitch_deg'] - first['pitch_deg']),
            'yaw_deg': wrap180(last['yaw_deg'] - first['yaw_deg']),
        }

    result = {
        'measured_at': datetime.now().isoformat(timespec='seconds'),
        'command': {
            'direction': args.direction,
            'body_speed_m_s': args.speed,
            'target_revs_per_wheel': args.revs,
            'run_time_s': run_time,
            'wire_vx': wire_vx,
            'wheel_radius_m': args.wheel_radius,
            'expected_distance_m': args.revs * 2 * math.pi * args.wheel_radius,
        },
        'encoder': {
            'left_revs': rev_l,
            'right_revs': rev_r,
            'mean_revs': rev_mean,
            'left_distance_m': dist_l,
            'right_distance_m': dist_r,
            'mean_distance_m': encoder_distance,
            'effective_circumference_m': (2 * math.pi * args.wheel_radius),
        },
        'lidar': {
            'forward_angle_deg': args.forward_angle_deg,
            'sector_deg': args.sector_deg,
            'start_median_m': lidar_start,
            'end_median_m': lidar_end,
            'delta_m': lidar_delta,
            'samples': len(lidar_vals),
        },
        'imu_delta_deg': imu_delta,
        'tape_distance_m': None,
        'note': '请把卷尺实测距离填入 tape_distance_m，再计算有效轮径/里程比例。',
    }
    with open(args.report, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(result, fh, sort_keys=False, allow_unicode=True)

    print('\n=== 结果 ===')
    print(f"命令: {args.direction} {args.speed:.3f} m/s × {run_time:.2f}s，"
          f"目标 {args.revs:.2f} 圈，理论 {args.revs * 2 * math.pi * args.wheel_radius:.3f} m")
    print(f'编码器: L={rev_l:.3f} 圈 R={rev_r:.3f} 圈 均值={rev_mean:.3f} 圈')
    print(f'编码器距离: L={dist_l:.3f} m R={dist_r:.3f} m 均值={encoder_distance:.3f} m')
    if lidar_start is not None:
        print(f'雷达前向: 开始={lidar_start:.3f} m 结束={lidar_end:.3f} m '
              f'变化={lidar_delta:+.3f} m（样本 {len(lidar_vals)}）')
    else:
        print('雷达前向: 没有数据（检查驱动/目标是否在 +90° 扇区）')
    if imu_delta:
        print(f"IMU 变化: roll={imu_delta['roll_deg']:+.2f}° "
              f"pitch={imu_delta['pitch_deg']:+.2f}° "
              f"yaw={imu_delta['yaw_deg']:+.2f}°")
    print(f'\n报告: {args.report}\n逐帧: {args.csv}')
    if scan_file is not None:
        print(f'整圈扫描: {args.scan_csv}')
    print('请用卷尺测量起点标记到终点同一参考点的距离，告诉我。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
