#!/usr/bin/env python3
"""键盘直连 STM32 USB CDC 的标定工具（不经过 ROS）。

启动时自动做三件事：
  1. DISARM，静止采集 IMU/轮速基线（--baseline 秒）；
  2. ARM，进入键盘控制；
  3. 全程把每一帧遥测写入 CSV，并按 W/S/A/D 按键段统计
     轮速目标、实测轮速、IMU 的 roll/pitch/yaw 变化；
  4. 退出时生成 YAML 标定报告（--report），并零速 + DISARM。

按键：
  w  前进（ROS linear.x > 0）
  s  后退（ROS linear.x < 0）
  a  左转（ROS angular.z > 0，逆时针）
  d  右转（ROS angular.z < 0，顺时针）
  空格 / x  立即停止
  q / Esc   退出

wire 约定：vx<0 前进、wz<0 左转；脚本把 ROS 的 +linear/+angular
换算成 wire 的 -linear/-angular。键盘工具直接发原始帧，允许倒车。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import select
import struct
import sys
import termios
import time
import tty
from datetime import datetime

import serial
import yaml

SERVICE_ARM = 1
SERVICE_DISARM = 2
TELEMETRY_HEADER = b'\xcc\x55'
TELEMETRY_SIZE = 104
FLAG_MOTORS_READY = 1 << 2
FLAG_ARMED = 1 << 3
FLAG_FATAL = 1 << 5

_RESULT_NAMES = {
    0: 'OK', 1: 'BAD_COMMAND', 2: 'RANGE', 3: 'STATE', 4: 'CONFIRM',
    5: 'OFFLINE', 6: 'TX', 7: 'READBACK', 8: 'MOTOR_FAULT', 9: 'TIMEOUT',
    10: 'BUSOFF', 11: 'PROTECTION', 13: 'HEALTH', 14: 'OVERSPEED',
    15: 'SERVICE_GAP', 16: 'PENDING', 17: 'BUSY',
}

KEY_MAP = {
    'w': ('forward', 1.0, 0.0),
    's': ('backward', -1.0, 0.0),
    'a': ('left', 0.0, 1.0),
    'd': ('right', 0.0, -1.0),
}


def result_name(value: int) -> str:
    return _RESULT_NAMES.get(value, str(value))


def wrap180(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def mean(values):
    return sum(values) / len(values) if values else float('nan')


def minmax(values):
    return (min(values), max(values)) if values else (float('nan'), float('nan'))


def service_frame(command: int) -> bytes:
    if command not in (SERVICE_ARM, SERVICE_DISARM):
        raise ValueError('bad service command')
    return b'\xbb\x55' + bytes([command, command])


def speed_frame_wire(vx: float, wz: float, mode: int = 3) -> bytes:
    """直接按固件 wire 约定打包：vx<0 前进，wz<0 左转。"""
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


class Segment:
    def __init__(self, key, logical_v, logical_w, wire_vx, wire_wz):
        self.key = key
        self.logical_v = logical_v
        self.logical_w = logical_w
        self.wire_vx = wire_vx
        self.wire_wz = wire_wz
        self.start = time.time()
        self.samples = []

    def add(self, frame):
        self.samples.append(frame)

    def summary(self):
        if not self.samples:
            return None
        first = self.samples[0]
        last = self.samples[-1]
        left = [f['left_speed'] for f in self.samples]
        right = [f['right_speed'] for f in self.samples]
        ltgt = [f['left_target'] for f in self.samples]
        rtgt = [f['right_target'] for f in self.samples]
        yaw_delta = wrap180(last['yaw_deg'] - first['yaw_deg'])
        roll_delta = wrap180(last['roll_deg'] - first['roll_deg'])
        pitch_delta = wrap180(last['pitch_deg'] - first['pitch_deg'])
        return {
            'key': self.key,
            'logical_v': self.logical_v,
            'logical_w': self.logical_w,
            'wire_vx': self.wire_vx,
            'wire_wz': self.wire_wz,
            'duration_s': round(time.time() - self.start, 3),
            'frames': len(self.samples),
            'left_target_mean': mean(ltgt),
            'right_target_mean': mean(rtgt),
            'left_speed_mean': mean(left),
            'right_speed_mean': mean(right),
            'left_speed_range': minmax(left),
            'right_speed_range': minmax(right),
            'yaw_delta_deg': yaw_delta,
            'roll_delta_deg': roll_delta,
            'pitch_delta_deg': pitch_delta,
            'gyro_z_mean_dps': mean([f['gyro_deg_s'][2] for f in self.samples]),
            'flags_last': last['flags'],
            'result_last': result_name(last['last_result']),
        }


def make_suggestions(segments):
    out = []
    for seg in segments:
        if not seg:
            continue
        key = seg['key']
        lm = seg['left_speed_mean']
        rm = seg['right_speed_mean']
        ltm = seg['left_target_mean']
        rtm = seg['right_target_mean']
        dyaw = seg['yaw_delta_deg']
        if key in ('forward', 'backward'):
            if lm * ltm < 0 or rm * rtm < 0:
                out.append(f'{key}: 实测轮速与目标符号不一致，检查电机反馈/左右映射')
            if key == 'forward' and lm < 0 and rm < 0:
                out.append('forward: 两轮实测均为负，固件前进符号可能整体反了')
            if key == 'backward' and lm > 0 and rm > 0:
                out.append('backward: 两轮实测均为正，倒车符号可能整体反了')
        if key in ('left', 'right'):
            if key == 'left':
                if lm > rm:
                    out.append('left: 左轮实测大于右轮，左右/角速度映射可能反了')
                elif lm == rm:
                    out.append('left: 左右轮实测相同，角速度指令可能没生效')
                if dyaw < 0:
                    out.append('left: 左转指令下 yaw 减小，IMU yaw 方向与 ROS CCW 相反')
            else:
                if lm < rm:
                    out.append('right: 左轮实测小于右轮，左右/角速度映射可能反了')
                elif lm == rm:
                    out.append('right: 左右轮实测相同，角速度指令可能没生效')
                if dyaw > 0:
                    out.append('right: 右转指令下 yaw 增大，IMU yaw 方向与 ROS CW 相反')
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/stm32')
    parser.add_argument('--linear', type=float, default=0.08,
                        help='前进/后退最大线速度 m/s（默认 0.08）')
    parser.add_argument('--angular', type=float, default=0.25,
                        help='左/右转最大角速度 rad/s（默认 0.25）')
    parser.add_argument('--hold', type=float, default=0.35,
                        help='最后一次按键后保持指令的秒数（默认 0.35）')
    parser.add_argument('--rate', type=float, default=50.0,
                        help='下行帧频率 Hz（默认 50）')
    parser.add_argument('--baseline', type=float, default=2.5,
                        help='启动时静止采集基线秒数（默认 2.5）')
    parser.add_argument('--mode', type=int, default=3, choices=(2, 3))
    parser.add_argument('--arm-timeout', type=float, default=8.0)
    parser.add_argument('--log', default='/tmp/keyboard_cmd.log')
    parser.add_argument('--csv', default='/tmp/keyboard_cmd.csv')
    parser.add_argument('--report', default='/tmp/keyboard_calibration.yaml')
    args = parser.parse_args()

    if args.rate < 20:
        parser.error('--rate 至少 20 Hz，否则 MCU 500 ms 看门狗会清零')

    ser = serial.Serial(args.port, 115200, timeout=0, write_timeout=0.2)
    stream = TelemetryStream()
    telemetry = None
    log = open(args.log, 'a', buffering=1)
    csv_file = open(args.csv, 'w', newline='', encoding='utf-8')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        'time', 'key', 'logical_v', 'logical_w', 'wire_vx', 'wire_wz',
        'left_target', 'right_target', 'left_speed', 'right_speed',
        'roll', 'pitch', 'yaw', 'gyro_x', 'gyro_y', 'gyro_z',
        'accel_x', 'accel_y', 'accel_z', 'flags', 'result'])

    command = (0.0, 0.0)
    wire = (0.0, 0.0)
    last_key = 0.0
    last_key_name = '-'
    current_key = '-'
    segments = []
    segment = None
    baseline = None

    def log_line(text: str):
        stamp = time.strftime('%H:%M:%S')
        line = f'[{stamp}] {text}'
        print(line)
        log.write(line + '\n')

    def drain_serial():
        nonlocal telemetry
        data = ser.read(ser.in_waiting or 1)
        if data:
            for frame in stream.feed(data):
                telemetry = frame
                if segment is not None:
                    segment.add(frame)
                csv_writer.writerow([
                    f'{time.time():.6f}', current_key,
                    f'{command[0]:.4f}', f'{command[1]:.4f}',
                    f'{wire[0]:.4f}', f'{wire[1]:.4f}',
                    f'{frame["left_target"]:.4f}',
                    f'{frame["right_target"]:.4f}',
                    f'{frame["left_speed"]:.4f}',
                    f'{frame["right_speed"]:.4f}',
                    f'{frame["roll_deg"]:.3f}',
                    f'{frame["pitch_deg"]:.3f}',
                    f'{frame["yaw_deg"]:.3f}',
                    f'{frame["gyro_deg_s"][0]:.4f}',
                    f'{frame["gyro_deg_s"][1]:.4f}',
                    f'{frame["gyro_deg_s"][2]:.4f}',
                    f'{frame["accel_g"][0]:.4f}',
                    f'{frame["accel_g"][1]:.4f}',
                    f'{frame["accel_g"][2]:.4f}',
                    f'0x{frame["flags"]:02x}',
                    result_name(frame['last_result'])])

    def shutdown_segment():
        nonlocal segment
        if segment is not None:
            summary = segment.summary()
            if summary:
                segments.append(summary)
            segment = None

    def send(vx: float, wz: float):
        ser.write(speed_frame_wire(vx, wz, args.mode))

    old_term = termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(sys.stdin.fileno())

    try:
        # 确保上一轮遗留的 ARM 不会带目标运行。
        ser.write(service_frame(SERVICE_DISARM))
        time.sleep(0.2)

        log_line('检测已启动：先采静止基线，再 ARM 进入键盘控制')
        log_line('按键：w 前进 / s 后退 / a 左转 / d 右转 / 空格停止 / q 退出')

        # 静止基线：ARM 前采 IMU，给后续按键段做相对变化。
        base_start = time.time()
        base_frames = []
        while time.time() - base_start < args.baseline:
            data = ser.read(ser.in_waiting or 1)
            if data:
                for frame in stream.feed(data):
                    telemetry = frame
                    base_frames.append(frame)
            time.sleep(0.005)
        if base_frames:
            baseline = {
                'frames': len(base_frames),
                'roll_deg': mean([f['roll_deg'] for f in base_frames]),
                'pitch_deg': mean([f['pitch_deg'] for f in base_frames]),
                'yaw_deg': mean([f['yaw_deg'] for f in base_frames]),
                'gyro_bias_dps': [mean([f['gyro_deg_s'][i] for f in base_frames])
                                  for i in range(3)],
                'accel_g': [mean([f['accel_g'][i] for f in base_frames])
                            for i in range(3)],
            }
            log_line(f"基线: roll={baseline['roll_deg']:+.2f}° "
                     f"pitch={baseline['pitch_deg']:+.2f}° "
                     f"yaw={baseline['yaw_deg']:+.2f}° "
                     f"gyro=({baseline['gyro_bias_dps'][0]:+.2f},"
                     f"{baseline['gyro_bias_dps'][1]:+.2f},"
                     f"{baseline['gyro_bias_dps'][2]:+.2f})°/s")
        else:
            log_line('警告：基线阶段没有遥测帧')

        # ARM：每秒重发，直到遥测显示 armed。
        arm_start = time.time()
        while time.time() - arm_start < args.arm_timeout:
            ser.write(service_frame(SERVICE_ARM))
            drain_serial()
            if telemetry and (telemetry['flags'] & FLAG_ARMED):
                log_line(f"ARM 成功：result={result_name(telemetry['last_result'])} "
                         f"motors_ready={bool(telemetry['flags'] & FLAG_MOTORS_READY)}")
                break
            time.sleep(0.1)
        else:
            log_line(f'ARM 超时：result={result_name(telemetry["last_result"]) if telemetry else "无遥测"}')
            ser.write(service_frame(SERVICE_DISARM))
            return 1

        period = 1.0 / args.rate
        next_send = time.perf_counter()
        next_status = time.perf_counter() + 0.25
        running = True
        last_print_key = '-'

        while running:
            now = time.perf_counter()

            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                ch = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
                if ch in ('q', '\x1b'):
                    running = False
                    break
                if ch in (' ', 'x'):
                    shutdown_segment()
                    command = (0.0, 0.0)
                    current_key = '-'
                    last_key = 0.0
                    log_line('>>> 停止')
                elif ch.lower() in KEY_MAP:
                    name, lsign, asign = KEY_MAP[ch.lower()]
                    new_command = (lsign * args.linear, asign * args.angular)
                    if current_key != ch.lower() or command == (0.0, 0.0):
                        shutdown_segment()
                        current_key = ch.lower()
                        segment = Segment(
                            name, new_command[0], new_command[1],
                            -new_command[0], -new_command[1])
                    command = new_command
                    last_key = time.time()
                    last_key_name = ch.lower()
                    log_line(f'>>> {ch.upper()} {name}: '
                             f'linear={command[0]:+.3f} angular={command[1]:+.3f}')

            if command != (0.0, 0.0) and time.time() - last_key > args.hold:
                shutdown_segment()
                command = (0.0, 0.0)
                current_key = '-'
                log_line(f'>>> 松开 {last_key_name.upper()}，归零')

            wire = (-command[0], -command[1])

            now_perf = time.perf_counter()
            if now_perf >= next_send:
                send(wire[0], wire[1])
                next_send += period
                if next_send < now_perf:
                    next_send = now_perf + period

            drain_serial()

            if time.perf_counter() >= next_status:
                next_status += 0.5
                if telemetry:
                    flags = telemetry['flags']
                    dyaw = ''
                    if baseline is not None:
                        dyaw = f" dYaw={wrap180(telemetry['yaw_deg'] - baseline['yaw_deg']):+.2f}°"
                    log_line(
                        f"CMD v={command[0]:+.3f} w={command[1]:+.3f} | "
                        f"WIRE vx={wire[0]:+.3f} wz={wire[1]:+.3f} | "
                        f"TGT L={telemetry['left_target']:+.2f} "
                        f"R={telemetry['right_target']:+.2f} | "
                        f"MEAS L={telemetry['left_speed']:+.2f} "
                        f"R={telemetry['right_speed']:+.2f} | "
                        f"roll={telemetry['roll_deg']:+.1f} "
                        f"pitch={telemetry['pitch_deg']:+.1f} "
                        f"yaw={telemetry['yaw_deg']:+.1f}{dyaw} | "
                        f"armed={int(bool(flags & FLAG_ARMED))} "
                        f"flags=0x{flags:02x} "
                        f"result={result_name(telemetry['last_result'])}")
                else:
                    log_line('暂无遥测帧')

            time.sleep(0.002)

    finally:
        shutdown_segment()
        try:
            for _ in range(10):
                send(0.0, 0.0)
                time.sleep(0.02)
            ser.write(service_frame(SERVICE_DISARM))
            ser.flush()
            time.sleep(0.1)
            drain_serial()
            if telemetry:
                log_line(f"DISARM 已发送，最后 armed="
                         f"{int(bool(telemetry['flags'] & FLAG_ARMED))}")
        finally:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
            ser.close()

        suggestions = make_suggestions(segments)
        imu_static = None
        if baseline:
            imu_static = {
                'roll_offset_deg': -baseline['roll_deg'],
                'pitch_offset_deg': -baseline['pitch_deg'],
                'yaw_offset_deg': -baseline['yaw_deg'],
                'gyro_bias_dps': baseline['gyro_bias_dps'],
                'accel_g': baseline['accel_g'],
            }
        report = {
            'measured_at': datetime.now().isoformat(timespec='seconds'),
            'port': args.port,
            'limits': {'linear_m_s': args.linear, 'angular_rad_s': args.angular},
            'baseline': baseline,
            'imu_static': imu_static,
            'segments': segments,
            'suggestions': suggestions,
        }
        with open(args.report, 'w', encoding='utf-8') as fh:
            yaml.safe_dump(report, fh, sort_keys=False, allow_unicode=True)
        csv_file.close()
        log.close()

        print(f'\n标定报告: {args.report}')
        print(f'逐帧数据: {args.csv}')
        if segments:
            print('\n=== 按键段统计 ===')
            for seg in segments:
                print(
                    f"{seg['key']:8s} wire vx={seg['wire_vx']:+.2f} "
                    f"wz={seg['wire_wz']:+.2f} | "
                    f"TGT L={seg['left_target_mean']:+.2f} "
                    f"R={seg['right_target_mean']:+.2f} | "
                    f"MEAS L={seg['left_speed_mean']:+.2f} "
                    f"R={seg['right_speed_mean']:+.2f} | "
                    f"dYaw={seg['yaw_delta_deg']:+.2f}° "
                    f"dRoll={seg['roll_delta_deg']:+.2f}° "
                    f"dPitch={seg['pitch_delta_deg']:+.2f}°")
        else:
            print('\n没有记录到按键段（没有按 W/S/A/D）')
        if imu_static:
            print('\n=== IMU 静止零偏建议 ===')
            print(f"  roll_offset_deg:  {imu_static['roll_offset_deg']:+.3f}")
            print(f"  pitch_offset_deg: {imu_static['pitch_offset_deg']:+.3f}")
            print(f"  yaw_offset_deg:   {imu_static['yaw_offset_deg']:+.3f}")
            print(f"  gyro_bias_dps:    "
                  f"[{imu_static['gyro_bias_dps'][0]:+.4f}, "
                  f"{imu_static['gyro_bias_dps'][1]:+.4f}, "
                  f"{imu_static['gyro_bias_dps'][2]:+.4f}]")
        if suggestions:
            print('\n=== 自动建议 ===')
            for item in suggestions:
                print(' -', item)
        else:
            print('\n没有触发自动建议；把实车左/右轮实际转向一起对照。')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
