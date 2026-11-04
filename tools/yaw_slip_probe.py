#!/usr/bin/env python3
"""原地纯旋转打滑轮询：比较指令角速度、轮速反推角速度、IMU 陀螺角速度。

用途：大角度回头时机器人只转了指令的 1/3（陀螺+激光一致），
      用本工具判断不同角速度下的实际打滑比例和轮电流。

⚠️ 会让机器人原地旋转！运行前确认：
  - 机器人放在地面、周围至少 1.5 m 净空；
  - 手放在电源/急停上；
  - 固件已烧录且 /dev/stm32 存在。

用法：
    python3 tools/yaw_slip_probe.py --yes
    python3 tools/yaw_slip_probe.py --yes --wz 0.15 0.25 0.35 0.55 --hold 2
"""

import argparse
import os
import sys
import time

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'robot_ros2_ws', 'src',
                                'robot_base_bringup', 'scripts'))

import math  # noqa: E402

from chassis_protocol import (  # noqa: E402
    FLAG_ARMED,
    FLAG_MOTORS_READY,
    SERVICE_ARM,
    SERVICE_DISARM,
    TelemetryStream,
    result_name,
    service_frame,
    speed_frame,
)

WHEEL_RADIUS_M = 0.041713
TRACK_WIDTH_M = 0.148


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/stm32')
    ap.add_argument('--wz', type=float, nargs='+',
                    default=[0.15, 0.25, 0.35, 0.55])
    ap.add_argument('--hold', type=float, default=2.0)
    ap.add_argument('--yes', action='store_true',
                    help='确认已清空旋转空间并准备好急停')
    args = ap.parse_args()

    if not args.yes:
        print('拒绝执行：会原地旋转。确认净空/急停后加 --yes。')
        return 2

    ser = serial.Serial(args.port, 115200, timeout=0,
                        write_timeout=0.2, exclusive=True)
    stream = TelemetryStream()
    last = {'msg': None}

    def pump(seconds, wz=None):
        end = time.monotonic() + seconds
        samples = []
        while time.monotonic() < end:
            if wz is not None:
                # ⚠️ speed_frame 默认 max_angular=0.45，会把 0.55 静默钳掉；
                # 这里显式放开，保证测的是真实指令。
                ser.write(speed_frame(0.0, wz, max_linear=0.24,
                                      max_angular=max(0.55, abs(wz) + 0.01),
                                      mode=3))
            time.sleep(0.02)
            for msg in stream.feed(ser.read(512)):
                last['msg'] = msg
                samples.append(msg)
        return samples

    try:
        ser.write(service_frame(SERVICE_ARM))
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            pump(0.1)
            m = last['msg']
            if m and (m['flags'] & FLAG_ARMED):
                break
        m = last['msg']
        if not (m and (m['flags'] & FLAG_ARMED)):
            print('ARM 失败：'
                  f"{result_name(m['last_result']) if m else 'no telemetry'}")
            print('motors_ready='
                  f"{bool(m['flags'] & FLAG_MOTORS_READY) if m else False}")
            return 1

        print(f"{'wz_cmd':>7} {'wheel_wz':>9} {'gyro_wz':>9} "
              f"{'ratio':>6} {'I_L':>6} {'I_R':>6} "
              f"{'tgt_L':>7} {'tgt_R':>7} {'speed_L':>8} {'speed_R':>8}")
        for wz in args.wz:
            pump(0.4, wz)            # 起转过渡
            samples = pump(args.hold, wz)
            if not samples:
                print(f'{wz:7.2f}  no telemetry')
                continue
            ws = [((s['right_speed'] - s['left_speed']) * WHEEL_RADIUS_M
                   / TRACK_WIDTH_M) for s in samples]
            gy = [math.radians(s['gyro_deg_s'][2]) for s in samples]
            il = [s['left_current'] for s in samples]
            ir = [s['right_current'] for s in samples]
            sl = [s['left_speed'] for s in samples]
            sr = [s['right_speed'] for s in samples]
            tl = [s['left_target'] for s in samples]
            tr = [s['right_target'] for s in samples]
            wheel_wz = sum(ws) / len(ws)
            gyro_wz = sum(gy) / len(gy)
            ratio = gyro_wz / wheel_wz if abs(wheel_wz) > 1e-6 else 0.0
            print(f'{wz:7.2f} {wheel_wz:9.3f} {gyro_wz:9.3f} {ratio:6.2f} '
                  f'{sum(il)/len(il):6.2f} {sum(ir)/len(ir):6.2f} '
                  f'{sum(tl)/len(tl):7.2f} {sum(tr)/len(tr):7.2f} '
                  f'{sum(sl)/len(sl):8.2f} {sum(sr)/len(sr):8.2f}')

        pump(0.3, 0.0)
        ser.write(service_frame(SERVICE_DISARM))
        pump(0.5, 0.0)
        print('DISARM 完成')
        return 0
    finally:
        try:
            ser.write(service_frame(SERVICE_DISARM))
            ser.write(speed_frame(0.0, 0.0, mode=3))
        except (serial.SerialException, OSError):
            pass
        ser.close()


if __name__ == '__main__':
    sys.exit(main())
