#!/usr/bin/env python3
"""树莓派 <-> STM32 USB CDC 桥接台架测试（默认不驱动电机）。

用法：
    python3 tools/usb_bridge_test.py                 # 只测链路/遥测/ARM
    python3 tools/usb_bridge_test.py --drive 0.05    # 额外低速转轮 2 s（会动！）

检查项：
  1. 遥测帧率/版本/校验和；
  2. 下行速度帧能被 MCU 正确回显；
  3. 未 ARM 时 dry-run 轮速 req_left/req_right 与运动学换算一致；
  4. ARM 成功、旧指令不会在使能瞬间生效；
  5. DISARM 后 armed/targets 归零。
"""

import argparse
import os
import sys
import time

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'robot_ros2_ws', 'src',
                                'robot_base_bringup', 'scripts'))

from chassis_protocol import (  # noqa: E402
    FLAG_ARMED,
    FLAG_CMD_FRESH,
    FLAG_CMD_TIMEOUT,
    FLAG_IMU_ONLINE,
    FLAG_MOTORS_READY,
    SERVICE_ARM,
    SERVICE_DISARM,
    TelemetryStream,
    result_name,
    service_frame,
    speed_frame,
)

WHEEL_RADIUS_M = 0.041713  # 2026-11-05 有效滚动半径
TRACK_WIDTH_M = 0.148


class Bench:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0,
                                 write_timeout=0.2, exclusive=True)
        self.stream = TelemetryStream()
        self.last = None
        self.frames = 0

    def close(self):
        self.ser.close()

    def pump(self, seconds, send=None, period=0.02):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if send is not None:
                self.ser.write(send())
            time.sleep(period)
            for msg in self.stream.feed(self.ser.read(512)):
                self.last = msg
                self.frames += 1
        return self.last

    def print_state(self, label):
        m = self.last
        if m is None:
            print(f'{label}: no telemetry')
            return
        print(f"{label}: armed={bool(m['flags'] & FLAG_ARMED)} "
              f"motors_ready={bool(m['flags'] & FLAG_MOTORS_READY)} "
              f"timeout={bool(m['flags'] & FLAG_CMD_TIMEOUT)} "
              f"fresh={bool(m['flags'] & FLAG_CMD_FRESH)} "
              f"targets=({m['left_target']:+.3f},{m['right_target']:+.3f}) "
              f"speed=({m['left_speed']:+.3f},{m['right_speed']:+.3f}) "
              f"result={result_name(m['last_result'])}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/stm32')
    parser.add_argument('--drive', type=float, default=0.0,
                        help='ARM 后以该线速度(m/s)转轮 2 s；默认 0 = 不驱动')
    args = parser.parse_args()
    if args.drive < 0 or args.drive > 0.10:
        raise SystemExit('--drive 只允许 0..0.10 m/s（台架测试）')

    bench = Bench(args.port)
    ok = True
    try:
        print('1) telemetry ...')
        m = bench.pump(1.5)
        if m is None:
            raise SystemExit('FAIL: no telemetry')
        print(f"   rate ~{bench.frames / 1.5:.1f} Hz version={m['version']} "
              f"imu_online={bool(m['flags'] & FLAG_IMU_ONLINE)} "
              f"rpy=({m['roll_deg']:+.2f},{m['pitch_deg']:+.2f},{m['yaw_deg']:+.2f}) deg")
        if not (m['flags'] & FLAG_IMU_ONLINE):
            print('   WARN: JY901S IMU offline')

        print('2) zero command echo ...')
        m = bench.pump(0.5, lambda: speed_frame(0.0, 0.0, mode=3))
        if abs(m['cmd_vx']) > 1e-6 or abs(m['cmd_vz']) > 1e-6:
            ok = False
            print('   FAIL: zero command not echoed')

        print('3) dry-run kinematics (not armed, no motion) ...')
        linear, angular = 0.10, 0.20
        m = bench.pump(0.6, lambda: speed_frame(linear, -angular, mode=3))
        exp_left = (linear - angular * TRACK_WIDTH_M / 2) / WHEEL_RADIUS_M
        exp_right = (linear + angular * TRACK_WIDTH_M / 2) / WHEEL_RADIUS_M
        print(f"   cmd echo=({m['cmd_vx']:+.3f},{m['cmd_vz']:+.3f}) "
              f"req=({m['req_left']:+.3f},{m['req_right']:+.3f}) "
              f"expected=({exp_left:+.3f},{exp_right:+.3f})")
        if abs(m['req_left'] - exp_left) > 0.02 or abs(m['req_right'] - exp_right) > 0.02:
            ok = False
            print('   FAIL: dry-run wheel conversion mismatch')
        if abs(m['left_target']) > 1e-6 or abs(m['right_target']) > 1e-6:
            ok = False
            print('   FAIL: disarmed MCU applied a target')

        print('4) ARM ...')
        bench.ser.write(service_frame(SERVICE_ARM))
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            bench.pump(0.1)
            if bench.last and (bench.last['flags'] & FLAG_ARMED):
                break
        bench.print_state('   after ARM')
        if not (bench.last and (bench.last['flags'] & FLAG_ARMED)):
            ok = False
            print('   FAIL: ARM not accepted (motors offline / CAN / power?)')
        else:
            # ARM 后故意先不发新帧：旧的非零指令不得生效。
            bench.pump(0.6)
            if abs(bench.last['left_target']) > 1e-6 or abs(bench.last['right_target']) > 1e-6:
                ok = False
                print('   FAIL: stale command applied right after ARM')
            bench.pump(0.5, lambda: speed_frame(0.0, 0.0, mode=3))
            bench.print_state('   zero frames')

        if args.drive > 0 and bench.last and (bench.last['flags'] & FLAG_ARMED):
            print(f'5) DRIVE {args.drive:.2f} m/s for 2 s ...')
            bench.pump(2.0, lambda: speed_frame(args.drive, 0.0, mode=3))
            for _ in range(3):
                bench.pump(0.05, lambda: speed_frame(0.0, 0.0, mode=3))
            bench.print_state('   after drive')

        print('6) DISARM ...')
        bench.ser.write(service_frame(SERVICE_DISARM))
        bench.pump(0.8, lambda: speed_frame(0.0, 0.0, mode=3))
        bench.print_state('   after DISARM')
        if bench.last and (bench.last['flags'] & FLAG_ARMED):
            ok = False
            print('   FAIL: still armed')
        if bench.last and (bench.last['flags'] & FLAG_CMD_TIMEOUT):
            ok = False
            print('   FAIL: stale CMD_TIMEOUT flag after DISARM')
    finally:
        try:
            bench.ser.write(service_frame(SERVICE_DISARM))
            bench.ser.write(speed_frame(0.0, 0.0, mode=3))
        except (serial.SerialException, OSError):
            pass
        bench.close()

    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
