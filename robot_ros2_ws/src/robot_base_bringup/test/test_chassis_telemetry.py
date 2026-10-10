"""telemetry 解析 + 轮速里程计算子的主机单测（不需要 ROS、不需要硬件）。

用例里的数值取自 2026-10-10 的真机实测：
  - 原地左转（VZ_SCALE=60，指令 0.45 rad/s）时四轮为 [-26,+26,-26,+26]（FL,FR,RL,RR）
  - 静止时四轮恰好为 0，字节流实测 14.2 帧/s、重同步丢弃 0 字节
"""

import math
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from chassis_telemetry import (  # noqa: E402
    FRAME_SIZES, OFF, OFF_V2, FLAG_IMU_VALID, FLAG_MAG_VALID,
    DEG2RAD, G_TO_MS2, WHEEL_TRACK_M, FrameAssembler,
    parse_frame, wheel_twist_from_counts)

K = 12000.0        # 占位标定系数（未标定，见 README）
DT = 0.035         # 实测控制环周期 35 ms
REST = 1.0


def build_frame(version=2, motor_speed=(0.0, 0.0, 0.0, 0.0),
                gyro_z=0.0, acc_z=1.0, flags=None):
    size = FRAME_SIZES[version]
    buf = bytearray(size)
    buf[0], buf[1] = 0xCC, 0x55
    buf[OFF['version']] = version
    buf[OFF['flags']] = (FLAG_MAG_VALID | FLAG_IMU_VALID) if flags is None else flags
    struct.pack_into('<H', buf, OFF['sequence'], 7)
    struct.pack_into('<f', buf, OFF['gyro_z'], gyro_z)
    struct.pack_into('<f', buf, OFF['acc_z'], acc_z)
    for i, v in enumerate(motor_speed):
        struct.pack_into('<f', buf, OFF['motor_speed'] + 4 * i, v)
    if version >= 2:
        struct.pack_into('<H', buf, OFF_V2['loop_period_ms'], 35)
    checksum = 0
    for b in buf[2:size - 1]:
        checksum ^= b
    buf[size - 1] = checksum
    return bytes(buf)


class FrameParsingTest(unittest.TestCase):
    def test_v1_and_v2_both_parse_with_correct_units(self):
        for ver in (1, 2):
            got = parse_frame(build_frame(ver, gyro_z=-1.83, acc_z=0.97))
            self.assertIsNotNone(got, f'v{ver} 应能解析')
            self.assertEqual(got['version'], ver)
            # 保持固件原始单位：陀螺是 °/s、加计是 g
            self.assertAlmostEqual(got['gyro_z'], -1.83, places=3)
            self.assertAlmostEqual(got['acc_z'], 0.97, places=3)

    def test_bad_frames_are_rejected(self):
        good = build_frame(2)
        bad_ck = bytearray(good)
        bad_ck[-1] ^= 0xFF
        self.assertIsNone(parse_frame(bytes(bad_ck)), '坏校验和应被拒')
        bad_ver = bytearray(good)
        bad_ver[OFF['version']] = 9
        self.assertIsNone(parse_frame(bytes(bad_ver)), '未知版本应被拒')
        self.assertIsNone(parse_frame(good[:151]), 'v2 截成 151 不应通过')
        self.assertIsNone(parse_frame(build_frame(1) + b'\x00' * 32), 'v1 补长不应通过')

    def test_unit_conversions(self):
        # 这两条是曾经踩过的坑，写成断言防止回归
        self.assertAlmostEqual(90.0 * DEG2RAD, math.pi / 2)      # °/s -> rad/s
        self.assertAlmostEqual(1.0 * G_TO_MS2, 9.80665)          # g -> m/s²
        self.assertAlmostEqual(WHEEL_TRACK_M, 0.2804)            # 实测轮距


class FrameAssemblerTest(unittest.TestCase):
    def test_reassembles_split_and_noisy_stream(self):
        f1, f2 = build_frame(2), build_frame(2)
        asm = FrameAssembler()
        out = []
        out += asm.feed(b'\x00\x11' + f1[:40])   # 噪声 + f1 的前 40 字节
        out += asm.feed(f1[40:])                 # f1 补齐 -> 第 1 帧
        out += asm.feed(f2[:60])                 # f2 的前 60 字节
        out += asm.feed(f2[60:])                 # f2 补齐 -> 第 2 帧
        self.assertEqual(len(out), 2, '应恰好解出两帧')
        self.assertTrue(asm.resync_bytes > 0, '应记录到被丢弃的同步字节')

    def test_unknown_version_does_not_block(self):
        bad = bytearray(build_frame(2))
        bad[OFF['version']] = 9            # 未知版本
        asm = FrameAssembler()
        out = asm.feed(bytes(bad[:3]) + build_frame(2))
        self.assertEqual(len(out), 1, '未知版本只应跳过 magic，不能堵住后面的好帧')


class WheelOdomTest(unittest.TestCase):
    def test_stationary_counts_give_zupt(self):
        # 实测静止时四轮恰好为 0
        vx, vy, wz, still = wheel_twist_from_counts([0, 0, 0, 0], DT, K,
                                                    WHEEL_TRACK_M, REST)
        self.assertTrue(still)
        self.assertEqual((vx, vy, wz), (0.0, 0.0, 0.0))

    def test_plus_minus_one_jitter_still_counts_as_rest(self):
        # ⚠️ 这是回归测试：阈值若用严格小于，±1 抖动会被判成"在动"、ZUPT 失效
        for counts in ([1, -1, 1, -1], [1, 1, 1, 1], [-1, 0, 0, 1]):
            _, _, _, still = wheel_twist_from_counts(counts, DT, K,
                                                     WHEEL_TRACK_M, REST)
            self.assertTrue(still, f'{counts} 应判为静止')

    def test_in_place_left_turn_sign_matches_rep103(self):
        # 实测原地左转：FL,FR,RL,RR = [-26,+26,-26,+26]
        vx, vy, wz, still = wheel_twist_from_counts([-26, 26, -26, 26], DT, K,
                                                    WHEEL_TRACK_M, REST)
        self.assertFalse(still)
        self.assertAlmostEqual(vx, 0.0, places=9, msg='纯自转不应有平移')
        self.assertGreater(wz, 0.0, '左转（逆时针）必须为 wz > 0（REP-103）')

    def test_forward_drive_is_vx_positive_and_no_yaw(self):
        # ⚠️ 本固件【前进时四轮计数为负】（固件 Vx 与 ROS 相反，见
        #    wheel_twist_from_counts 的推导）。所以这里用【负计数】表示前进，
        #    并要求输出的 vx 为【正】。这条曾经写反，导致发布的 vx 与指令反向。
        vx, vy, wz, still = wheel_twist_from_counts([-24, -24, -24, -24], DT, K,
                                                    WHEEL_TRACK_M, REST)
        self.assertFalse(still)
        self.assertGreater(vx, 0.0, '前进(-24 计数) 必须给出 vx > 0')
        self.assertAlmostEqual(wz, 0.0, places=9)
        self.assertAlmostEqual(vy, 0.0, places=9)

    def test_backward_counts_give_negative_vx(self):
        vx, _, _, _ = wheel_twist_from_counts([24, 24, 24, 24], DT, K,
                                             WHEEL_TRACK_M, REST)
        self.assertLess(vx, 0.0, '正计数=后退 -> vx < 0')

    def test_invert_flips_every_channel(self):
        a = wheel_twist_from_counts([-24, -24, -24, -24], DT, K, WHEEL_TRACK_M, REST)
        b = wheel_twist_from_counts([-24, -24, -24, -24], DT, K, WHEEL_TRACK_M, REST,
                                    invert=True)
        self.assertGreater(a[0], 0.0, '默认口径: -24 计数 = 前进')
        self.assertLess(b[0], 0.0, 'invert 后整体取反')

    def test_scale_is_linear_and_period_matters(self):
        # 加倍计数 -> 加倍速度；周期减半 -> 速度加倍（这两条是"周期会变"的防线）
        v1 = wheel_twist_from_counts([24, 24, 24, 24], DT, K, WHEEL_TRACK_M, REST)[0]
        v2 = wheel_twist_from_counts([48, 48, 48, 48], DT, K, WHEEL_TRACK_M, REST)[0]
        v3 = wheel_twist_from_counts([24, 24, 24, 24], DT / 2, K, WHEEL_TRACK_M, REST)[0]
        self.assertAlmostEqual(v2, 2 * v1, places=9)
        self.assertAlmostEqual(v3, 2 * v1, places=9)

    def test_zero_period_falls_back_not_divides_by_zero(self):
        vx, _, _, _ = wheel_twist_from_counts([24, 24, 24, 24], 0.0, K,
                                             WHEEL_TRACK_M, REST)
        self.assertTrue(math.isfinite(vx))


if __name__ == '__main__':
    unittest.main()
