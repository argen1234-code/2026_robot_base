import math
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from chassis_protocol import speed_frame


class ChassisProtocolTest(unittest.TestCase):
    def test_frame_matches_firmware_layout_and_forward_sign(self):
        frame = speed_frame(0.12, 0.2)
        self.assertEqual(len(frame), 12)
        self.assertEqual(frame[:3], b'\xaa\x55\x03')
        vx, wz = struct.unpack('<ff', frame[3:11])
        self.assertAlmostEqual(vx, -0.12)
        self.assertAlmostEqual(wz, 0.2)
        checksum = 0
        for value in frame[2:11]:
            checksum ^= value
        self.assertEqual(frame[11], checksum)

    def test_clamps_and_zero(self):
        vx, wz = struct.unpack('<ff', speed_frame(1.0, -1.0)[3:11])
        self.assertAlmostEqual(vx, -0.2)
        self.assertAlmostEqual(wz, -0.45)
        self.assertEqual(struct.unpack('<ff', speed_frame(0.0, 0.0)[3:11]), (0.0, 0.0))

    def test_rejects_reverse_and_non_finite(self):
        with self.assertRaises(ValueError):
            speed_frame(-0.01, 0.0)
        with self.assertRaises(ValueError):
            speed_frame(math.nan, 0.0)

    def test_wechat_mode_allows_reverse(self):
        frame = speed_frame(-0.1, -0.2, mode=2)
        self.assertEqual(frame[2], 2)
        vx, wz = struct.unpack('<ff', frame[3:11])
        self.assertAlmostEqual(vx, 0.1)
        self.assertAlmostEqual(wz, -0.2)


if __name__ == '__main__':
    unittest.main()
