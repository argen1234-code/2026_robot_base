"""Regression for the deployed camera's measured image/yaw response."""
import json
from pathlib import Path
import unittest

from robot_base_decision.visual_follow import VisualFollower, validate_config


ROOT = Path(__file__).resolve().parents[4]


class VisualDirectionTest(unittest.TestCase):
    def test_deployed_configuration_centers_both_sides_in_actual_image_response(self):
        config = json.loads((ROOT/'config/visual_follow.json').read_text())
        # Physical probe: +0.18 rad/s moves image features left (-75 px);
        # -0.18 rad/s moves them right (+70 px). Thus dx/dyaw is negative.
        for initial_center in (150., 490.):
            with self.subTest(initial_center=initial_center):
                controller = VisualFollower(config)
                controller.start(follow=True)
                center, now = initial_center, 100.
                first_error = abs(center-320.)
                for _ in range(80):
                    packet = dict(source_stamp=now, depth_source_stamp=now,
                        image_size=[640, 480], persons=[dict(
                            bbox=[center-60., 80., 120., 300.], conf=.9, depth_m=.7)])
                    result = controller.step(packet, 0., now+.01)
                    corrected = center-500.*result['angular_rps']*.05
                    self.assertLessEqual(abs(corrected-320.), abs(center-320.)+1e-7)
                    center = corrected
                    now += .05
                self.assertLess(abs(center-320.), first_error*.15)

    def test_original_polarity_moves_target_farther_from_center(self):
        controller = VisualFollower({'image_yaw_sign': -1.})
        controller.start()
        for center in (150., 490.):
            controller.start()
            packet = dict(source_stamp=100., depth_source_stamp=100.,
                image_size=[640, 480], persons=[dict(
                    bbox=[center-60., 80., 120., 300.], conf=.9, depth_m=.7)])
            result = controller.step(packet, 0., 100.1)
            moved = center-500.*result['angular_rps']*.05
            self.assertGreater(abs(moved-320.), abs(center-320.))

    def test_polarity_is_explicitly_binary(self):
        for sign in (0., .5, 2.):
            with self.subTest(sign=sign), self.assertRaises(ValueError):
                validate_config({'image_yaw_sign': sign})


if __name__ == '__main__':
    unittest.main()
