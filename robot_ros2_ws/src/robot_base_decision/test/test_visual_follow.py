"""Behavior checks for the direct RGB/depth control path, without ROS."""
import copy
from pathlib import Path
import sys
import unittest

import numpy as np

from robot_base_decision.visual_follow import VisualFollower, validate_config

sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'tools'))
from person_depth import bbox_depth


def detection(depth=1., center=320., stamp=100., depth_stamp=100.):
    return dict(source_stamp=stamp, depth_source_stamp=depth_stamp,
                image_size=[640, 480], persons=[dict(
                    bbox=[center-60, 80, 120, 300], conf=.9, depth_m=depth)])


class VisualFollowTest(unittest.TestCase):
    def setUp(self):
        self.controller = VisualFollower({'target_distance_m': .20})
        self.controller.start()

    def step(self, packet=None, age=0., now=100.1):
        return self.controller.step(packet if packet is not None else detection(), age, now)

    def test_forward_with_no_navigation_or_laser(self):
        result = self.step()
        self.assertGreater(result['linear_mps'], 0)
        self.assertEqual(result['angular_rps'], 0)

    def test_increased_speed_limits_and_gain_apply_below_saturation(self):
        self.controller = VisualFollower({'target_distance_m': .05})
        self.controller.start()
        result = self.step(detection(depth=.16))
        self.assertAlmostEqual(result['linear_mps'], .198)
        self.assertAlmostEqual(self.step(detection(depth=1.))['linear_mps'], .24)
        self.controller.start()
        self.assertAlmostEqual(abs(self.step(detection(center=490))['angular_rps']), .55)

    def test_left_right_sign_and_off_center_stops_forward(self):
        left = self.step(detection(center=150))
        self.assertGreater(left['angular_rps'], 0)
        self.assertEqual(left['linear_mps'], 0)
        self.controller.start()
        right = self.step(detection(center=490))
        self.assertLess(right['angular_rps'], 0)
        self.assertEqual(right['linear_mps'], 0)

    def test_initial_invalid_depth_creeps_with_live_stream(self):
        result = self.step(detection(depth=None))
        self.assertEqual(result['state'], 'approaching_without_depth')
        self.assertEqual(result['linear_mps'], .07)

    def test_missing_frozen_or_unsynchronized_depth_uses_rgb_in_both_modes(self):
        for follow in (False, True):
            for stamp in (None, 98., 99.8, 101., 'bad', float('nan'), float('inf')):
                with self.subTest(follow=follow, stamp=stamp):
                    self.controller.start(follow=follow)
                    # A stale close reading must neither block RGB motion nor
                    # be mistaken for a current arrival measurement.
                    result = self.step(detection(depth=.02, depth_stamp=stamp, center=350))
                    self.assertEqual(result['state'], 'approaching_without_depth')
                    self.assertAlmostEqual(result['linear_mps'], .07)
                    self.assertLess(result['angular_rps'], 0)
                    self.assertIsNone(result['distance_m'])
                    self.assertTrue(result['active'])
            self.controller.start(follow=follow)
            packet = detection(depth=.02)
            del packet['depth_source_stamp']
            self.assertAlmostEqual(self.step(packet)['linear_mps'], .07)

    def test_depth_loss_after_measurement_keeps_approaching_in_both_modes(self):
        for follow in (False, True):
            for depth in (None, 0., -.1, 9., 'bad', float('nan'), float('inf')):
                with self.subTest(follow=follow, depth=depth):
                    self.controller.start(follow=follow)
                    self.assertGreater(self.step(detection(depth=.4))['linear_mps'], 0)
                    lost = self.step(detection(depth=depth, center=350))
                    self.assertEqual(lost['state'], 'approaching_without_depth')
                    self.assertAlmostEqual(lost['linear_mps'], .07)
                    self.assertLess(lost['angular_rps'], 0)
                    self.assertGreater(self.step(detection(depth=.5))['linear_mps'], 0)

    def test_follow_already_holding_waits_for_valid_farther_depth(self):
        self.controller.start(follow=True)
        self.assertEqual(self.step(detection(depth=.2))['state'], 'holding_distance')
        for packet in (detection(depth=None, center=350),
                       detection(depth=.5, depth_stamp=98., center=350)):
            result = self.step(packet)
            self.assertEqual(result['state'], 'holding_without_depth')
            self.assertEqual(result['linear_mps'], 0)
            self.assertLess(result['angular_rps'], 0)
        self.assertEqual(self.step(detection(depth=.23))['linear_mps'], 0)
        self.assertGreater(self.step(detection(depth=.3))['linear_mps'], 0)

    def test_one_shot_latches_arrival_at_twenty_centimeters(self):
        for distance in (.20, .19, .15):
            with self.subTest(distance=distance):
                self.controller.start()
                arrived = self.step(detection(depth=distance))
                self.assertEqual(arrived['state'], 'arrived')
                self.assertFalse(arrived['active'])
                self.assertEqual(arrived['linear_mps'], 0)
                self.assertEqual(self.step(detection(depth=1.))['linear_mps'], 0)

    def test_one_shot_first_arrival_stops_both_axes_and_ignores_depth_jitter(self):
        self.controller = VisualFollower({'target_distance_m': .05})
        self.controller.start()
        self.assertGreater(self.step(detection(depth=.15))['linear_mps'], 0)
        arrived = self.step(detection(depth=.049, center=350))
        self.assertEqual(arrived['state'], 'arrived')
        self.assertTrue(arrived['arrived_latched'])
        self.assertEqual((arrived['linear_mps'], arrived['angular_rps']), (0., 0.))
        for depth in (.06, .03, .20, None, .051):
            result = self.step(detection(depth=depth, center=350))
            self.assertEqual(result['state'], 'arrived')
            self.assertFalse(result['active'])
            self.assertTrue(result['arrived_latched'])
            self.assertEqual(result['distance_m'], .049)
            self.assertEqual((result['linear_mps'], result['angular_rps']), (0., 0.))
        self.controller.start()
        self.assertGreater(self.step(detection(depth=.15))['linear_mps'], 0)

    def test_follow_holds_never_reverses_and_resumes_after_margin(self):
        self.controller.start(follow=True)
        for distance in (.2, .15, .23):
            result = self.step(detection(depth=distance))
            self.assertEqual(result['state'], 'holding_distance')
            self.assertTrue(result['active'])
            self.assertEqual(result['linear_mps'], 0)
        self.assertGreater(self.step(detection(depth=.3))['linear_mps'], 0)

    def test_single_approach_stops_at_outer_deadband_and_latches(self):
        self.controller = VisualFollower({'target_distance_m': .05})
        self.controller.start()
        self.assertGreater(self.step(detection(depth=.071))['linear_mps'], 0)
        reached = self.step(detection(depth=.07, center=350))
        self.assertEqual(reached['state'], 'arrived')
        self.assertAlmostEqual(reached['stop_distance_m'], .07)
        self.assertEqual((reached['linear_mps'], reached['angular_rps']), (0., 0.))
        for depth in (.071, .12, .04, None):
            result = self.step(detection(depth=depth))
            self.assertTrue(result['arrived_latched'])
            self.assertEqual(result['distance_m'], .07)
            self.assertEqual((result['linear_mps'], result['angular_rps']), (0., 0.))

    def test_follow_does_not_use_single_approach_deadband(self):
        self.controller = VisualFollower({'target_distance_m': .05})
        self.controller.start(follow=True)
        result = self.step(detection(depth=.06))
        self.assertGreater(result['linear_mps'], 0)
        self.assertEqual(result['stop_distance_m'], .05)
        self.assertTrue(result['active'])

    def test_no_person_stale_replayed_and_malformed_results_stop(self):
        packets = [dict(source_stamp=100., image_size=[640, 480], persons=[]),
                   detection(stamp=98.), detection(stamp=102.),
                   detection(stamp=float('nan')), {},
                   dict(source_stamp=100., image_size=[0, 0], persons=[])]
        for packet in packets:
            with self.subTest(packet=packet):
                self.controller.start()
                result = self.step(packet)
                self.assertEqual((result['linear_mps'], result['angular_rps']), (0., 0.))
        self.assertEqual(self.step(age=2.)['linear_mps'], 0)

    def test_target_lock_does_not_switch_to_another_person(self):
        self.step()
        result = self.step(detection(center=550))
        self.assertEqual(result['state'], 'target_lost')
        self.assertEqual(result['linear_mps'], 0)

    def test_ambiguous_people_are_not_silently_selected(self):
        packet = detection(center=300)
        other = copy.deepcopy(packet['persons'][0])
        other['bbox'][0] += 40
        packet['persons'].append(other)
        self.assertEqual(self.step(packet)['state'], 'ambiguous_person_choice')

    def test_stop_requires_new_start(self):
        self.controller.stop()
        self.assertEqual(self.step()['linear_mps'], 0)
        self.controller.start()
        self.assertGreater(self.step()['linear_mps'], 0)

    def test_depth_range_and_speed_configuration_validation(self):
        for patch in ({'target_distance_m': .005}, {'target_distance_m': float('nan')},
                      {'max_linear_mps': .5}, {'resume_margin_m': 0},
                      {'approach_deadband_m': -.01}, {'approach_deadband_m': 8.}):
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                validate_config(patch)


class DepthSamplingTest(unittest.TestCase):
    def test_twenty_centimeters_is_not_filtered_in_visual_mode(self):
        depth = np.full((240, 320), 200, dtype=np.uint16)
        # Existing navigation detector threshold remains unchanged by default.
        self.assertIsNone(bbox_depth(depth, [0, 0, 640, 480], (640, 480))[0])
        distance, count = bbox_depth(depth, [0, 0, 640, 480], (640, 480), minimum_m=.1)
        self.assertAlmostEqual(distance, .2)
        self.assertGreater(count, 10)

    def test_zero_and_out_of_range_depth_have_no_distance(self):
        for value in (0, 100, 9000):
            depth = np.full((24, 32), value, dtype=np.uint16)
            self.assertIsNone(bbox_depth(depth, [0, 0, 640, 480], (640, 480), minimum_m=.1)[0])

    def test_body_roi_rejects_background_outside_the_box(self):
        depth = np.full((240, 320), 2000, dtype=np.uint16)
        depth[30:180, 130:190] = 200
        distance, _ = bbox_depth(depth, [260, 60, 120, 300], (640, 480), minimum_m=.1)
        self.assertAlmostEqual(distance, .2)


if __name__ == '__main__':
    unittest.main()
