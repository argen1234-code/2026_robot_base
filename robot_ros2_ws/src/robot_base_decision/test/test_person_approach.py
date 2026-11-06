import json
import math
from pathlib import Path
import unittest
from robot_base_decision.person_approach import (
    StableTarget, TargetMemory, associate_person, grid_cost, moving_away,
    project_base_point, relative_transform, scan_clusters,
    select_centered_person, standoff_goal, transform)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        config = Path(__file__).with_name('person_approach.json')
        if not config.exists():
            config = Path(__file__).resolve().parents[4]/'config/person_approach.json'
        self.c = json.loads(config.read_text())
        self.c.update(camera_xyz_m=[0., 0., .5], camera_pitch_down_deg=0.)
        self.tf = ((0., 0., .1), (0., 0., 0., 1.))

    def test_camera_right_left_and_pitch(self):
        self.assertLess(project_base_point((2, .2, .1), self.c, 640, 480)[0], self.c['cx'])
        self.assertGreater(project_base_point((2, -.2, .1), self.c, 640, 480)[0], self.c['cx'])
        v = project_base_point((2, 0, .1), self.c, 640, 480)[1]
        self.c['camera_pitch_down_deg'] = 10
        self.assertLess(project_base_point((2, 0, .1), self.c, 640, 480)[1], v)

    def test_range_discontinuity_separates_adjacent_indices(self):
        ranges = [2., 2., 2., 4., 4., 4.]
        self.assertEqual(len(scan_clusters(ranges, -.02, .01, self.c)), 2)

    def test_far_two_point_leg_cluster_survives_mounting_uncertainty(self):
        cluster = [(0, (3., -.035, 0.)), (1, (3., .035, 0.))]
        # With only nominal mounting this projects above the lower body.
        self.c['camera_pitch_down_deg'] = 10.
        self.c['pitch_uncertainty_deg'] = 15.
        box = [270., 20., 100., 400.]
        match, _ = associate_person(box, [640, 480], [cluster], self.tf, self.c)
        self.assertIsNotNone(match)
        self.assertEqual(match['points'], 2)

    def test_previous_position_resolves_wall_side_object(self):
        def cluster(x):
            return [(i, (x, -.1+i*.02, 0.)) for i in range(11)]
        match, _ = associate_person([260, 0, 120, 440], [640, 480],
            [cluster(2), cluster(4)], self.tf,
            dict(self.c, max_person_height_m=4.), prior_base=(2., 0., .1))
        self.assertIsNotNone(match)
        self.assertAlmostEqual(match['base_point'][0], 2.)

    def test_scan_reprojection_accounts_for_robot_turn_and_translation(self):
        base_to_map = ((1., 0., 0.), (0., 0., math.sin(math.pi/4), math.cos(math.pi/4)))
        laser_to_map = ((0., 0., .1), (0., 0., 0., 1.))
        tf = relative_transform(base_to_map, laser_to_map)
        point = transform(*base_to_map, transform(*tf, (2., .3, 0.)))
        for actual, expected in zip(point, (2., .3, .1)):
            self.assertAlmostEqual(actual, expected)

    def test_whole_wall_is_rejected_in_narrow_person_box(self):
        wall = [(i, (2., -.8+i*.02, 0.)) for i in range(81)]
        match, _ = associate_person([270, 100, 100, 350], [640, 480], [wall], self.tf, self.c)
        self.assertIsNone(match)

    def test_person_cluster_associates_and_ambiguous_ranges_rejected(self):
        def cluster(x):
            return [(i, (x, -.1+i*.02, 0.)) for i in range(11)]
        box = [260, 0, 120, 440]
        match, _ = associate_person(box, [640, 480], [cluster(2)], self.tf, self.c)
        self.assertIsNotNone(match)
        self.assertAlmostEqual(match['base_point'][0], 2.)
        match, reason = associate_person(box, [640, 480], [cluster(2), cluster(4)],
            self.tf, dict(self.c, max_person_height_m=4.))
        self.assertIsNone(match)
        self.assertEqual(reason, 'ambiguous_lidar_clusters')

    def test_distant_furniture_behind_large_near_person_box_is_rejected(self):
        cluster = [(i, (3.1, -.08+i*.02, 0.)) for i in range(9)]
        match, _ = associate_person([170, 0, 300, 475], [640, 480], [cluster],
                                   self.tf, dict(self.c, pitch_uncertainty_deg=20.))
        self.assertIsNone(match)

    def test_near_cropped_feet_still_associate_visible_person_with_lidar(self):
        cluster = [(i, (1.1, -.08+i*.02, 0.)) for i in range(9)]
        config = dict(self.c, camera_pitch_down_deg=-10., pitch_uncertainty_deg=5.)
        match, _ = associate_person([180, 0, 280, 478], [640, 480], [cluster],
                                   self.tf, config)
        self.assertIsNotNone(match)

    def test_multiple_people_select_nearest_image_center_with_margin(self):
        persons = [{'bbox': [10., 0., 100., 400.]},
                   {'bbox': [270., 0., 100., 400.]}]
        chosen, reason = select_centered_person(persons, [640, 480])
        self.assertIs(chosen, persons[1])
        self.assertEqual(reason, 'centered_person')
        tied = [{'bbox': [240., 0., 140., 400.]},
                {'bbox': [260., 0., 140., 400.]}]
        chosen, reason = select_centered_person(tied, [640, 480])
        self.assertIsNone(chosen)
        self.assertEqual(reason, 'ambiguous_person_choice')
        self.assertEqual(select_centered_person([], [640, 480]),
                         (None, 'no_person'))

    def test_moving_away_guard_requires_motion_and_distance_gain(self):
        moving_back = [(0., 2.0, 0., 0.), (.5, 2.1, -.1, 0.),
                       (1., 2.2, -.2, 0.), (1.5, 2.3, -.3, 0.),
                       (2., 2.4, -.4, 0.), (2.5, 2.5, -.5, 0.)]
        self.assertTrue(moving_away(moving_back, 2.5))
        # Person walks away while the robot is stationary: do not cancel.
        walking = [(t, 2.0+.1*t, 0., 0.) for t in (0., .5, 1., 1.5, 2., 2.5)]
        self.assertFalse(moving_away(walking, 2.5))
        # Normal approach: robot moves forward and the distance shrinks.
        approaching = [(t, 2.0-.2*t, .2*t, 0.) for t in (0., .5, 1., 1.5, 2., 2.5)]
        self.assertFalse(moving_away(approaching, 2.5))
        # Not enough history yet.
        self.assertFalse(moving_away(moving_back[:3], .5))

    def test_repeated_frame_and_weak_confidence_cannot_confirm(self):
        tracker = StableTarget()
        box, point = [0, 0, 10, 20], (2, 0, 0)
        for _ in range(10):
            self.assertIsNone(tracker.add(1., box, point))
        self.assertIsNone(tracker.add(1.2, box, point, strong=False))
        self.assertIsNone(tracker.add(1.4, box, point))
        self.assertIsNotNone(tracker.add(1.6, box, point))
        self.assertIsNone(tracker.add(1.8, box, (4, 0, 0)))

    def test_standoff_is_free_and_faces_person(self):
        grid = dict(x=-5., y=-5., resolution=.1, width=100, height=100, data=[0]*10000)
        goal, _ = standoff_goal((3, 0, 0), (0, 0, 0), 1., grid)
        self.assertAlmostEqual(goal['x'], 2.)
        self.assertAlmostEqual(goal['yaw'], 0.)
        self.assertEqual(standoff_goal((.8, 0, 0), (0, 0, 0), 1., grid)[1], 'already_close')
        grid['data'] = [-1]*10000
        self.assertIsNone(standoff_goal((3, 0, 0), (0, 0, 0), 1., grid)[0])

    def test_rotated_costmap(self):
        grid = dict(x=1., y=2., yaw=math.pi/2, resolution=1., width=2, height=2, data=[1, 2, 3, 4])
        self.assertEqual(grid_cost(grid, .5, 2.5), 1)

    def test_thirty_centimeter_gap_is_measured_from_vehicle_front(self):
        grid = dict(x=-5., y=-5., resolution=.1, width=100, height=100, data=[0]*10000)
        for angle in (0., math.pi/3, -math.pi/2):
            person = (2*math.cos(angle), 2*math.sin(angle), 0.)
            goal, reason = standoff_goal(person, (0., 0., 0.), .3, grid,
                                        front_offset=.246, arrival_tolerance=.02)
            self.assertEqual(reason, 'goal_ready')
            front = (goal['x']+.246*math.cos(goal['yaw']),
                     goal['y']+.246*math.sin(goal['yaw']))
            self.assertAlmostEqual(math.dist(front, person[:2]), .3)
        self.assertEqual(standoff_goal((.56, 0., 0.), (0., 0., 0.), .3, grid,
            front_offset=.246, arrival_tolerance=.02)[1], 'already_close')
        self.assertIsNotNone(standoff_goal((.8, 0., 0.), (0., 0., 0.), .3, grid,
            front_offset=.246, arrival_tolerance=.02)[0])

    def test_target_memory_requires_fresh_confirmations_and_rejects_jump(self):
        memory = TargetMemory()
        box, point = [200., 0., 150., 400.], (2., 0., .1)
        self.assertIsNotNone(memory.observe(1., 1., box, point)[0])
        self.assertIsNone(memory.observe(2., 2., box, (4., 0., .1))[0])
        self.assertIsNone(memory.observe(2.1, 2.1, box, point)[0])
        self.assertIsNone(memory.observe(2.2, 2.1, box, point)[0])
        self.assertIsNotNone(memory.observe(2.3, 2.3, box, point)[0])
        self.assertTrue(memory.reacquired)
        last_seen = memory.last_seen
        self.assertIsNone(memory.observe(2.4, 2.4, box, point, strong=False)[0])
        self.assertEqual(memory.last_seen, last_seen)


if __name__ == '__main__':
    unittest.main()
