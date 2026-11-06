import math
import time
import unittest

from robot_base_decision.person_centering import FootprintGuard, centered_command


class CenteringTests(unittest.TestCase):
    def command(self, center, v=.15, w=0., width=640, fx=570):
        return centered_command(v, w, dict(width=width, center_x=center, fx=fx), {})

    def test_image_right_turns_right_and_left_turns_left(self):
        self.assertLess(self.command(400)[1], 0.)
        self.assertGreater(self.command(240)[1], 0.)

    def test_center_band_suppresses_box_jitter(self):
        for center in (300, 320, 340):
            v, w, error = self.command(center)
            self.assertEqual(v, .15)
            self.assertEqual(w, 0.)

    def test_large_error_stops_translation_before_turning(self):
        self.assertEqual(self.command(500)[0], 0.)
        self.assertLess(self.command(500)[1], 0.)
        self.assertGreater(self.command(390)[0], 0.)
        self.assertLess(self.command(390)[0], .15)

    def test_stopped_navigation_cannot_start_a_visual_turn(self):
        self.assertEqual(self.command(500, v=0., w=0.)[:2], (0., 0.))

    def test_resize_keeps_turn_direction_and_scale(self):
        a = self.command(400)
        b = self.command(200, width=320, fx=285)
        self.assertEqual(a[:2], b[:2])

    def test_invalid_box_and_motion_are_rejected(self):
        for center in (float('nan'), -1, 641):
            with self.assertRaises(ValueError):
                self.command(center)
        with self.assertRaises(ValueError):
            self.command(320, v=float('nan'))

    def grid(self, obstacles=(), unknown=False, yaw=0.):
        data = [-1 if unknown else 0]*10000
        # Obstacles are expressed in the unrotated 3x3m grid frame.
        for x, y in obstacles:
            col, row = int((x+1.5)/.03), int((y+1.5)/.03)
            data[row*100+col] = 100
        return FootprintGuard(data, 100, 100, .03, (-1.5, -1.5), yaw)

    def test_clear_rollout_and_obstacle_on_future_body_path(self):
        self.assertTrue(self.grid().clear((0., 0., 0.), .15, .3))
        self.assertFalse(self.grid([(.36, .05)]).clear((0., 0., 0.), .15, 0.))

    def test_rotating_front_corner_is_checked(self):
        guard = self.grid([(.18, .21)])
        self.assertTrue(guard.clear((0., 0., 0.), 0., 0.))
        self.assertFalse(guard.clear((0., 0., 0.), 0., .3))

    def test_unknown_and_outside_grid_stop_motion(self):
        self.assertFalse(self.grid(unknown=True).clear((0., 0., 0.), .15, 0.))
        self.assertFalse(self.grid().clear((1.3, 0., 0.), .15, 0.))

    def test_grid_rotation_is_respected(self):
        # Rotate the grid about its origin; its center moves to (-3, 0).
        guard = self.grid(yaw=math.pi/2)
        self.assertTrue(guard.clear((-3., 0., math.pi/2), .15, .1))
        self.assertFalse(guard.clear((0., 0., 0.), .15, 0.))

    def test_guard_runtime(self):
        guard = self.grid([(x*.03, y*.03) for x in range(-40, 41)
                           for y in (-40, -39, 39, 40)])
        started = time.perf_counter()
        for _ in range(30):
            self.assertTrue(guard.clear((0., 0., 0.), .15, .3))
        print(f'Centering footprint check: {(time.perf_counter()-started)*1000/30:.2f} ms/call')


if __name__ == '__main__':
    unittest.main()
