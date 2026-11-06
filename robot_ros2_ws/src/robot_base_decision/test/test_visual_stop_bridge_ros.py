"""Production vision + production chassis bridge on domain 79 with fake USB."""
import argparse
import json
import os
from pathlib import Path
import struct
import sys
import time
import unittest

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from robot_base_decision.visual_follow_node import VisualFollowNode

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'robot_base_bringup/scripts'))
from chassis_bridge import ChassisBridge


class FakeUsb:
    def __init__(self):
        self.frames = []
        self.in_waiting = 0

    def write(self, frame):
        self.frames.append(frame)

    def read(self, size):
        return b''

    def close(self):
        pass

    def flush(self):
        pass


class FastStopBridgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROS_DOMAIN_ID'] = '79'
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.bridge = ChassisBridge()
        self.bridge.device = FakeUsb()
        self.bridge.mode, self.bridge.enabled = 2, True
        self.bridge.max_linear, self.bridge.max_angular = .24, .55

    def tearDown(self):
        self.bridge.stop()
        self.bridge.destroy_node()

    def command(self, linear, angular):
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        self.bridge.on_remote(command)

    def test_remote_stop_writes_usb_frame_without_timer(self):
        self.command(.24, .55)
        self.assertEqual(self.bridge.device.frames, [])
        self.command(0., 0.)
        frame = self.bridge.device.frames[-1]
        self.assertEqual(frame[:3], b'\xaa\x55\x02')
        self.assertEqual(struct.unpack('<ff', frame[3:11]), (0., 0.))

    def test_new_speed_limits_reach_usb_without_old_clamping(self):
        self.bridge.armed = True
        self.command(.24, .55)
        self.bridge.tick()
        frame = self.bridge.device.frames[-1]
        vx, wz = struct.unpack('<ff', frame[3:11])
        self.assertAlmostEqual(vx, -.24)
        self.assertAlmostEqual(wz, -.55)

    def test_navigation_mode_is_not_stopped_by_remote_zero(self):
        self.bridge.mode = 3
        self.command(.2, .45)
        self.command(0., 0.)
        self.assertEqual(self.bridge.device.frames, [])

    def test_one_arrival_sample_sends_zero_to_usb_then_jitter_stays_stopped(self):
        vision = VisualFollowNode(argparse.Namespace(config='', distance=.05,
            execute=True, auto=False, follow=False, http_port=0, http_host='127.0.0.1'))
        try:
            # Connect production callbacks with an in-memory Twist transport.
            # Neither a hardware device nor a timer is required for arrival.
            def send(linear=0., angular=0.):
                self.command(linear, angular)
            vision.publish_velocity = send
            vision.controller.start()
            for depth in (.15, .069, .18, .03, .1):
                now = time.time()
                vision.on_detection(String(data=json.dumps(dict(
                    source_stamp=now, depth_source_stamp=now, image_size=[640, 480],
                    persons=[dict(bbox=[260., 80., 120., 300.], conf=.9, depth_m=depth)]))))
            self.assertEqual(struct.unpack('<ff', self.bridge.device.frames[-1][3:11]), (0., 0.))
            self.assertTrue(vision.snapshot['arrived_latched'])
            self.assertEqual(vision.snapshot['distance_m'], .069)
            self.assertEqual(self.bridge.remote_command, (0., 0.))
        finally:
            vision.close()
            vision.destroy_node()


if __name__ == '__main__':
    unittest.main()
