"""Real ROS pub/sub tests on domain 79. No serial device or hardware nodes."""
import argparse
from functools import partial
from http.server import ThreadingHTTPServer
import json
import os
import threading
import time
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8, String

from robot_base_decision.visual_follow_node import VisualFollowNode, WebHandler


class VisualFollowRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROS_DOMAIN_ID'] = '79'
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = VisualFollowNode(argparse.Namespace(
            config='', distance=.20, execute=True, auto=False, follow=False,
            http_port=0, http_host='127.0.0.1'))
        self.peer = Node('visual_test_peer')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.peer)
        self.velocities = []
        self.modes = []
        self.peer.create_subscription(Twist, '/remote_cmd_vel',
            lambda msg: self.velocities.append((msg.linear.x, msg.angular.z)), 10)
        self.peer.create_subscription(Int8, '/robot_mode', lambda msg: self.modes.append(msg.data), 10)
        self.detections = self.peer.create_publisher(String, '/perception/person_detections', 1)
        self.stops = self.peer.create_publisher(Bool, '/stop_request', 1)
        self.mode = self.peer.create_publisher(Int8, '/robot_mode', 1)
        self.depth, self.center, self.present = 1., 320., True
        self.stamp_offset = 0.
        self.depth_stamp_offset = 0.
        self.include_depth_stamp = True
        self.peer.create_timer(.05, self.publish_detection)
        self.wait(lambda: bool(self.node.received_at))

    def tearDown(self):
        self.node.close()
        self.executor.remove_node(self.node)
        self.executor.remove_node(self.peer)
        self.node.destroy_node()
        self.peer.destroy_node()
        self.executor.shutdown()

    def publish_detection(self):
        stamp = time.time()+self.stamp_offset
        packet = dict(source_stamp=stamp,
            image_size=[640, 480], persons=[dict(
                bbox=[self.center-60, 80, 120, 300], conf=.9,
                depth_m=self.depth)] if self.present else [])
        if self.include_depth_stamp:
            packet['depth_source_stamp'] = stamp+self.depth_stamp_offset
        self.detections.publish(String(data=json.dumps(packet)))

    def wait(self, predicate, timeout=4.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        self.fail('Timed out; snapshot='+str(self.node.snapshot))

    def test_direct_velocity_and_mode_without_nav_or_scan(self):
        self.node.controller.start()
        self.wait(lambda: any(v > 0 for v, w in self.velocities) and 2 in self.modes)
        self.assertEqual(self.node.snapshot['mode'], 'visual')
        subscriptions = {name for name, _ in self.node.get_subscriber_names_and_types_by_node(
            self.node.get_name(), self.node.get_namespace())}
        self.assertFalse(any('/scan' in name or 'costmap' in name or name == '/tf'
                             for name in subscriptions))

    def test_follow_stops_at_point_two_and_resumes(self):
        self.node.controller.start(follow=True)
        self.wait(lambda: self.node.snapshot.get('output_linear_mps', 0) > 0)
        self.depth = .2
        self.wait(lambda: self.node.snapshot['state'] == 'holding_distance')
        self.assertEqual(self.node.snapshot['output_linear_mps'], 0)
        self.depth = .15
        self.wait(lambda: self.node.snapshot.get('distance_m') == .15)
        self.assertEqual(self.node.snapshot['output_linear_mps'], 0)
        self.depth = .4
        self.wait(lambda: self.node.snapshot['output_linear_mps'] > 0)

    def test_single_arrival_callback_stops_before_timer_and_cannot_be_overwritten(self):
        self.node.controller.cfg['target_distance_m'] = .05
        self.node.controller.start()
        def packet(depth):
            stamp = time.time()
            return String(data=json.dumps(dict(source_stamp=stamp, depth_source_stamp=stamp,
                image_size=[640, 480], persons=[dict(
                    bbox=[290., 80., 120., 300.], conf=.9, depth_m=depth)])))
        with patch.object(self.node, 'publish_velocity', wraps=self.node.publish_velocity) as send:
            self.node.on_detection(packet(.15))
            self.assertGreater(send.call_args.args[0], 0)
            self.node.on_detection(packet(.069))
            self.assertEqual(send.call_args.args, (0., 0.))
            self.assertTrue(self.node.snapshot['arrived_latched'])
            self.assertEqual(self.node.snapshot['state'], 'arrived')
            # A rebound immediately after that sample and before any timer
            # must not erase the stop or produce another motion command.
            self.node.on_detection(packet(.15))
            self.node.tick()
            self.assertEqual(send.call_args.args, (0., 0.))
            self.assertFalse(self.node.controller.active)
            self.assertEqual(self.node.snapshot['distance_m'], .069)

    def test_depth_dropout_keeps_forward_and_centering_in_both_modes(self):
        for follow in (False, True):
            with self.subTest(follow=follow):
                self.depth, self.center = 1., 320.
                self.wait(lambda: self.node.packet['persons'][0]['depth_m'] == 1. and
                          self.node.packet['persons'][0]['bbox'][0] == 260.)
                self.node.controller.start(follow=follow)
                self.wait(lambda: self.node.snapshot.get('distance_m') == 1. and
                          self.node.snapshot.get('output_linear_mps', 0) > 0)
                self.depth, self.center = None, 350.
                self.wait(lambda: self.node.snapshot['state'] == 'approaching_without_depth')
                self.wait(lambda: self.velocities[-1][0] > 0 and self.velocities[-1][1] < 0)
                self.assertAlmostEqual(self.node.snapshot['output_linear_mps'], .07)
                self.depth = .19
                expected = 'holding_distance' if follow else 'arrived'
                self.wait(lambda: self.node.snapshot['state'] == expected)
                self.assertEqual(self.node.snapshot['output_linear_mps'], 0)
                self.depth = None
                self.wait(lambda: self.node.packet['persons'][0]['depth_m'] is None)
                self.assertEqual(self.node.snapshot['output_linear_mps'], 0)

    def test_missing_or_stale_depth_stream_keeps_rgb_motion_until_valid_arrival(self):
        for follow in (False, True):
            with self.subTest(follow=follow):
                self.include_depth_stamp = False
                self.depth, self.center = .19, 350.
                self.wait(lambda: 'depth_source_stamp' not in self.node.packet)
                self.node.controller.start(follow=follow)
                self.wait(lambda: self.node.snapshot['state'] == 'approaching_without_depth')
                self.wait(lambda: self.velocities[-1][0] > 0 and self.velocities[-1][1] < 0)
                self.include_depth_stamp, self.depth_stamp_offset = True, -4.
                self.wait(lambda: 'depth_source_stamp' in self.node.packet and
                          self.node.packet['source_stamp']-self.node.packet['depth_source_stamp'] > 3.)
                self.assertEqual(self.node.snapshot['state'], 'approaching_without_depth')
                self.assertAlmostEqual(self.node.snapshot['output_linear_mps'], .07)
                self.depth_stamp_offset = 0.
                self.wait(lambda: self.node.snapshot['state'] ==
                          ('holding_distance' if follow else 'arrived'))
                self.assertEqual(self.node.snapshot['output_linear_mps'], 0)

    def test_lost_and_stale_detection_stop_actual_velocity_topic(self):
        self.node.controller.start()
        self.wait(lambda: self.node.snapshot.get('output_linear_mps', 0) > 0)
        self.present = False
        self.wait(lambda: self.node.snapshot['state'] == 'target_lost')
        self.wait(lambda: self.velocities[-1] == (0., 0.))
        self.present, self.stamp_offset = True, -4.
        self.wait(lambda: self.node.snapshot['state'] == 'stale_detection')
        self.assertEqual(self.node.snapshot['output_linear_mps'], 0)

    def test_global_stop_and_manual_mode_preempt(self):
        self.node.controller.start()
        self.wait(lambda: self.node.snapshot.get('output_linear_mps', 0) > 0)
        self.stops.publish(Bool(data=True))
        self.wait(lambda: self.node.snapshot['state'] == 'stopped')
        self.assertFalse(self.node.controller.active)
        self.assertEqual(self.node.snapshot['output_linear_mps'], 0)
        self.node.controller.start()
        self.wait(lambda: self.node.snapshot['output_linear_mps'] > 0)
        self.mode.publish(Int8(data=3))
        self.wait(lambda: self.node.snapshot['state'] == 'mode_changed')
        self.assertEqual(self.node.snapshot['output_linear_mps'], 0)

    def test_preview_publishes_no_motion_or_mode_frames(self):
        self.node.execute = False
        self.velocities.clear()
        self.modes.clear()
        self.node.controller.start()
        self.wait(lambda: self.node.snapshot.get('linear_mps', 0) > 0)
        # Drain pending startup zero commands before observing preview.
        self.wait(lambda: self.node.snapshot.get('detection_age_ms') is not None)
        self.velocities.clear()
        self.modes.clear()
        until = time.monotonic()+.4
        while time.monotonic() < until:
            self.executor.spin_once(timeout_sec=.02)
        self.assertEqual(self.velocities, [])
        self.assertEqual(self.modes, [])
        self.assertEqual(self.node.snapshot['output_linear_mps'], 0)

    def test_web_commands_status_and_invalid_distance(self):
        server = ThreadingHTTPServer(('127.0.0.1', 0), partial(WebHandler, node=self.node))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        try:
            with urlopen(base) as response:
                self.assertIn('纯视觉'.encode(), response.read())
            with urlopen(Request(base+'/follow', data=b'{"target_distance_m":0.3}',
                                 headers={'Content-Type': 'application/json'})) as response:
                self.assertEqual(response.status, 202)
            self.wait(lambda: self.node.snapshot.get('follow') is True)
            with urlopen(base+'/status') as response:
                self.assertEqual(json.load(response)['target_distance_m'], .3)
            with self.assertRaises(HTTPError) as rejected:
                urlopen(Request(base+'/start', data=b'{"target_distance_m":0.005}'))
            self.assertEqual(rejected.exception.code, 400)
            urlopen(Request(base+'/stop', data=b'{}')).close()
            self.wait(lambda: self.node.snapshot['state'] == 'stopped')
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.)


if __name__ == '__main__':
    unittest.main()
