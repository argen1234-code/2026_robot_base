"""Exercise the real ROS node against fake Nav2 on isolated domain 79.

No hardware nodes, no motor topics, no publishers in the robot's domain 1.
"""
import argparse
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from robot_base_decision.vision_decision import PersonApproach


class FakeEnvironment(Node):
    def __init__(self, delay_accept=False):
        super().__init__('person_approach_test_environment')
        self.group = ReentrantCallbackGroup()
        self.people = True
        self.persons = None
        self.delay_accept = delay_accept
        self.nav_requests = self.nav_calls = self.path_calls = self.cancel_calls = 0
        self.force_finish = False
        self.target_range = 2.
        self.robot_x = 0.
        self.robot_y = 0.
        self.robot_yaw = 0.
        self.centering_packet = None
        self.create_subscription(String, '/person_nav/centering_target',
            lambda m: setattr(self, 'centering_packet', json.loads(m.data)), 1)
        self.det_pub = self.create_publisher(String, '/perception/person_detections', 1)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_filtered', qos_profile_sensor_data)
        self.stop_pub = self.create_publisher(Bool, '/stop_request', 1)
        self.manual_goal_pub = self.create_publisher(PoseStamped, '/goal_pose_safe', 1)
        self.map_pub = self.create_publisher(OccupancyGrid, '/global_costmap/costmap',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf_pub = StaticTransformBroadcaster(self)
        transforms = []
        for parent, child, z in [('base_footprint', 'base_link', 0.),
                                 ('base_link', 'laser_frame', .1)]:
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = parent, child
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.transform.translation.z = z
            tf.transform.rotation.w = 1.
            transforms.append(tf)
        self.tf_pub.sendTransform(transforms)
        # map->base_footprint is dynamic so tests can move the robot and check
        # that the approach node reacts to real closing/opening distance.
        self.dynamic_tf = TransformBroadcaster(self)
        self.create_timer(.02, self.publish_tf)
        self.plan_server = ActionServer(self, ComputePathToPose, '/compute_path_to_pose',
                                       self.plan, callback_group=self.group)
        self.nav_server = ActionServer(self, NavigateToPose, '/navigate_to_pose', self.navigate,
            goal_callback=self.accept, cancel_callback=self.cancel, callback_group=self.group)
        self.create_timer(.1, self.publish)

    def accept(self, request):
        self.nav_requests += 1
        if self.delay_accept:
            time.sleep(.8)
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        self.cancel_calls += 1
        return CancelResponse.ACCEPT

    def plan(self, handle):
        self.path_calls += 1
        result = ComputePathToPose.Result()
        result.path.header = handle.request.goal.header
        start = PoseStamped()
        start.header = result.path.header
        start.pose.orientation.w = 1.
        result.path.poses = [start, handle.request.goal]
        handle.succeed()
        return result

    def navigate(self, handle):
        self.nav_calls += 1
        deadline = time.monotonic()+15
        while not self.force_finish and time.monotonic() < deadline:
            if handle.is_cancel_requested:
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(.02)
        if self.force_finish:
            handle.succeed()
        else:
            handle.abort()
        return NavigateToPose.Result()

    def publish_tf(self):
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'base_footprint'
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.transform.translation.x = self.robot_x
        tf.transform.translation.y = self.robot_y
        tf.transform.rotation.z = math.sin(self.robot_yaw/2)
        tf.transform.rotation.w = math.cos(self.robot_yaw/2)
        self.dynamic_tf.sendTransform(tf)

    def publish(self):
        # Keep sensor stamps slightly behind the TF so tf2 can interpolate
        # between the 50 Hz dynamic transforms instead of extrapolating.
        stamp = (self.get_clock().now() - Duration(seconds=.1)).to_msg()
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'laser_frame', stamp
        scan.angle_min, scan.angle_increment = -math.pi, .01
        scan.range_min, scan.range_max = .1, 8.
        scan.ranges = [float('inf')]*629
        # Keep the person fixed in the map; the measurement changes as the
        # fake robot moves along x.
        person_range = max(.12, self.target_range-self.robot_x)
        for i in range(309, 320):
            scan.ranges[i] = person_range
        self.scan_pub.publish(scan)
        if self.persons is not None:
            people = self.persons
        else:
            people = [{'bbox': [250., 0., 140., 440.], 'conf': .9}] if self.people else []
        data = {'source_stamp': stamp.sec+stamp.nanosec/1e9, 'image_size': [640, 480],
                'persons': people}
        self.det_pub.publish(String(data=json.dumps(data)))
        grid = OccupancyGrid()
        grid.header.frame_id, grid.header.stamp = 'map', stamp
        grid.info.width = grid.info.height = 100
        grid.info.resolution = .1
        grid.info.origin.position.x = grid.info.origin.position.y = -5.
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*10000
        self.map_pub.publish(grid)


class RosIntegrationTests(unittest.TestCase):
    def start(self, execute=False, delay_accept=False, follow=False, persons=None):
        rclpy.init(domain_id=79, signal_handler_options=SignalHandlerOptions.NO)
        self.temp = tempfile.TemporaryDirectory()
        config_path = Path(__file__).with_name('person_approach.json')
        if not config_path.exists():
            config_path = Path(__file__).resolve().parents[4]/'config/person_approach.json'
        config = json.loads(config_path.read_text())
        config.update(calibration_confirmed=execute, camera_xyz_m=[0., 0., .5],
                      camera_pitch_down_deg=0., occlusion_grace_s=.45,
                      reacquire_timeout_s=3.)
        self.config = Path(self.temp.name)/'config.json'
        self.config.write_text(json.dumps(config))
        self.env = FakeEnvironment(delay_accept=delay_accept)
        if persons is not None:
            self.env.persons = persons
        self.node = PersonApproach(argparse.Namespace(config=str(self.config), execute=execute,
                                                     auto=execute, follow=follow, http_port=0))
        self.executor = MultiThreadedExecutor(num_threads=5)
        self.executor.add_node(self.env)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def until(self, predicate, seconds=7):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.03)
        self.fail(f'timed out: state={self.node.snapshot}, paths={self.env.path_calls}, '
                  f'nav_requests={self.env.nav_requests}, cancels={self.env.cancel_calls}')

    def tearDown(self):
        if not hasattr(self, 'executor'):
            return
        self.env.force_finish = True
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=2)
        self.node.destroy_node()
        self.env.plan_server.destroy()
        self.env.nav_server.destroy()
        self.env.destroy_node()
        rclpy.shutdown()
        self.temp.cleanup()

    def test_preview_computes_path_without_sending_navigation(self):
        self.start()
        self.until(lambda: self.node.snapshot['state'] == 'preview_ready')
        self.assertTrue(self.node.web_path)
        self.assertGreater(self.env.path_calls, 0)
        self.assertEqual(self.env.nav_requests, 0)
        self.assertFalse(self.node.execute)

    def test_multiple_people_without_prior_target_uses_image_center(self):
        persons = [{'bbox': [20., 0., 120., 440.], 'conf': .9},
                   {'bbox': [260., 0., 120., 440.], 'conf': .9},
                   {'bbox': [500., 0., 120., 440.], 'conf': .9}]
        self.start(execute=True, persons=persons)
        self.until(lambda: self.env.nav_calls == 1)
        self.assertEqual(self.node.snapshot.get('target_px'), 320)

    def test_multiple_people_near_center_are_not_swapped_silently(self):
        persons = [{'bbox': [240., 0., 140., 440.], 'conf': .9},
                   {'bbox': [260., 0., 140., 440.], 'conf': .9}]
        self.start(execute=True, persons=persons)
        self.until(lambda: self.node.snapshot['state'] == 'ambiguous_person_choice')
        time.sleep(.4)
        self.assertEqual(self.env.nav_requests, 0)

    def test_follow_waits_at_standoff_then_reapproaches_after_target_moves(self):
        self.start(execute=True, follow=True)
        self.until(lambda: self.env.nav_calls == 1)
        self.env.force_finish = True
        self.until(lambda: self.node.follow_wait)
        self.env.force_finish = False
        self.until(lambda: self.node.snapshot['state'] == 'arrived_waiting_for_target_move')
        self.assertEqual(self.env.nav_requests, 1)
        time.sleep(.6)
        self.assertEqual(self.env.nav_requests, 1)
        # Move gradually so TargetMemory keeps the same identity.
        for distance in (2.2, 2.5, 2.8):
            self.env.target_range = distance
            time.sleep(.35)
        self.until(lambda: self.env.nav_requests == 2)
        self.assertFalse(self.node.follow_wait)

    def test_one_shot_finishes_at_arrival(self):
        self.start(execute=True)
        self.until(lambda: self.env.nav_calls == 1)
        self.env.force_finish = True
        self.until(lambda: self.node.snapshot['state'] == 'arrived')
        self.env.force_finish = False
        self.assertFalse(self.node.follow_wait)
        self.assertTrue(self.node.consumed)
        time.sleep(.5)
        self.assertEqual(self.env.nav_requests, 1)

    def test_moving_away_cancels_when_robot_backs_off_target(self):
        self.start(execute=True)
        self.until(lambda: self.env.nav_calls == 1)
        # The person stays at x=2.0 while the fake robot backs away.
        for step in range(1, 13):
            self.env.robot_x = -0.05*step
            time.sleep(.25)
        self.until(lambda: self.node.stopped)
        self.assertEqual(self.node.snapshot['state'], 'moving_away_from_target')
        self.assertGreaterEqual(self.env.cancel_calls, 1)

    def test_follow_stops_when_target_stays_lost(self):
        self.start(execute=True, follow=True)
        self.until(lambda: self.env.nav_calls == 1)
        self.env.force_finish = True
        self.until(lambda: self.node.follow_wait)
        self.env.force_finish = False
        self.env.people = False
        self.until(lambda: self.node.stopped, seconds=6)
        self.assertEqual(self.node.snapshot['state'], 'target_reacquire_timeout')
        self.assertFalse(self.node.follow_wait)

    def test_brief_loss_keeps_goal_and_returns_without_restart(self):
        self.start(execute=True)
        self.until(lambda: self.env.nav_calls == 1)
        self.env.people = False
        self.until(lambda: self.node.snapshot.get('target_held') is True)
        time.sleep(.15)
        self.assertEqual(self.env.cancel_calls, 0)
        self.env.people = True
        self.until(lambda: self.node.snapshot.get('target_held') is False)
        self.assertEqual(self.env.nav_requests, 1)

    def test_centering_uses_current_selected_box_and_stops_with_task(self):
        self.start(execute=True)
        self.until(lambda: self.env.centering_packet and self.env.centering_packet.get('active'))
        packet = self.env.centering_packet
        self.assertTrue(packet['enabled'])
        self.assertEqual(packet['center_x'], 320.)
        self.assertEqual(packet['width'], 640)
        self.assertLess(time.time()-packet['source_stamp'], .5)
        self.env.stop_pub.publish(Bool(data=True))
        self.until(lambda: self.env.centering_packet.get('active') is False)
        self.assertTrue(self.node.stopped)

    def test_longer_occlusion_pauses_then_resumes_same_target(self):
        self.start(execute=True)
        self.until(lambda: self.env.nav_calls == 1)
        self.env.people = False
        self.until(lambda: self.env.cancel_calls >= 1 and self.node.nav_handle is None)
        self.assertEqual(self.node.snapshot['state'], 'paused_waiting_for_target')
        self.env.people = True
        self.until(lambda: self.env.nav_calls == 2)
        self.assertFalse(self.node.stopped)
        self.assertFalse(self.node.paused_for_target)

    def test_expired_target_does_not_switch_to_another_person(self):
        self.start(execute=True)
        self.until(lambda: self.env.nav_calls == 1)
        self.env.people = False
        self.until(lambda: self.node.snapshot['state'] == 'target_reacquire_timeout')
        self.env.target_range = 3.5
        self.env.people = True
        time.sleep(.8)
        self.assertEqual(self.env.nav_requests, 1)
        self.assertTrue(self.node.stopped)

    def test_explicit_stop_prevents_resume_after_occlusion(self):
        self.start(execute=True)
        self.until(lambda: self.env.nav_calls == 1)
        self.env.people = False
        self.until(lambda: self.node.paused_for_target)
        self.env.stop_pub.publish(Bool(data=True))
        self.until(lambda: self.node.stopped)
        self.env.people = True
        time.sleep(.8)
        self.assertEqual(self.env.nav_requests, 1)
        self.assertEqual(self.node.snapshot['state'], 'stopped')

    def test_sanitized_foxglove_goal_releases_person_navigation(self):
        self.start(execute=True)
        self.until(lambda: self.env.nav_calls == 1)
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x = 1.
        m.pose.orientation.w = 1.
        self.env.manual_goal_pub.publish(m)
        self.until(lambda: self.env.cancel_calls == 1 and self.node.nav_handle is None)
        self.assertEqual(self.node.snapshot['state'], 'manual_navigation_override')
        time.sleep(.4)
        self.assertEqual(self.env.nav_requests, 1)
        self.assertTrue(self.node.stopped)

    def test_stop_before_acceptance_cancels_late_goal(self):
        self.start(execute=True, delay_accept=True)
        self.until(lambda: self.env.nav_requests == 1)
        self.env.stop_pub.publish(Bool(data=True))
        self.until(lambda: self.env.cancel_calls >= 1 and self.node.nav_handle is None)
        self.assertEqual(self.node.snapshot['state'], 'stopped')
        self.assertEqual(self.env.nav_requests, 1)


if __name__ == '__main__':
    unittest.main()
