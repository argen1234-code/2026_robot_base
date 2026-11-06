#!/usr/bin/env python3
"""Experimental person approach/follow using the existing Nav2 servers.

Default is preview: publishes markers, a goal and a computed path; never
sends NavigateToPose. --execute additionally requires confirmed calibration.
--execute --follow keeps the task armed after arrival: when the same person
moves beyond follow_replan_m, a fresh path is planned and the robot follows.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import queue
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from robot_base_msgs.msg import TaskStatus
from robot_base_msgs.srv import SetTask

from .person_approach import (StableTarget, TargetMemory, associate_person,
                              is_near_person, iou, moving_away,
                              relative_transform, scan_clusters,
                              select_centered_person, standoff_goal, transform)


class PersonApproach(Node):
    def __init__(self, args):
        super().__init__('person_approach')
        self.args = args
        self.cfg = json.loads(Path(args.config).read_text())
        self.execute = args.execute
        self.trial = getattr(args, 'trial', False)
        self.follow = bool(getattr(args, 'follow', False))
        if self.follow and not self.execute:
            raise ValueError('Continuous follow requires --execute')
        if self.execute and self.cfg['calibration_confirmed'] is not True and not self.trial:
            raise ValueError('Motion requires checked camera/LiDAR calibration; use preview first')
        self.tracker = StableTarget(max_gap_s=self.cfg['occlusion_grace_s']+.2)
        self.target = TargetMemory(grace_s=self.cfg['occlusion_grace_s'],
            retention_s=self.cfg['reacquire_timeout_s'],
            resume_samples=self.cfg['reacquire_samples'],
            max_jump_m=self.cfg['reacquire_max_jump_m'])
        self.paused_for_target = False
        self.resume_after_cancel = False
        self.reacquire_expired = False
        self.task_started_at = None
        self.last_invalid_reason = None
        self.target_was_stable = False
        self.scans = deque(maxlen=20)
        self.detection = None
        self.last_processed = None
        self.grid = None
        self.grid_received = 0.
        self.last_seen = 0.
        self.last_strong = 0.
        self.last_person = None
        self.last_box = None
        self.last_good = None
        self.path_goal = None
        self.path_stamp = 0.
        self.path_busy = False
        self.path_epoch = 0
        self.nav_handle = None
        self.nav_pending = False
        self.pending_cancel = False
        self.started_at = 0.
        self.consumed = False
        self.auto_armed = bool(args.auto)
        self.manual_requested = False
        self.stopped = False
        self.follow_wait = False
        self.follow_anchor = None
        self.follow_started_at = None
        self.guard_history = deque(maxlen=100)
        self.epoch = 0
        self.commands = queue.SimpleQueue()
        self.latest_jpeg = b''
        self.snapshot = {'state': 'waiting', 'execute': self.execute,
                         'calibrated': self.cfg['calibration_confirmed']}
        self.web_map = None
        self.web_path = []
        self.http = None
        self.debug_packet = {}
        self.exit_requested = False
        self.task_result = None
        self.trial_reset_pending = False
        self.trial_status = {}
        self.trial_status_at = 0.
        self.centering_observation = None

        latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/perception/person_detections', self.on_detection, latest_qos)
        self.create_subscription(LaserScan, self.cfg['scan_topic'], self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_grid,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed',
                                 self.on_image, latest_qos)
        self.create_subscription(Bool, '/stop_request', self.on_stop, 1)
        self.create_subscription(String, '/person_nav/trial_status', self.on_trial_status, 1)
        # A manual Foxglove goal takes ownership from the automatic task.
        self.create_subscription(PoseStamped, '/goal_pose', self.on_external_goal, 1)
        self.create_subscription(PoseStamped, '/goal_pose_safe', self.on_external_goal, 1)
        self.tf = Buffer(cache_time=Duration(seconds=15))
        self.listener = TransformListener(self.tf, self)
        self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.spin_handle = None
        self.spin_pending = False
        self.search_active = False
        self.search_since = 0.0
        self.search_cooldown_until = 0.0
        self.search_delay = float(self.cfg.get('search_delay_s', 1.0))
        self.search_timeout = float(self.cfg.get('search_timeout_s', 120.0))
        self.status_pub = self.create_publisher(String, '/person_nav/status', 1)
        self.centering_pub = self.create_publisher(String, '/person_nav/centering_target', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/person_nav/markers', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/person_nav/goal', 1)
        self.path_pub = self.create_publisher(NavPath, '/person_nav/path', 1)
        self.create_service(Trigger, '/person_nav/start', self.start_service)
        self.create_service(Trigger, '/person_nav/stop', self.stop_service)
        self.trial_reset_client = self.create_client(Trigger, '/person_nav/reset_trial')
        self.decision_request_id = ''
        self.decision_status_pub = self.create_publisher(
            TaskStatus, '/decision/vision_status', 5)
        self.create_service(SetTask, '/decision/vision_task', self.on_decision_task)
        self.create_timer(.1, self.tick)
        self.create_timer(.5, self.publish_status)
        self.create_timer(.1, self.publish_centering)
        if args.http_port:
            self.start_http()
        self.get_logger().info(f'person approach ready: execute={self.execute} auto={args.auto} '
                               f'calibrated={self.cfg["calibration_confirmed"]}')

    def on_detection(self, msg):
        try:
            data = json.loads(msg.data)
            if not math.isfinite(float(data['source_stamp'])):
                return
            self.detection = data
        except (ValueError, TypeError, KeyError):
            return

    def on_scan(self, msg):
        self.scans.append(msg)

    def on_trial_status(self, msg):
        try:
            self.trial_status = json.loads(msg.data)
            self.trial_status_at = time.monotonic()
        except ValueError:
            pass

    def on_image(self, msg):
        self.latest_jpeg = bytes(msg.data)

    def on_grid(self, msg):
        if msg.header.frame_id != self.cfg['map_frame'] or msg.info.resolution <= 0:
            return
        q = msg.info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.grid = {'x': msg.info.origin.position.x, 'y': msg.info.origin.position.y,
                     'yaw': yaw, 'resolution': msg.info.resolution,
                     'width': msg.info.width, 'height': msg.info.height, 'data': msg.data}
        self.grid_received = time.monotonic()
        # Serialize once per costmap update, not once per browser request.
        self.web_map = dict(self.grid, data=list(msg.data))

    def lookup(self, target, source, stamp):
        tf = self.tf.lookup_transform(target, source, stamp)
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

    def start_service(self, request, response):
        if not self.execute:
            response.success = False
            response.message = 'Preview only: no motor navigation is enabled'
        elif self.nav_pending or self.nav_handle is not None:
            response.success = False
            response.message = 'An approach task is already active'
        else:
            self.commands.put(('start', self.follow))
            response.success = True
            response.message = ('Will follow once a fresh stable target and path are available'
                                if self.follow else
                                'Will approach once a fresh stable target and path are available')
        return response

    def stop_service(self, request, response):
        self.cancel('stopped')
        response.success, response.message = True, 'Person approach stopped'
        return response

    def on_decision_task(self, request, response):
        """Top-level coordinator entry for the vision decision layer."""
        task = (request.task or '').strip().lower()
        if task in ('stop', 'none', 'cancel'):
            self.commands.put('stop')
            response.success, response.message = True, 'stopping'
            return response
        if task in ('follow', 'start', 'approach_user', 'preview'):
            if request.preview or task == 'preview':
                response.success = True
                response.message = 'preview requested; no motion'
                return response
            if not self.execute:
                response.success = False
                response.message = 'vision decision is preview-only; no motion'
                return response
            if task in ('follow', 'approach_user'):
                # Voice 'approach_user' is a tracking task: keep re-planning
                # while the person moves and stay armed at the standoff point.
                self.follow = True
                self.commands.put(('start', True))
                response.success, response.message = True, 'follow queued'
            else:
                # Legacy one-shot start: track during the leg, then finish at
                # arrival without waiting for the person to move again.
                self.follow = False
                self.commands.put(('start', False))
                response.success, response.message = True, 'approach queued'
            return response
        response.success = False
        response.message = f'unknown vision task {task!r}'
        return response

    def on_stop(self, msg):
        if msg.data:
            self.cancel('stopped')

    def on_external_goal(self, msg):
        # Do not cancel_all_goals: only cancel the goal owned by this node.
        self.cancel('manual_navigation_override')

    def state(self, name, **fields):
        if self.stopped:
            name = self.task_result or 'stopped'
        elif self.paused_for_target:
            name = 'paused_waiting_for_target'
        elif self.execute and self.consumed and not (self.nav_pending or self.nav_handle is not None):
            name = self.task_result or name
        self.snapshot = dict(self.snapshot, state=name, **fields)

    def clear_preview(self):
        self.centering_observation = None
        self.path_epoch += 1
        self.last_good = None
        self.path_goal = None
        self.web_path = []
        path = NavPath()
        path.header.frame_id = self.cfg['map_frame']
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path)
        marker = Marker(action=Marker.DELETEALL)
        self.marker_pub.publish(MarkerArray(markers=[marker]))
        self.snapshot = {k: v for k, v in self.snapshot.items()
                         if k not in ('person', 'goal', 'cluster', 'distance_m')}
        self.snapshot.update(stable=False, target_held=False)

    def cancel(self, reason):
        self.epoch += 1
        self.stopped = True
        self.manual_requested = False
        self.auto_armed = False
        self.pending_cancel = True
        self.paused_for_target = False
        self.resume_after_cancel = False
        self.follow_wait = False
        self.follow_anchor = None
        self.follow_started_at = None
        self.search_active = False
        self.stop_search_spin()
        self.guard_history.clear()
        self.task_result = reason
        if self.nav_handle is not None:
            self.nav_handle.cancel_goal_async()
        self.state(reason)

    def pause_for_target(self):
        """Cancel the owned action while keeping the task armed to reacquire."""
        if self.paused_for_target:
            return
        self.epoch += 1
        self.pending_cancel = True
        self.paused_for_target = True
        self.resume_after_cancel = True
        self.consumed = False
        self.tracker.clear()
        self.guard_history.clear()
        self.path_epoch += 1
        self.path_goal = None
        self.state('paused_waiting_for_target')
        if self.nav_handle is not None:
            self.nav_handle.cancel_goal_async()

    def hold_at_standoff(self, person):
        """Follow mode: end this leg and wait for the same person to move on."""
        self.epoch += 1
        self.pending_cancel = True
        self.paused_for_target = False
        self.resume_after_cancel = False
        self.consumed = False
        self.follow_wait = True
        self.follow_anchor = tuple(person)
        self.guard_history.clear()
        self.task_result = 'arrived'
        self.task_started_at = None
        self.state('arrived')
        if self.nav_handle is not None:
            self.nav_handle.cancel_goal_async()

    def start_search_spin(self):
        """Rotate slowly in place until the camera/LiDAR acquires a person."""
        if not self.spin_client.server_is_ready():
            self.state('waiting_for_spin')
            return
        self.spin_pending = True
        goal = Spin.Goal()
        goal.target_yaw = float(self.cfg.get('search_spin_target_yaw_rad',
                                             2.0 * math.pi))
        allowance = float(self.cfg.get('search_spin_time_allowance_s', 30.0))
        goal.time_allowance.sec = int(allowance)
        goal.time_allowance.nanosec = int((allowance - int(allowance)) * 1e9)
        self.state('searching_person')
        self.spin_client.send_goal_async(goal).add_done_callback(
            self.on_spin_accepted)

    def on_spin_accepted(self, future):
        self.spin_pending = False
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.snapshot['detail'] = str(exc)
            self.search_cooldown_until = time.monotonic() + 1.0
            return
        if not handle.accepted:
            self.search_cooldown_until = time.monotonic() + 1.0
            return
        self.spin_handle = handle
        if not self.search_active or self.stopped:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self.on_spin_result)

    def on_spin_result(self, future):
        self.spin_handle = None
        try:
            future.result()
        except Exception:  # noqa: BLE001
            pass
        if self.search_active and not self.stopped:
            self.search_cooldown_until = time.monotonic() + 0.5

    def stop_search_spin(self):
        self.spin_pending = False
        if self.spin_handle is not None:
            self.spin_handle.cancel_goal_async()
            self.spin_handle = None

    def hold_target(self, reason):
        age = self.target.age(time.monotonic())
        if self.target.point is None or age > self.cfg['reacquire_timeout_s']:
            return False
        self.snapshot.update(person=self.target.point, target_held=True,
                             target_age_ms=round(age*1000), stable=False,
                             missing_reason=reason)
        goal = self.last_good[1] if self.last_good else None
        self.publish_markers(self.target.point, goal, held=True)
        self.state('target_temporarily_lost' if age <= self.cfg['occlusion_grace_s']
                   else 'reacquiring_target')
        return True

    def handle_near_person(self, box, active, now):
        """Person fills the frame: presence is good, range is not.

        At the standoff distance the feet leave the image, so the RGB/LiDAR
        association can fail exactly when the approach succeeded.  Keep the
        continuity timer alive and finish/hold the current leg instead of
        treating this as target loss.
        """
        self.target.mark_present(now)
        self.last_seen = now
        self.last_box = box
        self.last_invalid_reason = None
        width = self.snapshot['image_size'][0]
        target_px = box[0] + box[2]/2
        self.snapshot.update(target_px=round(target_px),
                             target_offset_px=round(target_px-width/2),
                             target_held=False,
                             missing_reason='near_person_distance_unreliable',
                             near_person=True)
        anchor = self.target.point or self.last_person
        # Also covers the case where the person walked into the near zone
        # while the previous leg was canceled awaiting reacquisition, or was
        # already at the standoff when the task started.
        task_active = (active or self.paused_for_target or
                       (not self.consumed and not self.follow_wait and
                        (self.manual_requested or self.auto_armed)))
        if task_active and not self.stopped and anchor is not None:
            self.paused_for_target = False
            self.search_active = False
            self.stop_search_spin()
            if self.follow:
                self.hold_at_standoff(anchor)
            else:
                # The one-shot task is complete once the person fills the
                # frame; do not keep driving toward an unmeasurable target.
                self.epoch += 1
                self.consumed = True
                self.task_result = 'arrived'
                self.state('arrived')
                if self.nav_handle is not None:
                    self.nav_handle.cancel_goal_async()
            self.publish_markers(anchor, None)
            return True
        if self.follow_wait:
            self.publish_markers(anchor, None)
            self.state('arrived_waiting_for_target_move', near_person=True)
            return True
        self.publish_markers(anchor, None)
        return False

    def invalid(self, reason):
        self.last_invalid_reason = reason
        recoverable = reason in ('no_person', 'no_person_lidar_cluster',
            'ambiguous_lidar_clusters', 'image_scan_not_synchronized',
            'stale_detection', 'confirming_returned_target',
            'target_position_mismatch', 'weak_or_duplicate')
        if recoverable and self.hold_target(reason):
            return
        self.tracker.clear()
        self.clear_preview()
        if self.nav_handle is None and not self.nav_pending:
            self.state(reason)

    def tick(self):
        try:
            self._tick()
        except Exception as exc:
            self.invalid('localization_error')
            self.snapshot['detail'] = str(exc)
            self.get_logger().warn(str(exc), throttle_duration_sec=5.)

    def _tick(self):
        now = time.monotonic()
        while not self.commands.empty():
            command = self.commands.get()
            if isinstance(command, tuple):
                command, follow = command
            else:
                follow = self.follow
            if command == 'stop':
                self.cancel('stopped')
            elif command == 'start' and self.trial:
                if self.nav_pending or self.nav_handle is not None or self.trial_reset_pending:
                    continue
                if not self.trial_reset_client.service_is_ready():
                    self.state('waiting_for_trial_chassis')
                    continue
                self.trial_reset_pending = True
                reset_epoch = self.epoch
                def reset_finished(future, epoch=reset_epoch, follow=follow):
                    self.trial_reset_pending = False
                    try:
                        response = future.result()
                        if response.success and epoch == self.epoch:
                            self.commands.put(('start_after_reset', follow))
                        else:
                            self.snapshot['detail'] = response.message
                    except Exception as exc:
                        self.snapshot['detail'] = str(exc)
                self.trial_reset_client.call_async(Trigger.Request()).add_done_callback(reset_finished)
            elif command in ('start', 'start_after_reset') and self.execute and not self.nav_pending and self.nav_handle is None:
                self.stopped = False
                self.pending_cancel = False
                self.consumed = False
                self.manual_requested = True
                self.follow = follow
                self.follow_wait = False
                self.follow_anchor = None
                self.follow_started_at = time.monotonic() if follow else None
                self.search_active = True
                self.search_since = time.monotonic()
                self.search_cooldown_until = 0.0
                self.stop_search_spin()
                self.guard_history.clear()
                self.task_result = None
                self.task_started_at = None
                self.paused_for_target = False
                self.resume_after_cancel = False
                self.reacquire_expired = False
                self.target.clear()
                self.target_was_stable = False
                self.clear_preview()
                self.tracker.clear()
                self.last_processed = None
                self.state('acquiring')
        active = self.nav_pending or self.nav_handle is not None
        if (self.search_active and not self.stopped and
                self.nav_handle is None and not self.nav_pending and
                self.spin_handle is None and not self.spin_pending and
                now >= self.search_cooldown_until):
            if now - self.search_since > self.search_timeout:
                self.cancel('search_timeout')
            elif (now - self.search_since >= self.search_delay and
                  (self.target.point is None or
                   self.target.age(now) > self.cfg['occlusion_grace_s'])):
                self.start_search_spin()
        if active and not self.pending_cancel:
            if now-self.grid_received > 3.:
                self.cancel('costmap_stale_cancelled')
            elif self.target.age(now) > self.cfg['occlusion_grace_s']:
                self.pause_for_target()
        if (self.task_started_at is not None and not self.stopped and
                (active or self.paused_for_target) and
                now-self.task_started_at > self.cfg['max_navigation_s']):
            self.cancel('navigation_timeout')
        follow_limit = self.cfg.get('follow_max_s', 0.)
        if (follow_limit and self.follow_started_at is not None and not self.stopped and
                now-self.follow_started_at > follow_limit):
            self.cancel('follow_timeout')
        if (self.target.point is not None and
                self.target.present_age(now) > self.cfg['reacquire_timeout_s']):
            if self.paused_for_target or self.follow_wait or (active and not self.stopped):
                self.cancel('target_reacquire_timeout')
                self.reacquire_expired = True
            self.target.clear()
            self.target_was_stable = False
            self.tracker.clear()
            self.clear_preview()
        if self.detection is None:
            self.invalid('waiting_for_detection')
            return
        data = self.detection
        stamp = float(data['source_stamp'])
        age = self.get_clock().now().nanoseconds/1e9-stamp
        if not -.1 <= age <= self.cfg['max_result_age_s']:
            self.invalid('stale_detection')
            return
        if stamp == self.last_processed:
            return
        self.last_processed = stamp
        persons = data.get('persons', [])
        self.snapshot.update(count=len(persons), persons=persons,
                             image_size=data.get('image_size', [640, 480]),
                             detection_age_ms=round(age*1000), source_stamp=stamp)
        prior_active = (self.target.box is not None and
                        self.target.present_age(now) <=
                        self.cfg['reacquire_timeout_s'])
        if prior_active:
            matches = sorted([p for p in persons if iou(p['bbox'], self.target.box) >= .15],
                key=lambda p: iou(p['bbox'], self.target.box), reverse=True)
            if len(matches) == 1:
                persons = matches
            elif len(matches) > 1:
                if iou(matches[0]['bbox'], self.target.box)-iou(matches[1]['bbox'], self.target.box) >= .15:
                    persons = matches[:1]
                else:
                    self.invalid('ambiguous_lidar_clusters')
                    return
        if len(persons) != 1:
            if prior_active:
                # Never silently switch identity while the remembered person
                # is still within the reacquire window.
                self.invalid('multiple_people_select_one' if persons else 'no_person')
                return
            # Margin is configured for a 640-wide image; scale with the
            # detector's actual color size.
            margin = (self.cfg.get('multi_person_center_margin_px', 32.) *
                      self.snapshot['image_size'][0]/640.)
            p, selection_reason = select_centered_person(
                persons, self.snapshot['image_size'], margin)
            if p is None:
                self.invalid(selection_reason)
                return
            persons = [p]
        p = persons[0]
        if not self.scans:
            self.invalid('waiting_for_scan')
            return
        def scan_time(s):
            return s.header.stamp.sec+s.header.stamp.nanosec/1e9
        scan = min(self.scans, key=lambda s: abs(scan_time(s)-stamp))
        skew = abs(scan_time(scan)-stamp)
        if skew > self.cfg['max_scan_skew_s']:
            self.invalid('image_scan_not_synchronized')
            return
        if self.grid is None or now-self.grid_received > 3.:
            self.invalid('waiting_for_costmap')
            return
        source_time = Time(nanoseconds=round(stamp*1e9))
        scan_stamp = Time.from_msg(scan.header.stamp)
        try:
            base_to_map = self.lookup(self.cfg['map_frame'], self.cfg['base_frame'], source_time)
            robot, _ = self.lookup(self.cfg['map_frame'],
                                   self.cfg.get('navigation_base_frame', self.cfg['base_frame']), Time())
        except Exception:
            self.invalid('waiting_for_tf')
            return
        # Account for base motion between the scan and color timestamps.
        try:
            scan_to_map = self.lookup(self.cfg['map_frame'], scan.header.frame_id, scan_stamp)
        except Exception:
            self.invalid('waiting_for_scan_tf')
            return
        laser_to_base = relative_transform(base_to_map, scan_to_map)
        clusters = scan_clusters(scan.ranges, scan.angle_min, scan.angle_increment, self.cfg)
        diagnostics = []
        prior_base = None
        if self.target.point is not None and self.target.age(now) < self.cfg['reacquire_timeout_s']:
            prior_base = relative_transform(base_to_map, (self.target.point, (0., 0., 0., 1.)))[0]
        match, reason = associate_person(p['bbox'], self.snapshot['image_size'], clusters,
                                         laser_to_base, self.cfg, diagnostics, prior_base=prior_base)
        self.debug_packet = dict(detection=data, scan=dict(ranges=list(scan.ranges),
            angle_min=scan.angle_min, angle_increment=scan.angle_increment),
            laser_to_base=laser_to_base, base_to_map=base_to_map,
            candidates=diagnostics, reason=reason)
        self.snapshot.update(scan_skew_ms=round(skew*1000), clusters=len(clusters))
        if match is None:
            if is_near_person(p['bbox'], self.snapshot['image_size'],
                              self.cfg.get('near_person_bottom_fraction', .98),
                              self.cfg.get('near_person_width_fraction', .42)):
                if self.handle_near_person(p['bbox'], active, now):
                    return
                self.invalid('near_person_distance_unreliable')
                return
            self.invalid(reason)
            return
        person = transform(*base_to_map, match['base_point'])
        distance = math.hypot(person[0]-robot[0], person[1]-robot[1])
        # Fail-safe closure check: if the robot is translating but the person
        # keeps getting farther away, the goal/frame is wrong (bad initial
        # pose, mirrored/rotated camera association, or opposite motion).
        # Stop rather than drive away and lose the person.
        if active and not self.pending_cancel:
            self.guard_history.append((now, distance, robot[0], robot[1]))
            if moving_away(self.guard_history, now,
                           self.cfg.get('moving_away_window_s', 2.5),
                           self.cfg.get('moving_away_m', .45),
                           self.cfg.get('moving_away_robot_m', .15)):
                self.cancel('moving_away_from_target')
                return
        elif not active:
            self.guard_history.clear()
        stop_radius = self.cfg['standoff_m']+self.cfg.get('robot_front_m', 0.)
        if active and distance <= stop_radius+self.cfg.get('arrival_tolerance_m', .05):
            if self.follow:
                self.hold_at_standoff(person)
            else:
                self.cancel('already_close')
            return
        if not self.cfg['min_range_m'] <= distance <= self.cfg['max_range_m']:
            self.invalid('person_out_of_range')
            return
        strong = float(p.get('conf', 0)) >= .35 and not p.get('confidence_supported', False)
        measured_person = person
        person, reason = self.target.observe(now, stamp, p['bbox'], measured_person, strong=strong)
        if person is None:
            self.invalid(reason)
            return
        if self.search_active:
            self.search_active = False
            self.stop_search_spin()
        stable = self.tracker.add(stamp, p['bbox'], person, strong=True)
        if self.target.reacquired and self.target_was_stable:
            stable = person
        if stable is not None:
            self.target_was_stable = True
        image_width = self.snapshot['image_size'][0]
        self.centering_observation = dict(
            source_stamp=stamp, center_x=p['bbox'][0]+p['bbox'][2]/2,
            width=image_width, fx=self.cfg['fx']*image_width/self.cfg['image_width'])
        if (active and self.last_person is not None and
                math.dist(person, self.last_person) > self.cfg['target_move_cancel_m']):
            self.pause_for_target()
        self.last_box = p['bbox']
        self.last_seen = now
        self.last_invalid_reason = None
        target_px = p['bbox'][0]+p['bbox'][2]/2
        self.snapshot.update(robot=robot, person=person, distance_m=round(distance, 2),
                             cluster=match, stable=stable is not None,
                             target_held=False, target_age_ms=0, missing_reason=None,
                             target_px=round(target_px),
                             target_offset_px=round(target_px-image_width/2),
                             near_person=False)
        if active:
            self.publish_markers(person, self.path_goal)
            if not self.paused_for_target and not self.stopped:
                self.state('navigating')
            return
        if self.paused_for_target:
            # Wait for cancellation RESULT before creating another Nav2 goal.
            self.paused_for_target = False
            self.pending_cancel = False
            if self.target_was_stable:
                stable = person
        if self.follow_wait:
            # Follow mode reached the standoff point.  Keep the same target
            # armed and only plan again after it moves far enough.
            moved = (math.dist(person[:2], self.follow_anchor[:2])
                     if self.follow_anchor is not None else float('inf'))
            if moved < self.cfg.get('follow_replan_m', .6):
                self.publish_markers(person, None)
                self.state('arrived_waiting_for_target_move', follow_moved_m=round(moved, 2))
                return
            self.follow_wait = False
        if stable is None:
            if not self.target_was_stable:
                self.clear_preview()
                self.snapshot.update(person=person, distance_m=round(distance, 2), cluster=match)
            self.state('stabilizing')
            self.publish_markers(person, None)
            return
        goal, reason = standoff_goal(stable, robot, self.cfg['standoff_m'],
                                    self.grid, self.cfg['max_cost'],
                                    front_offset=self.cfg.get('robot_front_m', 0.),
                                    arrival_tolerance=self.cfg.get('arrival_tolerance_m', .05))
        if goal is None:
            if self.follow and reason == 'already_close':
                self.follow_wait = True
                self.follow_anchor = tuple(person)
                self.publish_markers(person, None)
                self.state('arrived_waiting_for_target_move')
                return
            self.invalid(reason)
            self.publish_markers(person, None)
            return
        self.last_good = (stable, goal, now)
        self.get_logger().info(
            'person goal: robot=(%.2f,%.2f) person=(%.2f,%.2f) '
            'goal=(%.2f,%.2f) bbox_center=%.0fpx dist=%.2fm'
            % (robot[0], robot[1], stable[0], stable[1],
               goal['x'], goal['y'], p['bbox'][0]+p['bbox'][2]/2, distance),
            throttle_duration_sec=1.0)
        self.snapshot.update(goal=goal)
        self.goal_pub.publish(self.pose(goal))
        self.publish_markers(stable, goal)
        if self.path_busy:
            return
        if now-self.path_stamp < 1.5 and self.path_goal is not None:
            if math.hypot(goal['x']-self.path_goal['x'], goal['y']-self.path_goal['y']) < .25:
                self.maybe_navigate()
                return
        self.request_path(stable, goal)

    def pose(self, goal):
        pose = PoseStamped()
        pose.header.frame_id = self.cfg['map_frame']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = goal['x'], goal['y']
        pose.pose.orientation.z = math.sin(goal['yaw']/2)
        pose.pose.orientation.w = math.cos(goal['yaw']/2)
        return pose

    def request_path(self, person, goal):
        if not self.path_client.server_is_ready():
            self.state('waiting_for_planner')
            return
        self.path_busy = True
        self.path_stamp = time.monotonic()
        epoch = self.path_epoch
        request = ComputePathToPose.Goal()
        request.goal = self.pose(goal)
        request.planner_id = self.cfg['planner_id']
        request.use_start = False
        self.state('planning')
        def accepted(future):
            try:
                handle = future.result()
                if not handle.accepted:
                    self.path_busy = False
                    self.state('path_rejected')
                    return
                handle.get_result_async().add_done_callback(finished)
            except Exception as exc:
                self.path_busy = False
                self.state('path_error', detail=str(exc))
        def finished(future):
            self.path_busy = False
            try:
                response = future.result()
                if epoch != self.path_epoch:
                    return
                if response.status != GoalStatus.STATUS_SUCCEEDED or not response.result.path.poses:
                    self.path_goal = None
                    self.state('no_path')
                    return
                path = response.result.path
                self.path_goal = goal
                self.path_stamp = time.monotonic()
                self.web_path = [[p.pose.position.x, p.pose.position.y] for p in path.poses]
                self.path_pub.publish(path)
                self.state('ready' if self.execute else 'preview_ready')
                self.maybe_navigate()
            except Exception as exc:
                self.state('path_error', detail=str(exc))
        self.path_client.send_goal_async(request).add_done_callback(accepted)

    def maybe_navigate(self):
        if self.follow_wait:
            return
        if (not self.execute or self.stopped or self.consumed or self.nav_pending or
                self.nav_handle is not None or not (self.auto_armed or self.manual_requested)):
            return
        if self.last_good is None or self.path_goal is None:
            return
        if self.trial and (time.monotonic()-self.trial_status_at > .75 or
                self.trial_status.get('locked', True) or
                not self.trial_status.get('armed') or
                not self.trial_status.get('motors_ready')):
            self.state('waiting_for_trial_chassis')
            return
        person, goal, checked_at = self.last_good
        if time.monotonic()-checked_at > .5 or time.monotonic()-self.path_stamp > 1.5:
            return
        if self.target.age(time.monotonic()) > .25 or self.last_invalid_reason is not None:
            return
        if math.hypot(goal['x']-self.path_goal['x'], goal['y']-self.path_goal['y']) > .25:
            return
        if not self.nav_client.server_is_ready():
            self.state('waiting_for_navigator')
            return
        self.consumed = True
        self.search_active = False
        self.stop_search_spin()
        if self.follow and self.follow_started_at is None:
            # --auto starts without a start command; begin the follow budget
            # on the first actual navigation leg.
            self.follow_started_at = time.monotonic()
        self.task_result = None
        self.nav_pending = True
        self.pending_cancel = False
        self.resume_after_cancel = False
        self.started_at = time.monotonic()
        if self.task_started_at is None:
            self.task_started_at = self.started_at
        self.last_person = person
        epoch = self.epoch
        request = NavigateToPose.Goal()
        request.pose = self.pose(self.path_goal)
        self.state('navigating')
        def accepted(future):
            self.nav_pending = False
            try:
                handle = future.result()
                if not handle.accepted:
                    self.task_result = 'navigation_rejected'
                    self.state('navigation_rejected')
                    return
                self.nav_handle = handle
                if epoch != self.epoch or self.pending_cancel:
                    handle.cancel_goal_async()
                handle.get_result_async().add_done_callback(finished)
            except Exception as exc:
                self.state('navigation_error', detail=str(exc))
        def finished(future):
            self.nav_handle = None
            try:
                status = future.result().status
                if self.resume_after_cancel and not self.stopped:
                    if status != GoalStatus.STATUS_CANCELED:
                        self.cancel('navigation_failed_during_pause')
                    return
                if epoch == self.epoch:
                    if status == GoalStatus.STATUS_SUCCEEDED:
                        self.task_result = 'arrived'
                        anchor = self.target.point or self.last_person
                        if self.follow and anchor is not None:
                            self.hold_at_standoff(anchor)
                        else:
                            self.state('arrived')
                    else:
                        self.task_result = 'navigation_failed'
                        self.state('navigation_failed')
            except Exception as exc:
                self.state('navigation_error', detail=str(exc))
        self.nav_client.send_goal_async(request).add_done_callback(accepted)

    def publish_markers(self, person, goal, held=False):
        array = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        array.markers.append(clear)
        for index, (name, point, color, marker_type) in enumerate([
                ('person', person, (1., .55, .05), Marker.SPHERE),
                ('stop_point', (goal['x'], goal['y'], .05) if goal else None,
                 (.1, 1., .3), Marker.ARROW)]):
            if point is None:
                continue
            m = Marker()
            m.header.frame_id = self.cfg['map_frame']
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = name, index, marker_type, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = point[0], point[1], .15
            yaw = goal['yaw'] if name == 'stop_point' else 0.
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            m.scale.x, m.scale.y, m.scale.z = (.45, .10, .10) if name == 'stop_point' else (.22, .22, .3)
            m.color.r, m.color.g, m.color.b, m.color.a = *color, .35 if held else .9
            m.lifetime = Duration(seconds=1.).to_msg()
            array.markers.append(m)
        self.marker_pub.publish(array)

    def publish_centering(self):
        packet = dict(self.centering_observation or {})
        packet.update(enabled=self.cfg.get('center_person', False),
                      active=bool(self.execute and not self.stopped and
                                  not self.paused_for_target and
                                  (self.nav_pending or self.nav_handle is not None)),
                      task_epoch=self.epoch)
        self.centering_pub.publish(String(data=json.dumps(packet)))

    def publish_status(self):
        self.snapshot.update(execute=self.execute, calibrated=self.cfg['calibration_confirmed'],
                             standoff_m=self.cfg['standoff_m'],
                             standoff_reference=self.cfg.get('standoff_reference', 'robot_base'),
                             robot_front_m=self.cfg.get('robot_front_m', 0.),
                             center_person=self.cfg.get('center_person', False),
                             trial=self.trial,
                             trial_status=self.trial_status,
                             task_consumed=self.consumed, active=self.nav_pending or self.nav_handle is not None,
                             timestamp=time.time(), auto_armed=self.auto_armed,
                             follow=self.follow, follow_wait=self.follow_wait,
                             follow_replan_m=self.cfg.get('follow_replan_m', .6),
                             follow_elapsed_s=(round(time.monotonic()-self.follow_started_at, 1)
                                               if self.follow_started_at is not None else None),
                             paused=self.paused_for_target,
                             target_age_ms=round(self.target.age(time.monotonic())*1000)
                                 if self.target.point is not None else None,
                             target_present_age_ms=round(
                                 self.target.present_age(time.monotonic())*1000)
                                 if self.target.point is not None else None,
                             continuity=dict(grace_s=self.cfg['occlusion_grace_s'],
                                             retention_s=self.cfg['reacquire_timeout_s']),
                             calibration_note='Approximate camera mounting: preview only' if not self.cfg['calibration_confirmed'] else '')
        self.status_pub.publish(String(data=json.dumps(self.snapshot, ensure_ascii=False)))
        decision = TaskStatus()
        decision.stamp = self.get_clock().now().to_msg()
        decision.owner = 'vision'
        decision.task = 'follow' if self.follow else (
            'approach' if self.execute else 'preview')
        decision.state = str(self.snapshot.get('state', 'unknown'))
        decision.detail = str(self.snapshot.get('missing_reason') or
                              self.snapshot.get('detail') or '')
        decision.request_id = self.decision_request_id
        self.decision_status_pub.publish(decision)

    def start_http(self):
        node = self
        html = Path(__file__).with_name('person_approach.html').read_bytes()
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass
            def reply(self, content, content_type, code=200):
                self.send_response(code)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(content)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    self.wfile.write(content)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            def do_GET(self):
                path = urlparse(self.path).path
                if path == '/':
                    self.reply(html, 'text/html; charset=utf-8')
                elif path == '/status':
                    self.reply(json.dumps(dict(node.snapshot, path=node.web_path)).encode(), 'application/json')
                elif path == '/map':
                    self.reply(json.dumps(node.web_map).encode(), 'application/json')
                elif path == '/debug':
                    self.reply(json.dumps(node.debug_packet).encode(), 'application/json')
                elif path == '/image.jpg':
                    self.reply(node.latest_jpeg, 'image/jpeg')
                else:
                    self.reply(b'not found', 'text/plain', 404)
            def do_POST(self):
                # JSON requests and same-origin UI; GET never changes motion.
                origin = self.headers.get('Origin')
                if origin and urlparse(origin).netloc != self.headers.get('Host'):
                    self.reply(b'origin rejected', 'text/plain', 403)
                    return
                if self.headers.get('Content-Type') != 'application/json':
                    self.reply(b'JSON required', 'text/plain', 415)
                    return
                if self.path == '/stop':
                    node.commands.put('stop')
                    self.reply(b'{"accepted":true}', 'application/json')
                elif self.path == '/start' and node.execute:
                    node.commands.put(('start', False))
                    self.reply(b'{"accepted":true,"follow":false}', 'application/json')
                elif self.path == '/follow' and node.execute:
                    node.commands.put(('start', True))
                    self.reply(b'{"accepted":true,"follow":true}', 'application/json')
                else:
                    self.reply(b'{"accepted":false,"reason":"preview_only"}', 'application/json', 409)
        self.http = ThreadingHTTPServer(('0.0.0.0', self.args.http_port), Handler)
        thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        thread.start()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(
        Path(os.environ.get('PERSON_APPROACH_ROOT', '/home/pi/2026_robot_base')) /
        'config/person_approach.json'))
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--trial', action='store_true',
                        help='Bounded hardware trial; requires person_trial_chassis.py gate')
    parser.add_argument('--auto', action='store_true',
                        help='Start automatically when a stable target appears (requires --execute)')
    parser.add_argument('--follow', action='store_true',
                        help='Keep following the same person after arrival (requires --execute)')
    parser.add_argument('--http-port', type=int, default=8878)
    # launch_ros appends --ros-args ...; accept and ignore those here so the
    # node's own argparse does not reject them.
    args, _ = parser.parse_known_args()
    if args.follow and not args.execute:
        parser.error('--follow requires --execute')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PersonApproach(args)
    def stop_signal(signum, frame):
        node.commands.put('stop')
        node.exit_requested = True
    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    try:
        while rclpy.ok() and not node.exit_requested:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cancel('shutdown')
        # Let an owned goal cancellation reach Nav2 before destroying the client.
        deadline = time.monotonic()+1.
        while (node.nav_handle is not None or node.nav_pending) and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if node.http:
            node.http.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
