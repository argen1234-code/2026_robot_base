#!/usr/bin/env python3
"""Short physical trial using the production MCU bridge, with a motion gate.

Run as the ONLY chassis bridge. Motion limits are read from person_approach.json.
Fresh person task state, TF, scan and MCU telemetry
are required. Exposes measurements on /person_nav/trial_status.
"""
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/
    'robot_ros2_ws/src/robot_base_bringup/scripts'))
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from chassis_bridge import ChassisBridge
from chassis_protocol import FLAG_FATAL, SERVICE_DISARM, service_frame
try:
    from robot_base_decision.person_centering import FootprintGuard, centered_command
except ImportError:  # 兼容迁移前的 tools/ 布局
    from person_centering import FootprintGuard, centered_command


class TrialBridge(ChassisBridge):
    def __init__(self):
        super().__init__()
        cfg = json.loads((Path(__file__).resolve().parents[1]/
                          'config/person_approach.json').read_text())
        self.max_travel = cfg.get('trial_max_travel_m', .30)
        self.max_rotation = cfg.get('trial_max_turn_rad', .5)
        self.max_duration = cfg.get('trial_max_duration_s', 10.)
        self.front_x = cfg.get('robot_front_m', .246)
        self.half_width = cfg.get('robot_half_width_m', .118)
        self.front_clearance = cfg.get('trial_front_clearance_m', .20)
        self.stop_cause = None
        self.control_source = 'idle'
        self.centering_cfg = cfg
        self.center_person = cfg.get('center_person', False)
        self.centering_target = {}
        self.centering_at = self.local_grid_at = 0.
        self.local_guard = None
        self.local_frame = None
        self.center_error = None
        self.centering_state = 'waiting_for_target'
        self.last_centering_yaw = 0.
        self.last_centering_at = time.monotonic()
        self.output_command = (0., 0.)
        self.status = {}
        self.status_at = self.scan_at = self.odom_at = 0.
        self.scan = None
        self.last_odom = None
        self.travel = self.rotation = 0.
        self.started = None
        self.locked = False
        self.block_reason = 'waiting_for_target'
        self.fatal = False
        self.tf = Buffer(cache_time=Duration(seconds=5))
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(String, '/person_nav/status', self.on_status, 1)
        # The sanitized Foxglove/voice goal starts ordinary Nav2 operation.
        # Person goals use the action directly and never publish this topic.
        self.create_subscription(PoseStamped, '/goal_pose_safe', self.on_navigation_goal, 10)
        self.create_subscription(Bool, '/stop_request', self.on_stop_request, 1)
        self.create_subscription(String, '/person_nav/centering_target', self.on_centering, 1)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_local_grid,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, '/scan_filtered', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/wheel/odom', self.on_odom, qos_profile_sensor_data)
        self.stop_pub = self.create_publisher(Bool, '/stop_request', 1)
        self.trial_pub = self.create_publisher(String, '/person_nav/trial_status', 1)
        self.create_service(Trigger, '/person_nav/reset_trial', self.reset_trial)
        self.create_timer(.25, self.report)

    def on_remote(self, msg):
        # Remote mode belongs to the normal bridge, not this one-time trial.
        pass

    def reset_trial(self, request, response):
        if self.status.get('active') or time.monotonic()-self.status_at > .75:
            response.success, response.message = False, 'Wait for the old navigation task to stop'
            return response
        if self.fatal:
            response.success, response.message = False, 'MCU fault; trial remains disabled'
            return response
        self.travel = self.rotation = 0.
        self.started = None
        self.last_command = None
        self.locked = False
        self.enabled = True
        self.control_source = 'person'
        self.stop_cause = None
        self.block_reason = 'waiting_for_person_task'
        response.success, response.message = True, 'New short trial ready'
        return response

    def on_navigation_goal(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        if (not msg.header.frame_id or
                not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w < 1e-6 or self.fatal):
            return
        self.control_source = 'navigation'
        self.enabled, self.locked = True, False
        self.started = self.stop_cause = None
        self.travel = self.rotation = 0.
        self.last_command = None
        self.last_centering_yaw = 0.
        self.centering_state = 'off_for_navigation'
        self.block_reason = 'navigation_waiting_for_velocity'

    def on_stop_request(self, msg):
        if msg.data and not self.locked:
            self.latch_stop('stopped')

    def on_telemetry(self, msg):
        super().on_telemetry(msg)
        self.fatal = bool(msg['flags'] & FLAG_FATAL)

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
            self.status_at = time.monotonic()
        except ValueError:
            pass

    def on_scan(self, msg):
        self.scan, self.scan_at = msg, time.monotonic()

    def on_centering(self, msg):
        try:
            self.centering_target = json.loads(msg.data)
            self.centering_at = time.monotonic()
        except (ValueError, TypeError):
            self.centering_target = {}

    def on_local_grid(self, msg):
        if msg.info.resolution <= 0 or not msg.header.frame_id:
            return
        q = msg.info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.local_guard = FootprintGuard(msg.data, msg.info.width, msg.info.height,
            msg.info.resolution, (msg.info.origin.position.x, msg.info.origin.position.y), yaw)
        self.local_frame, self.local_grid_at = msg.header.frame_id, time.monotonic()

    def centered_velocity(self, linear, angular, now):
        target = self.centering_target
        age = time.time()-float(target.get('source_stamp', 0.))
        if (now-self.centering_at > .35 or not target.get('enabled') or
                not target.get('active') or
                not -.1 <= age <= self.centering_cfg.get('centering_max_result_age_s', .5)):
            self.block_reason = self.centering_state = 'centering_waiting_for_target'
            return None
        if self.local_guard is None or now-self.local_grid_at > .45:
            self.block_reason = self.centering_state = 'centering_waiting_for_costmap'
            return None
        try:
            v, w, self.center_error = centered_command(linear, angular, target, self.centering_cfg)
            limit = self.centering_cfg.get('centering_max_angular_accel', .6)*min(.1, max(.001, now-self.last_centering_at))
            w = min(self.last_centering_yaw+limit, max(self.last_centering_yaw-limit, w))
            # Zero from Nav2 remains zero even if a previous centering turn was active.
            if abs(linear) < .001 and abs(angular) < .01:
                v = w = 0.
            tf = self.tf.lookup_transform(self.local_frame, 'base_footprint', Time())
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            if not self.local_guard.clear((t.x, t.y, yaw), v, w, front=self.front_x,
                    half_width=self.half_width,
                    horizon=self.centering_cfg.get('centering_horizon_s', 1.2)):
                self.block_reason = self.centering_state = 'centering_path_blocked'
                return None
            self.last_centering_yaw, self.last_centering_at = w, now
            self.centering_state = 'turning_to_center' if v < .001 and abs(w) > .01 else 'centering'
            return v, w
        except (KeyError, TypeError, ValueError):
            self.block_reason = self.centering_state = 'centering_waiting_for_target'
        except Exception:
            self.block_reason = self.centering_state = 'centering_waiting_for_tf'
        return None

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        now = time.monotonic()
        if self.last_odom is not None and self.started is not None:
            x, y, old_yaw = self.last_odom
            self.travel += math.hypot(p.x-x, p.y-y)
            self.rotation += abs(math.atan2(math.sin(yaw-old_yaw), math.cos(yaw-old_yaw)))
        self.last_odom = (p.x, p.y, yaw)
        self.odom_at = now

    def latch_stop(self, reason):
        if self.locked:
            return
        self.locked, self.enabled, self.block_reason = True, False, reason
        self.stop_pub.publish(Bool(data=True))
        if self.device is not None:
            self.device.write(service_frame(SERVICE_DISARM))
        self.get_logger().warn('Trial stopped: '+reason)

    def _command(self):
        self.output_command = (0., 0.)
        command = self.guarded_command()
        if command == (0., 0.):
            self.last_centering_yaw = 0.
            self.last_centering_at = time.monotonic()
        self.output_command = command
        return command

    def guarded_command(self):
        now = time.monotonic()
        if self.locked:
            return 0., 0.
        if self.fatal:
            self.latch_stop('mcu_fault')
            return 0., 0.
        if self.control_source == 'navigation':
            # Ordinary navigation retains Nav2's obstacle avoidance and the
            # production MCU watchdog. Camera presence and person trial limits
            # must not prevent a user-issued navigation goal from executing.
            if self.mode != 3:
                self.block_reason = 'navigation_wrong_mode'
                return 0., 0.
            if self.last_telemetry is None or now-self.last_telemetry > .35:
                self.block_reason = 'waiting_for_mcu_telemetry'
                return 0., 0.
            v, w = super()._command()
            self.centering_state = 'off_for_navigation'
            self.block_reason = ('navigation_driving' if abs(v) > .001 or abs(w) > .01
                                 else 'navigation_waiting_for_velocity')
            return (min(self.max_linear, max(0., v)),
                    min(self.max_angular, max(-self.max_angular, w)))
        if self.control_source != 'person':
            self.block_reason = 'waiting_for_navigation_or_person'
            return 0., 0.
        if self.started is not None:
            for exceeded, cause in ((self.travel >= self.max_travel, 'distance_limit'),
                                    (self.rotation >= self.max_rotation, 'turn_limit'),
                                    (now-self.started >= self.max_duration, 'time_limit')):
                if exceeded:
                    self.stop_cause = cause
                    self.latch_stop('short_trial_complete')
                    return 0., 0.
        if self.mode != 3 or now-self.status_at > .75:
            self.block_reason = 'waiting_for_person_task'
            return 0., 0.
        s = self.status
        if not (s.get('execute') and s.get('active') and not s.get('paused')
                and s.get('state') in ('navigating', 'target_temporarily_lost')):
            self.block_reason = 'person_task_not_moving'
            return 0., 0.
        if now-self.scan_at > .35 or self.scan is None or now-self.odom_at > .35:
            self.block_reason = 'waiting_for_scan_or_odometry'
            return 0., 0.
        if self.last_telemetry is None or now-self.last_telemetry > .35:
            self.block_reason = 'waiting_for_mcu_telemetry'
            return 0., 0.
        v, w = super()._command()
        if self.center_person:
            centered = self.centered_velocity(v, w, now)
            if centered is None:
                return 0., 0.
            v, w = centered
        try:
            tf = self.tf.lookup_transform('base_footprint', self.scan.header.frame_id, Time())
            q = tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        except Exception:
            self.block_reason = 'waiting_for_lidar_tf'
            return 0., 0.
        forward_samples = 0
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or not max(.20, self.scan.range_min) <= r < self.scan.range_max:
                continue
            a = self.scan.angle_min+i*self.scan.angle_increment+yaw
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) < math.radians(55):
                forward_samples += 1
            # Measure clearance from the actual front of the chassis. A raw
            # 0.60 m laser range prevented reaching a 0.30 m body-to-human gap.
            x = tf.transform.translation.x+r*math.cos(a)
            y = tf.transform.translation.y+r*math.sin(a)
            if (-.05 < x < self.front_x+self.front_clearance and
                    abs(y) < self.half_width+.05 and v > .001):
                self.block_reason = 'obstacle_in_front'
                return 0., 0.
            if abs(w) > .01 and r < .30:
                self.block_reason = 'obstacle_near_turn'
                return 0., 0.
        if forward_samples < 3:
            self.block_reason = 'insufficient_front_scan'
            return 0., 0.
        if abs(v) > .001 or abs(w) > .01:
            if self.started is None:
                self.started = now
                self.travel = self.rotation = 0.
            self.block_reason = 'driving'
        else:
            self.block_reason = 'waiting_for_nav2_velocity'
        return min(.15, max(0., v)), min(.30, max(-.30, w))

    def report(self):
        self.trial_pub.publish(String(data=json.dumps(dict(
            control_source=self.control_source,
            state=self.block_reason, travel_m=round(self.travel, 3),
            rotation_rad=round(self.rotation, 3), armed=self.armed,
            stop_cause=self.stop_cause,
            centering=dict(enabled=self.center_person and self.control_source == 'person',
                           state=self.centering_state,
                           error_px=None if self.center_error is None else round(self.center_error, 1),
                           costmap_age_ms=None if not self.local_grid_at else
                               round((time.monotonic()-self.local_grid_at)*1000),
                           linear_m_s=round(self.output_command[0], 3),
                           angular_rad_s=round(self.output_command[1], 3)),
            limits=dict(travel_m=self.max_travel, turn_rad=self.max_rotation,
                        duration_s=self.max_duration),
            motors_ready=self.motors_ready, locked=self.locked,
            telemetry_count=self.telemetry_count))))


def main():
    rclpy.init()
    node = TrialBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
