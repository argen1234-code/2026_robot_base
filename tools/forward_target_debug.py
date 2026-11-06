#!/usr/bin/env python3
"""Passive forward-target debug node (LiDAR based, no depth dependency).

When a person is visible in the color detector, publish a virtual target on
the robot's forward ray, 0.5 m before the nearest LiDAR obstacle in the
forward corridor.

Outputs (all in map):
  /person_debug/obstacle       PoseStamped  nearest forward obstacle point
  /person_debug/forward_target PoseStamped  target = obstacle_x - 0.5 m
  /person_debug/markers        MarkerArray  red obstacle / green target
  /person_debug/status         String JSON

No cmd_vel, no goal_pose, no Nav2 actions, no depth dependency.
"""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def select_centered(persons, image_size, max_offset_fraction=0.30):
    """Return (person, offset_px) if a person is in the forward centre band."""
    if not persons:
        return None, None
    width = float(image_size[0])
    limit = max_offset_fraction * width
    ranked = []
    for person in persons:
        try:
            x, y, w, h = [float(v) for v in person['bbox']]
        except (KeyError, TypeError, ValueError):
            continue
        if not all(math.isfinite(v) for v in (x, y, w, h)) or w <= 0:
            continue
        offset = abs(x + w / 2.0 - width / 2.0)
        if offset <= limit:
            ranked.append((offset, person))
    if not ranked:
        return None, None
    ranked.sort(key=lambda item: item[0])
    return ranked[0][1], ranked[0][0]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rotate(q, p):
    x, y, z, w = q
    vx, vy, vz = p
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


class ForwardTargetDebug(Node):
    def __init__(self, args):
        super().__init__('forward_target_debug')
        self.args = args
        self.map_frame = 'map'
        self.base_frame = 'base_footprint'
        self.tf = Buffer(cache_time=Duration(seconds=5.0))
        self.listener = TransformListener(self.tf, self)

        self.last_person = 0.0
        self.person_count = 0
        self.person_ahead = False
        self.person_offset_px = None
        self.scan = None
        self.scan_at = 0.0
        self.last_publish = 0.0
        # A published virtual target is a one-shot latch: no further updates.
        self.latched = None

        self.create_subscription(
            String, '/perception/person_detections', self.on_detection,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            LaserScan, '/scan_filtered', self.on_scan, qos_profile_sensor_data)
        self.obstacle_pub = self.create_publisher(
            PoseStamped, '/person_debug/obstacle', 5)
        self.target_pub = self.create_publisher(
            PoseStamped, '/person_debug/forward_target', 5)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/person_debug/markers', 5)
        self.status_pub = self.create_publisher(
            String, '/person_debug/status', 5)
        self.create_timer(0.5, self.tick)
        self.get_logger().info(
            'forward target debug ready '
            f'(lidar, standoff={args.standoff_m:.2f}m, '
            f'window=+/-{args.window_deg:.0f} deg)')

    def on_detection(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        count = int(data.get('count', 0))
        stamp = float(data.get('source_stamp', 0.0))
        if not math.isfinite(stamp) or time.time() - stamp > self.args.max_age:
            self.person_ahead = False
            self.last_person = 0.0
            return
        self.person_count = count
        person, offset = select_centered(
            data.get('persons', []), data.get('image_size', [640, 480]),
            self.args.center_fraction)
        if person is None:
            self.person_ahead = False
            self.person_offset_px = None
            self.last_person = 0.0
            return
        self.person_ahead = True
        self.person_offset_px = round(offset, 1)
        self.last_person = time.monotonic()

    def on_scan(self, msg):
        self.scan = msg
        self.scan_at = time.monotonic()
        self.publish_target()

    def tick(self):
        if self.latched is not None:
            self.publish_latched()
            return
        if self.scan is None:
            self.clear('no_scan', 'waiting for /scan_filtered')
        elif time.monotonic() - self.scan_at > 1.0:
            self.clear('no_scan', 'scan stale')
        elif (not self.person_ahead or not self.last_person or
              time.monotonic() - self.last_person > self.args.max_age):
            self.clear('no_person_ahead', 'no fresh person in the forward centre')
        else:
            self.publish_target()

    def lookup(self, target, source):
        return self.tf.lookup_transform(
            target, source, Time(), timeout=Duration(seconds=0.2))

    def clear(self, state, detail='', **fields):
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))
        self.emit_status(state, detail, **fields)

    def emit_status(self, state, detail='', **fields):
        payload = {
            'state': state, 'detail': detail,
            'person_count': self.person_count,
            'person_ahead': self.person_ahead,
            'person_offset_px': self.person_offset_px,
            'center_fraction': self.args.center_fraction,
            'standoff_m': self.args.standoff_m,
            'window_deg': self.args.window_deg,
        }
        payload.update(fields)
        self.status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

    def publish_target(self):
        if self.latched is not None:
            self.publish_latched()
            return
        if self.scan is None:
            return
        if (not self.person_ahead or not self.last_person or
                time.monotonic() - self.last_person > self.args.max_age):
            return
        if time.monotonic() - self.last_scan_time() > 1.0:
            return
        # Throttle marker/pose updates; scan is 10 Hz.
        now = time.monotonic()
        if now - self.last_publish < 0.12:
            return
        scan = self.scan
        try:
            base_from_scan = self.lookup(self.base_frame, scan.header.frame_id)
            base_to_map = self.lookup(self.map_frame, self.base_frame)
        except Exception as exc:  # noqa: BLE001
            self.clear('tf_failed', str(exc))
            return
        t, q = base_from_scan.transform.translation, base_from_scan.transform.rotation
        quat = (q.x, q.y, q.z, q.w)
        window = math.radians(self.args.window_deg)
        best = None
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < self.args.min_range or r > self.args.max_range:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            p_laser = (r * math.cos(angle), r * math.sin(angle), 0.0)
            rx, ry, rz = rotate(quat, p_laser)
            x = t.x + rx
            y = t.y + ry
            if x <= self.args.min_range:
                continue
            if abs(y) > math.tan(window) * x + 0.05:
                continue
            if best is None or x < best[0]:
                best = (x, y, rz)
        if best is None:
            self.clear('no_obstacle', 'no LiDAR point in forward corridor')
            return
        obstacle_x, obstacle_y, _ = best
        target_x = obstacle_x - self.args.standoff_m
        if target_x <= 0.05:
            self.clear('already_close',
                       f'obstacle {obstacle_x:.2f}m <= standoff',
                       obstacle_m=round(obstacle_x, 3))
            return
        bm_t, bm_q = base_to_map.transform.translation, base_to_map.transform.rotation
        yaw = yaw_from_quat(bm_q)
        # obstacle point in map (actual, may be off-axis)
        ox = bm_t.x + math.cos(yaw) * obstacle_x - math.sin(yaw) * obstacle_y
        oy = bm_t.y + math.sin(yaw) * obstacle_x + math.cos(yaw) * obstacle_y
        # target always on the robot's forward ray
        tx = bm_t.x + target_x * math.cos(yaw)
        ty = bm_t.y + target_x * math.sin(yaw)
        self.latched = {
            'ox': ox, 'oy': oy, 'tx': tx, 'ty': ty, 'yaw': yaw,
            'obstacle_x': obstacle_x, 'obstacle_y': obstacle_y,
            'target_x': target_x,
            'robot': [bm_t.x, bm_t.y, math.degrees(yaw)],
        }
        self.publish_latched()

    def publish_latched(self):
        if self.latched is None:
            return
        now = time.monotonic()
        if now - self.last_publish < 0.4:
            return
        d = self.latched
        stamp = self.get_clock().now().to_msg()
        self.publish_pose(self.obstacle_pub, d['ox'], d['oy'], 0.0,
                          d['yaw'], stamp)
        self.publish_pose(self.target_pub, d['tx'], d['ty'], 0.0,
                          d['yaw'], stamp)
        self.publish_markers(d['ox'], d['oy'], d['tx'], d['ty'], d['yaw'])
        self.last_publish = now
        self.emit_status(
            'latched', 'target fixed; no further updates',
            obstacle_m=round(d['obstacle_x'], 3),
            obstacle_base=[round(d['obstacle_x'], 3),
                           round(d['obstacle_y'], 3)],
            obstacle_map=[round(d['ox'], 3), round(d['oy'], 3)],
            target_m=round(d['target_x'], 3),
            target_gap_to_obstacle_m=round(self.args.standoff_m, 3),
            target_map=[round(d['tx'], 3), round(d['ty'], 3)],
            robot=[round(d['robot'][0], 3), round(d['robot'][1], 3),
                   round(d['robot'][2], 1)])

    def last_scan_time(self):
        return self.scan_at if self.scan is not None else 0.0

    def publish_pose(self, pub, x, y, z, yaw, stamp):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = stamp
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        pub.publish(msg)

    def publish_markers(self, ox, oy, tx, ty, yaw):
        now = self.get_clock().now().to_msg()
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for marker_id, (x, y, z, color, scale, kind, text) in enumerate([
                (ox, oy, 0.25, (1.0, 0.2, 0.2), (0.20, 0.20, 0.40),
                 Marker.SPHERE, 'obstacle'),
                (tx, ty, 0.20, (0.1, 1.0, 0.3), (0.30, 0.10, 0.10),
                 Marker.ARROW, 'target 0.5m')]):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = now
            marker.ns = 'person_debug'
            marker.id = marker_id
            marker.type = kind
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            if kind == Marker.ARROW:
                marker.pose.orientation.z = math.sin(yaw / 2.0)
                marker.pose.orientation.w = math.cos(yaw / 2.0)
            marker.scale.x, marker.scale.y, marker.scale.z = scale
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                *color, 0.9)
            marker.lifetime = Duration(seconds=1.0).to_msg()
            array.markers.append(marker)
            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = now
            label.ns = 'person_debug'
            label.id = 10 + marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 0.35
            label.scale.z = 0.18
            label.color.r, label.color.g, label.color.b, label.color.a = (
                *color, 1.0)
            label.text = text
            label.lifetime = Duration(seconds=1.0).to_msg()
            array.markers.append(label)
        self.marker_pub.publish(array)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--standoff-m', type=float, default=0.5,
                        help='target point distance before the obstacle')
    parser.add_argument('--window-deg', type=float, default=20.0,
                        help='forward corridor half-angle')
    parser.add_argument('--center-fraction', type=float, default=0.30,
                        help='person bbox centre must be within this fraction '
                             'of image width from the centre line')
    parser.add_argument('--min-range', type=float, default=0.15)
    parser.add_argument('--max-range', type=float, default=3.0)
    parser.add_argument('--max-age', type=float, default=1.5)
    args = parser.parse_args()
    rclpy.init()
    node = ForwardTargetDebug(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
