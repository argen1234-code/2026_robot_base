"""
Geometry helpers for turning a camera detection into a navigation goal.
/
把摄像头检测结果转换为导航目标所需的几何计算。

This module intentionally contains no ROS imports so that its pure functions
can be unit-tested without a running ROS graph.  ROS-specific TF lookups are
passed in as callables by the caller.
"""

import json
import math
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

Box = Tuple[float, float, float, float]


def _extract_json(raw: str) -> Optional[Dict[str, Any]]:
    """Extract the first JSON object from an LLM response."""
    if not isinstance(raw, str):
        return None

    text = raw.strip()
    if "```json" in text:
        text = text.split("```json", 1)[1].split("```", 1)[0].strip()
    elif "```" in text:
        text = text.split("```", 1)[1].split("```", 1)[0].strip()

    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1 or end <= start:
        return None

    try:
        value = json.loads(text[start:end + 1])
    except (TypeError, ValueError, json.JSONDecodeError):
        return None

    return value if isinstance(value, dict) else None


def parse_single_box(raw: str, image_width: int, image_height: int) -> Optional[Box]:
    """
    Parse a VLM response of the form {"boxes": [[x1, y1, x2, y2]]}.

    Returns None when there is no person, more than one person, or the box is
    not plausible.  Rejecting is deliberate: a movement command must never be
    based on a malformed detection.
    """
    data = _extract_json(raw)
    if not data:
        return None

    boxes = data.get("boxes")
    if not isinstance(boxes, list) or len(boxes) != 1:
        return None

    item = boxes[0]
    if isinstance(item, dict):
        try:
            x1, y1, x2, y2 = (item["x1"], item["y1"], item["x2"], item["y2"])
        except (KeyError, TypeError):
            return None
    elif isinstance(item, (list, tuple)) and len(item) == 4:
        x1, y1, x2, y2 = item
    else:
        return None

    try:
        x1, y1, x2, y2 = map(float, (x1, y1, x2, y2))
    except (TypeError, ValueError):
        return None

    if not all(math.isfinite(v) for v in (x1, y1, x2, y2)):
        return None

    # Clamp to the image before validating.
    x1 = max(0.0, min(float(image_width), x1))
    x2 = max(0.0, min(float(image_width), x2))
    y1 = max(0.0, min(float(image_height), y1))
    y2 = max(0.0, min(float(image_height), y2))

    if not (0.0 <= x1 < x2 <= image_width and 0.0 <= y1 < y2 <= image_height):
        return None

    box_w = x2 - x1
    box_h = y2 - y1

    # Reject tiny or degenerate boxes: VLM hallucinations are usually either
    # enormous or a few pixels.
    if box_w < 0.03 * image_width or box_h < 0.05 * image_height:
        return None
    if box_w > 0.95 * image_width and box_h > 0.95 * image_height:
        return None

    aspect = box_h / max(box_w, 1.0)
    if aspect < 0.6 or aspect > 6.0:
        return None

    return (x1, y1, x2, y2)


def bearing_from_box(box: Box, fx: float, cx: float) -> float:
    """
    Horizontal bearing in the camera optical frame.

    Positive means the person is to the right of the optical axis.
    Bearing is the most reliable quantity from a monocular camera because it
    does not depend on depth.
    """
    u = (box[0] + box[2]) * 0.5
    return math.atan2(u - cx, fx)


def quat_rotate(quat: Sequence[float], vector: Sequence[float]) -> Tuple[float, float, float]:
    """Rotate a 3D vector by a quaternion (x, y, z, w)."""
    qx, qy, qz, qw = quat
    vx, vy, vz = vector

    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    # v' = v + qw * t + cross(q.xyz, t)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def ground_point_from_box(
    box: Box,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    camera_xyz: Sequence[float],
    camera_quat: Sequence[float],
    max_range_m: float,
) -> Optional[Tuple[float, float, float]]:
    """
    Project the bottom-centre of the person box onto the floor plane z = 0.

    This is the monocular fallback when no depth or LiDAR range is available.
    It assumes a flat floor and a calibrated camera pose.  The result is a
    point in the map frame.
    """
    u = (box[0] + box[2]) * 0.5
    v = box[3]  # bottom centre, i.e. approximately the person's feet

    ray_camera = ((u - cx) / fx, (v - cy) / fy, 1.0)
    norm = math.sqrt(sum(c * c for c in ray_camera))
    if norm <= 0.0:
        return None
    ray_camera = tuple(c / norm for c in ray_camera)

    ray_map = quat_rotate(camera_quat, ray_camera)

    # Ray must point downwards to meet the floor.
    if ray_map[2] >= -1e-6:
        return None

    t = -float(camera_xyz[2]) / ray_map[2]
    if t <= 0.0 or t > max_range_m:
        return None

    return (
        float(camera_xyz[0]) + t * ray_map[0],
        float(camera_xyz[1]) + t * ray_map[1],
        0.0,
    )


def _angle_diff(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def lidar_point_for_bearing(
    scan: Any,
    bearing_camera: float,
    tf_lookup: Callable[[str, str, Any], Tuple[Sequence[float], Sequence[float]]],
    camera_frame: str,
    laser_frame: str,
    stamp: Any,
    window_deg: float = 8.0,
    min_range_m: float = 0.20,
    max_range_m: float = 8.0,
    min_points: int = 3,
) -> Optional[Tuple[Tuple[float, float, float], float]]:
    """
    Use the horizontal LiDAR to measure the range at the person's bearing.

    ``tf_lookup(target, source, stamp)`` must return
    ``(translation_xyz, quaternion_xyzw)`` of ``source`` expressed in
    ``target``.

    Returns ``((x, y, z), range)`` in the laser frame, or None.
    """
    if scan is None or not hasattr(scan, "ranges"):
        return None

    # Direction of the bearing in the camera optical frame: x right, z forward.
    direction_camera = (math.sin(bearing_camera), 0.0, math.cos(bearing_camera))

    try:
        _, laser_from_camera_quat = tf_lookup(laser_frame, camera_frame, stamp)
    except Exception:
        return None

    direction_laser = quat_rotate(laser_from_camera_quat, direction_camera)
    if math.hypot(direction_laser[0], direction_laser[1]) < 1e-6:
        return None

    target_angle = math.atan2(direction_laser[1], direction_laser[0])
    window = math.radians(window_deg)

    ranges = list(scan.ranges)
    angle_min = float(scan.angle_min)
    angle_increment = float(scan.angle_increment)

    candidates: List[Tuple[int, float, float]] = []
    for i, r in enumerate(ranges):
        try:
            r = float(r)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(r) or r < min_range_m or r > max_range_m:
            continue
        angle = angle_min + i * angle_increment
        if _angle_diff(angle, target_angle) <= window:
            candidates.append((i, angle, r))

    if len(candidates) < min_points:
        return None

    # Group contiguous scan indices.  A person's legs produce a compact
    # cluster; walls usually produce much wider or discontinuous returns.
    clusters: List[List[Tuple[int, float, float]]] = []
    current: List[Tuple[int, float, float]] = []
    for item in candidates:
        if not current or item[0] - current[-1][0] <= 2:
            current.append(item)
        else:
            clusters.append(current)
            current = [item]
    if current:
        clusters.append(current)

    valid_clusters = [c for c in clusters if len(c) >= min_points]
    if not valid_clusters:
        return None

    def cluster_score(cluster: List[Tuple[int, float, float]]) -> Tuple[float, float]:
        mean_angle = sum(a for _, a, _ in cluster) / len(cluster)
        mean_range = sum(r for _, _, r in cluster) / len(cluster)
        return (_angle_diff(mean_angle, target_angle), mean_range)

    best = min(valid_clusters, key=cluster_score)
    best_range = sorted(r for _, _, r in best)[len(best) // 2]

    return ((best_range * math.cos(target_angle),
             best_range * math.sin(target_angle),
             0.0), best_range)


def make_standoff_goal(
    person_xyz: Sequence[float],
    robot_xyz: Sequence[float],
    standoff_distance_m: float,
) -> Optional[Dict[str, float]]:
    """
    Build a goal that stops ``standoff_distance_m`` short of the person,
    facing the person.  Returns None when already inside the standoff radius.
    """
    if standoff_distance_m <= 0.0:
        return None

    dx = float(person_xyz[0]) - float(robot_xyz[0])
    dy = float(person_xyz[1]) - float(robot_xyz[1])
    distance = math.hypot(dx, dy)

    if not math.isfinite(distance) or distance <= standoff_distance_m:
        return None

    ratio = (distance - standoff_distance_m) / distance
    yaw = math.atan2(dy, dx)

    return {
        "x": float(robot_xyz[0]) + ratio * dx,
        "y": float(robot_xyz[1]) + ratio * dy,
        "yaw": yaw,
        "person_distance": distance,
    }
