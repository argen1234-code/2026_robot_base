"""Validate the cloud command schema used by the reference mini-program."""

import json
import math


def parse_command(payload):
    if len(payload) > 16384:
        raise ValueError('oversized MQTT command')
    data = json.loads(payload)
    if not isinstance(data, dict):
        raise ValueError('command must be an object')
    name = data.get('command')
    if not isinstance(name, str):
        raise ValueError('missing command')
    name = name.upper()
    if name in ('FORWARD', 'BACKWARD', 'LEFT', 'RIGHT', 'STOP'):
        speed = float(data.get('speed', 50))
        if not math.isfinite(speed):
            raise ValueError('non-finite speed')
        return name, max(0.0, min(speed, 100.0)) / 100.0
    if name in ('REMOTE', 'INDOOR', 'LINE', 'EMERGENCY', 'RESET_EMERGENCY',
                'INDOOR_MISSION_CANCEL'):
        return name, None
    if name == 'INDOOR_MISSION_START':
        if str(data.get('patrol_mode', 'ONCE')).upper() != 'ONCE':
            raise ValueError('only ONCE missions are supported')
        points = data.get('waypoints')
        if not isinstance(points, list) or not 1 <= len(points) <= 30:
            raise ValueError('waypoints must have 1..30 points')
        poses = []
        for point in points:
            if not isinstance(point, dict):
                raise ValueError('invalid waypoint')
            x, y, yaw = float(point['x']), float(point['y']), float(point.get('yaw', 0.0))
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise ValueError('non-finite waypoint')
            poses.append((x, y, yaw))
        return name, poses
    raise ValueError('unsupported command')
