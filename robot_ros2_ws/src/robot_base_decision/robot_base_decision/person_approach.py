"""ROS-free geometry for the experimental camera/LiDAR person approach.

Association is a foreground-cluster heuristic, not a person range sensor.
Camera mounting/intrinsics must be checked before enabling motion.
"""
import math
from collections import deque


def rotate(q, p):
    x, y, z, w = q
    vx, vy, vz = p
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz,
            vz+w*tz+x*ty-y*tx)


def transform(translation, quaternion, point):
    return tuple(a+b for a, b in zip(translation, rotate(quaternion, point)))


def relative_transform(base_to_map, laser_to_map):
    """laser at scan time -> base at image time, including robot rotation."""
    bt, bq = base_to_map
    lt, lq = laser_to_map
    inv = (-bq[0], -bq[1], -bq[2], bq[3])
    x, y, z, w = inv
    a, b, c, d = lq
    q = (w*a+x*d+y*c-z*b, w*b-x*c+y*d+z*a,
         w*c+x*b-y*a+z*d, w*d-x*a-y*b-z*c)
    return rotate(inv, tuple(l-b for l, b in zip(lt, bt))), q


def iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    intersection = max(0, min(ax+aw, bx+bw)-max(ax, bx)) * max(
        0, min(ay+ah, by+bh)-max(ay, by))
    return intersection/max(1e-9, aw*ah+bw*bh-intersection)


def select_centered_person(persons, image_size, center_margin_px=32.):
    """Pick the person whose box center is nearest the image center line.

    This is the simple rule used by the wastebasket_car follow script: when
    several people are visible, the one closest to the horizontal center is
    the intended target.  A center margin is kept as a safety guard: two
    people at nearly the same bearing cannot be told apart from RGB alone and
    must not be swapped silently.  Only call this when no remembered target is
    being reacquired.
    """
    if not persons:
        return None, 'no_person'
    width = float(image_size[0])
    ranked = []
    for person in persons:
        try:
            x, y, w, h = [float(v) for v in person['bbox']]
        except (KeyError, TypeError, ValueError):
            continue
        if not all(math.isfinite(v) for v in (x, y, w, h)) or w <= 0 or h <= 0:
            continue
        ranked.append((abs(x+w/2-width/2), person))
    if not ranked:
        return None, 'no_person'
    ranked.sort(key=lambda item: item[0])
    if len(ranked) > 1 and ranked[1][0]-ranked[0][0] < center_margin_px:
        return None, 'ambiguous_person_choice'
    return ranked[0][1], 'centered_person'


def project_base_point(point, config, width, height):
    """Project a base-frame LiDAR endpoint into the color camera.

    camera_pitch_down_deg is positive when the camera looks at the floor.
    Base axes: forward/left/up; optical axes: right/down/forward.
    """
    offset = config['camera_xyz_m']
    x, y, z = [p-o for p, o in zip(point, offset)]
    yaw = math.radians(config['camera_yaw_deg'])
    pitch = math.radians(config['camera_pitch_down_deg'])
    forward = math.cos(yaw)*x+math.sin(yaw)*y
    left = -math.sin(yaw)*x+math.cos(yaw)*y
    optical_z = math.cos(pitch)*forward-math.sin(pitch)*z
    if optical_z <= .05:
        return None
    optical_y = -math.sin(pitch)*forward-math.cos(pitch)*z
    sx, sy = width/config['image_width'], height/config['image_height']
    return (sx*(config['cx']-config['fx']*left/optical_z),
            sy*(config['cy']+config['fy']*optical_y/optical_z))


def moving_away(history, now, window_s=2.5, distance_gain_m=.45, robot_gain_m=.15):
    """True when the robot moved but the person ended up farther away.

    history entries are (monotonic_time, person_distance_m, robot_x, robot_y).
    Requiring both a distance increase and robot translation avoids cancelling
    merely because the person walked away while the robot was stationary.
    """
    recent = [h for h in history if now-h[0] <= window_s]
    if len(recent) < 5 or now-recent[0][0] < window_s*.8:
        return False
    distances = [h[1] for h in recent]
    robot_moved = math.hypot(recent[-1][2]-recent[0][2],
                             recent[-1][3]-recent[0][3])
    return (distances[-1]-min(distances) > distance_gain_m and
            robot_moved > robot_gain_m)


def scan_clusters(ranges, angle_min, angle_increment, config):
    """Cluster the WHOLE scan before projecting: a wall must remain a wall.

    Split at Euclidean range jumps as well as missing samples. Merely
    clipping a wall to the detection's angular window creates a fake person.
    """
    clusters, current = [], []
    lower, upper = config['min_range_m'], config['max_range_m']
    for index, value in enumerate(ranges):
        angle = angle_min+index*angle_increment
        if not math.isfinite(value) or not lower <= value <= upper:
            continue
        point = (value*math.cos(angle), value*math.sin(angle), 0.)
        if current and (index-current[-1][0] > 2 or
                        math.dist(point, current[-1][1]) > config['cluster_gap_m']):
            clusters.append(current)
            current = []
        current.append((index, point))
    if current:
        clusters.append(current)
    # The beginning/end of a 360 degree scan may cut one object in half.
    if len(clusters) > 1 and abs(abs(angle_increment)*len(ranges)-2*math.pi) < .15:
        first, last = clusters[0], clusters[-1]
        if (first[0][0]+len(ranges)-last[-1][0] <= 2 and
                math.dist(first[0][1], last[-1][1]) <= config['cluster_gap_m']):
            clusters = [last+first]+clusters[1:-1]
    return clusters


def associate_person(box, image_size, clusters, laser_to_base, config, diagnostics=None,
                     prior_base=None):
    x, y, w, h = [float(v) for v in box]
    if w <= 0 or h <= 0:
        return None, 'invalid_box'
    candidates = []
    clipped_bottom = y+h >= .98*image_size[1]
    translation, quaternion = laser_to_base
    uncertainty = config.get('pitch_uncertainty_deg', 0.)
    projection_configs = [config]
    if uncertainty:
        projection_configs += [dict(config, camera_pitch_down_deg=config['camera_pitch_down_deg']+d)
                               for d in (-uncertainty, uncertainty)]
    for cluster in clusters:
        if len(cluster) < config['cluster_min_points']:
            continue
        points = [transform(translation, quaternion, p) for _, p in cluster]
        # Full cluster span rejects long walls before any image clipping.
        span = math.hypot(max(p[0] for p in points)-min(p[0] for p in points),
                          max(p[1] for p in points)-min(p[1] for p in points))
        if not config['cluster_min_width_m'] <= span <= config['cluster_max_width_m']:
            continue
        inside = []
        for point in points:
            projected = [project_base_point(point, c, *image_size) for c in projection_configs]
            projected = [uv for uv in projected if uv is not None]
            if (projected and any(x+.03*w <= uv[0] <= x+.97*w for uv in projected)
                    and max(uv[1] for uv in projected) >= y+.55*h
                    and (clipped_bottom or min(uv[1] for uv in projected) <= y+1.08*h)):
                inside.append(point)
        if len(inside) < config['cluster_min_points'] or len(inside)/len(points) < .6:
            continue
        px = sorted(p[0] for p in inside)[len(inside)//2]
        py = sorted(p[1] for p in inside)[len(inside)//2]
        pz = sum(p[2] for p in inside)/len(inside)
        # A cropped person still cannot be physically taller/wider than the
        # whole person. Reject distant furniture behind a large near-person box.
        cyaw = math.radians(config['camera_yaw_deg'])
        cpitch = math.radians(config['camera_pitch_down_deg'])
        dx, dy, dz = [a-b for a, b in zip((px, py, pz), config['camera_xyz_m'])]
        forward = math.cos(cyaw)*dx+math.sin(cyaw)*dy
        optical_z = math.cos(cpitch)*forward-math.sin(cpitch)*dz
        fx = config['fx']*image_size[0]/config['image_width']
        fy = config['fy']*image_size[1]/config['image_height']
        if (w*optical_z/fx > config.get('max_person_width_m', 1.1) or
                h*optical_z/fy > config.get('max_person_height_m', 2.2)):
            continue
        uv = project_base_point((px, py, pz), config, *image_size)
        center_error = abs(uv[0]-(x+w/2))/w
        # The LiDAR cluster must project near the centre of the person's box,
        # not merely somewhere inside it (edge matches are often walls/furniture).
        if center_error > .25:
            continue
        prior_distance = math.dist((px, py, pz), prior_base) if prior_base is not None else None
        score = center_error
        if prior_distance is not None:
            score = .3*center_error+min(2., prior_distance/.5)
        candidates.append({'base_point': (px, py, pz), 'width_m': span,
                           'points': len(inside), 'score': center_error,
                           'association_score': score, 'prior_distance_m': prior_distance,
                           'range_m': math.hypot(px, py), 'pixel': uv})
    if diagnostics is not None:
        diagnostics.extend(candidates)
    if not candidates:
        return None, 'no_person_lidar_cluster'
    candidates.sort(key=lambda c: c['association_score'])
    best = candidates[0]
    # Distinct objects at comparable bearings cannot be resolved by RGB alone.
    for other in candidates[1:]:
        if (other['association_score']-best['association_score'] < .2 and
                math.dist(other['base_point'], best['base_point']) > .65):
            return None, 'ambiguous_lidar_clusters'
    return best, 'candidate'


class TargetMemory:
    """Bounded memory of a measured target; held positions are never observations.

    Brief gaps retain continuity. Returning after the grace period requires
    two NEW strong RGB/LiDAR observations near the previous map position.
    This is spatial association, not a person identity recognition system.
    """
    def __init__(self, grace_s=.8, retention_s=8., resume_samples=2,
                 resume_seconds=.12, max_jump_m=.8):
        self.grace_s, self.retention_s = grace_s, retention_s
        self.resume_samples, self.resume_seconds = resume_samples, resume_seconds
        self.max_jump_m = max_jump_m
        self.clear()

    def clear(self):
        self.point = self.raw_point = self.box = None
        self.last_seen = None
        self.last_source = None
        self.pending = deque(maxlen=max(8, self.resume_samples))
        self.reacquired = False

    def age(self, now):
        return float('inf') if self.last_seen is None else max(0., now-self.last_seen)

    def observe(self, now, source_stamp, box, point, strong=True):
        self.reacquired = False
        if source_stamp == self.last_source or not strong:
            return None, 'weak_or_duplicate'
        self.last_source = source_stamp
        age = self.age(now)
        if self.point is not None and age <= self.retention_s:
            limit = min(self.max_jump_m, .30+.8*age)
            if math.dist(point, self.raw_point) > limit:
                self.pending.clear()
                return None, 'target_position_mismatch'
            if age > self.grace_s:
                if self.pending and (now-self.pending[-1][0] > .5 or
                                     math.dist(point, self.pending[-1][2]) > .3 or
                                     iou(box, self.pending[-1][1]) < .15):
                    self.pending.clear()
                self.pending.append((now, box, point))
                if (len(self.pending) < self.resume_samples or
                        now-self.pending[0][0] < self.resume_seconds):
                    return None, 'confirming_returned_target'
                self.reacquired = True
        elif self.point is not None:
            self.clear()
            self.last_source = source_stamp
        if self.point is None:
            self.point = tuple(point)
        else:
            alpha = .65 if self.reacquired else .4
            self.point = tuple(a+alpha*(b-a) for a, b in zip(self.point, point))
        self.raw_point, self.box = tuple(point), box
        self.last_seen = now
        self.pending.clear()
        return self.point, 'reacquired' if self.reacquired else 'observed'


class StableTarget:
    def __init__(self, frames=3, seconds=.4, jump_m=.45, max_gap_s=1.):
        self.frames, self.seconds, self.jump_m = frames, seconds, jump_m
        self.max_gap_s = max_gap_s
        self.samples = deque(maxlen=8)
        self.box = None
        self.last_source = None

    def clear(self):
        self.samples.clear()
        self.box = None
        self.last_source = None

    def add(self, source_stamp, box, point, strong=True):
        if source_stamp == self.last_source:
            return None
        self.last_source = source_stamp
        if self.samples and (source_stamp-self.samples[-1][0] > self.max_gap_s or
                             source_stamp <= self.samples[-1][0] or
                             iou(box, self.box) < .15 or
                             math.dist(point, self.samples[-1][1]) > self.jump_m):
            self.samples.clear()
        self.box = box
        if not strong:
            return None
        self.samples.append((source_stamp, point))
        if len(self.samples) < self.frames or self.samples[-1][0]-self.samples[0][0] < self.seconds:
            return None
        center = tuple(sum(p[i] for _, p in self.samples)/len(self.samples)
                       for i in (0, 1, 2))
        if max(math.dist(center, p) for _, p in self.samples) > self.jump_m:
            return None
        return center


def grid_cost(grid, x, y):
    """grid dictionary contains origin pose, resolution, dimensions and data."""
    yaw = grid.get('yaw', 0.)
    dx, dy = x-grid['x'], y-grid['y']
    col = math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/grid['resolution'])
    row = math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/grid['resolution'])
    if 0 <= col < grid['width'] and 0 <= row < grid['height']:
        return grid['data'][row*grid['width']+col]
    return None


def standoff_goal(person, robot, distance, grid, max_cost=50,
                  front_offset=0., arrival_tolerance=.05):
    # Nav2 places base_footprint (the axle), so include the body ahead of it.
    # Otherwise a requested human-to-body gap would be shortened by the chassis.
    distance += front_offset
    dx, dy = robot[0]-person[0], robot[1]-person[1]
    if math.hypot(dx, dy) <= distance+arrival_tolerance:
        return None, 'already_close'
    angle = math.atan2(dy, dx)
    # Try nearby stopping points if the point directly in front is blocked.
    for degrees in (0, 15, -15, 30, -30, 45, -45, 60, -60):
        a = angle+math.radians(degrees)
        x, y = person[0]+distance*math.cos(a), person[1]+distance*math.sin(a)
        probes = [(x, y)]+[(x+.2*math.cos(t*math.pi/4), y+.2*math.sin(t*math.pi/4))
                          for t in range(8)]
        costs = [grid_cost(grid, px, py) for px, py in probes]
        if all(c is not None and 0 <= c <= max_cost for c in costs):
            return {'x': x, 'y': y, 'yaw': math.atan2(person[1]-y, person[0]-x)}, 'goal_ready'
    return None, 'no_free_standoff_goal'
