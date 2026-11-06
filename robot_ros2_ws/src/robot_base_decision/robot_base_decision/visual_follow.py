"""Image-space person following. No map, laser, TF or navigation dependencies."""
import math

from .person_approach import iou, select_centered_person


DEFAULT_CONFIG = {
    'target_distance_m': .05, 'resume_margin_m': .05,
    'approach_deadband_m': .02,
    'max_linear_mps': .24, 'invalid_depth_speed_mps': .07,
    'distance_gain': 1.8, 'max_angular_rps': .55, 'centering_gain': 2.8,
    'image_yaw_sign': -1.,
    'center_deadband_fraction': .025, 'center_slow_fraction': .06,
    'center_stop_fraction': .18, 'max_result_age_s': .8,
    'max_depth_age_s': .5, 'max_depth_skew_s': .15,
    'min_confidence': .20, 'multi_person_center_margin_px': 32.,
    'depth_min_m': .01, 'depth_max_m': 8.,
}


def validate_config(config):
    cfg = dict(DEFAULT_CONFIG, **config)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in cfg.values()):
        raise ValueError('Visual-follow configuration must contain finite numbers')
    if not (0 < cfg['depth_min_m'] < cfg['target_distance_m'] < cfg['depth_max_m']):
        raise ValueError('Target distance must be inside accepted camera depth range')
    if not (0 < cfg['invalid_depth_speed_mps'] <= cfg['max_linear_mps'] <= .24
            and 0 < cfg['max_angular_rps'] <= .55):
        raise ValueError('Invalid speed limits')
    if not (0 <= cfg['approach_deadband_m'] and
            cfg['target_distance_m']+cfg['approach_deadband_m'] < cfg['depth_max_m']):
        raise ValueError('Invalid single-approach arrival deadband')
    if not (0 <= cfg['center_deadband_fraction'] < cfg['center_slow_fraction']
            < cfg['center_stop_fraction'] < .5):
        raise ValueError('Invalid image centering bands')
    if cfg['image_yaw_sign'] not in (-1., 1.):
        raise ValueError('image_yaw_sign must be -1 or +1')
    if not (0 <= cfg['min_confidence'] <= 1 and cfg['resume_margin_m'] > 0
            and cfg['centering_gain'] > 0 and cfg['distance_gain'] > 0
            and cfg['max_result_age_s'] > 0 and cfg['max_depth_age_s'] > 0
            and cfg['max_depth_skew_s'] > 0):
        raise ValueError('Invalid follow timing or gains')
    return cfg


class VisualFollower:
    def __init__(self, config=None):
        self.cfg = validate_config(config or {})
        self.active = False
        self.follow = False
        self.state = 'idle'
        self.box = None
        self.holding = False
        self.arrival_depth = None
        self.arrival_source_stamp = None

    def start(self, follow=False):
        self.active, self.follow = True, bool(follow)
        self.state, self.box = 'waiting_for_detection', None
        self.holding = False
        self.arrival_depth, self.arrival_source_stamp = None, None

    def stop(self, reason='stopped'):
        self.active, self.state = False, reason

    def step(self, packet, received_age, wall_time):
        approach_stop = self.cfg['target_distance_m']+self.cfg['approach_deadband_m']
        stop_distance = self.cfg['target_distance_m'] if self.follow else approach_stop
        result = dict(state=self.state, active=self.active, follow=self.follow,
                      linear_mps=0., angular_rps=0., distance_m=self.arrival_depth,
                      target_offset_px=None, target_distance_m=self.cfg['target_distance_m'],
                      image_yaw_sign=self.cfg['image_yaw_sign'],
                      arrived_latched=self.arrival_depth is not None,
                      arrival_source_stamp=self.arrival_source_stamp,
                      max_linear_mps=self.cfg['max_linear_mps'],
                      max_angular_rps=self.cfg['max_angular_rps'],
                      approach_deadband_m=self.cfg['approach_deadband_m'],
                      approach_stop_distance_m=approach_stop,
                      stop_distance_m=stop_distance)

        def finish(state, **fields):
            self.state = state
            result.update(state=state, active=self.active, **fields)
            return result

        if not self.active:
            return result
        try:
            stamp = float(packet['source_stamp'])
            age = wall_time-stamp
            if (not math.isfinite(age) or not -.1 <= age <= self.cfg['max_result_age_s']
                    or received_age > self.cfg['max_result_age_s']):
                return finish('stale_detection')
            width, height = map(float, packet['image_size'])
            if not all(math.isfinite(v) and v > 0 for v in (width, height)):
                return finish('invalid_detection')
            persons = []
            for person in packet['persons']:
                try:
                    x, y, w, h = map(float, person['bbox'])
                    confidence = float(person['conf'])
                    if (all(math.isfinite(v) for v in (x, y, w, h, confidence))
                            and w > 0 and h > 0 and 0 <= x+w/2 <= width
                            and 0 <= y+h/2 <= height
                            and confidence >= self.cfg['min_confidence']):
                        persons.append(person)
                except (KeyError, TypeError, ValueError):
                    continue
            if self.box is None:
                target, reason = select_centered_person(
                    persons, (width, height),
                    self.cfg['multi_person_center_margin_px']*width/640.)
            else:
                matches = [(iou(self.box, p['bbox']), p) for p in persons]
                matches.sort(key=lambda pair: pair[0], reverse=True)
                target, reason = None, 'target_lost'
                if matches and matches[0][0] >= .10:
                    if len(matches) > 1 and matches[1][0] >= matches[0][0]-.10:
                        reason = 'ambiguous_person_choice'
                    else:
                        target = matches[0][1]
            if target is None:
                return finish(reason)
            self.box = list(target['bbox'])
            x, _, w, _ = map(float, self.box)
            error = x+w/2-width/2
            fraction = abs(error)/width
            adjusted = max(0., fraction-self.cfg['center_deadband_fraction'])
            # Calibrate image motion against the complete camera/chassis path.
            # Keep ROS yaw unchanged downstream; the installed camera's image
            # response can have the opposite polarity to the usual projection.
            yaw = self.cfg['image_yaw_sign']*math.copysign(
                min(self.cfg['max_angular_rps'], self.cfg['centering_gain']*adjusted), error)
            result.update(target_offset_px=round(error, 1), target_bbox=self.box)
            # RGB alone drives approach and centering. Only trust a depth
            # reading for distance control when it is fresh and synchronized.
            try:
                depth_stamp = float(packet.get('depth_source_stamp'))
                depth_age = wall_time-depth_stamp
                current_depth = (math.isfinite(depth_age) and
                    -.1 <= depth_age <= self.cfg['max_depth_age_s'] and
                    abs(stamp-depth_stamp) <= self.cfg['max_depth_skew_s'])
            except (TypeError, ValueError, OverflowError):
                current_depth = False
            depth = target.get('depth_m')
            valid_depth = (current_depth and isinstance(depth, (int, float))
                           and math.isfinite(depth)
                           and self.cfg['depth_min_m'] < depth < self.cfg['depth_max_m'])
            if not valid_depth:
                # An actual distance hold remains until a valid farther
                # reading releases it. Depth loss itself never starts a hold.
                if self.holding:
                    return finish('holding_without_depth', angular_rps=yaw)
                speed, state = self.cfg['invalid_depth_speed_mps'], 'approaching_without_depth'
            else:
                result['distance_m'] = depth
                if depth <= stop_distance:
                    self.holding = True
                    if not self.follow:
                        # First valid arrival is terminal, with no debounce.
                        # Later depth rebounds cannot re-enable either wheel.
                        self.arrival_depth, self.arrival_source_stamp = depth, stamp
                        self.active = False
                        return finish('arrived', arrived_latched=True,
                                      arrival_source_stamp=stamp)
                    return finish('holding_distance', angular_rps=yaw)
                if self.holding and depth < (self.cfg['target_distance_m']+
                                            self.cfg['resume_margin_m']):
                    return finish('holding_distance', angular_rps=yaw)
                self.holding = False
                speed = min(self.cfg['max_linear_mps'],
                            self.cfg['distance_gain']*(depth-self.cfg['target_distance_m']))
                state = 'following' if self.follow else 'approaching'
            slow, stop = self.cfg['center_slow_fraction'], self.cfg['center_stop_fraction']
            speed *= max(0., min(1., (stop-fraction)/(stop-slow)))
            if fraction >= stop:
                state = 'centering'
            return finish(state, linear_mps=speed, angular_rps=yaw)
        except (KeyError, TypeError, ValueError, OverflowError):
            return finish('invalid_detection')
