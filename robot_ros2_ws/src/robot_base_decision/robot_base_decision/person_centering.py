"""Image centering and footprint checks for the person approach velocity gate."""
import math

import numpy as np


def centered_command(linear, angular, target, config):
    """ROS positive yaw turns left; image x increases to the right."""
    width, center, focal = (float(target[k]) for k in ('width', 'center_x', 'fx'))
    if (not all(math.isfinite(v) for v in (linear, angular, width, center, focal))
            or width <= 0 or focal <= 0 or not 0 <= center <= width):
        raise ValueError('Invalid centering observation')
    error = center-width/2
    fraction = abs(error)/width
    deadband = config.get('centering_deadband_fraction', .035)
    slow = config.get('centering_slow_fraction', .08)
    stop = config.get('centering_stop_fraction', .18)
    if not 0 <= deadband < slow < stop < .5:
        raise ValueError('Invalid centering bands')
    # A stopped Nav2 command is never permission to move independently.
    if abs(linear) < .001 and abs(angular) < .01:
        return 0., 0., error
    adjusted = math.copysign(max(0., abs(error)-deadband*width), error)
    correction = -config.get('centering_gain', 2.)*math.atan2(adjusted, focal)
    yaw = angular*config.get('centering_nav_turn_weight', .2)+correction
    speed_scale = max(0., min(1., (stop-fraction)/(stop-slow)))
    return (min(.15, max(0., linear))*speed_scale,
            min(.30, max(-.30, yaw)), error)


class FootprintGuard:
    """Check the complete swept rectangle against a local occupancy grid.

    Cell centers include unknown and lethal/inscribed cells. Extra padding
    covers the half-cell diagonal and travel between the sampled poses.
    """
    def __init__(self, data, width, height, resolution, origin, yaw=0.):
        self.width, self.height, self.resolution = width, height, resolution
        self.x, self.y = origin
        self.cos, self.sin = math.cos(yaw), math.sin(yaw)
        cells = np.asarray(data, dtype=np.int16).reshape(height, width)
        rows, cols = np.nonzero((cells < 0) | (cells >= 99))
        x, y = (cols+.5)*resolution, (rows+.5)*resolution
        self.points_x = self.x+self.cos*x-self.sin*y
        self.points_y = self.y+self.sin*x+self.cos*y

    def clear(self, pose, linear, angular, front=.246, rear=.045,
              half_width=.118, horizon=1.2):
        x, y, yaw = pose
        if not all(math.isfinite(v) for v in (*pose, linear, angular)):
            return False
        radius = math.hypot(max(front, rear), half_width)
        # Limit translation plus corner rotation to half a cell per step.
        step = min(.05, self.resolution*.5/max(.01, abs(linear)+radius*abs(angular)))
        steps = max(1, math.ceil(horizon/step))
        padding = self.resolution*math.sqrt(2)/2+self.resolution*.5
        corners = np.array([[-rear-padding, -half_width-padding],
                            [-rear-padding, half_width+padding],
                            [front+padding, half_width+padding],
                            [front+padding, -half_width-padding]])
        for i in range(steps+1):
            t = horizon*i/steps
            a = yaw+angular*t
            if abs(angular) > 1e-6:
                px = x+linear/angular*(math.sin(a)-math.sin(yaw))
                py = y-linear/angular*(math.cos(a)-math.cos(yaw))
            else:
                px, py = x+linear*t*math.cos(yaw), y+linear*t*math.sin(yaw)
            c, s = math.cos(a), math.sin(a)
            wx = px+c*corners[:, 0]-s*corners[:, 1]-self.x
            wy = py+s*corners[:, 0]+c*corners[:, 1]-self.y
            gx, gy = self.cos*wx+self.sin*wy, -self.sin*wx+self.cos*wy
            if (np.any(gx < 0) or np.any(gy < 0) or
                    np.any(gx >= self.width*self.resolution) or
                    np.any(gy >= self.height*self.resolution)):
                return False
            dx, dy = self.points_x-px, self.points_y-py
            bx, by = c*dx+s*dy, -s*dx+c*dy
            if np.any((bx >= -rear-padding) & (bx <= front+padding) &
                      (np.abs(by) <= half_width+padding)):
                return False
        return True
