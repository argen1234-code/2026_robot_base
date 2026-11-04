#!/usr/bin/env python3
"""离线分析实车导航 bag，与回放指标对齐。

用法：
    python3 tools/analyze_live_bag.py <bag1> [bag2 ...]

统计：
  · 各话题实际频率/最大间隔；
  · /scan、/scan_filtered 的周期与最近障碍距离；
  · /cmd_vel_smoothed 峰值、活动时间、等效距离、加速度、反向次数；
  · /odom 路径长度；
  · /behavior_tree_log 的 FollowPath 失败、目标成功/失败与每目标耗时；
  · /goal_sanitizer_status 文本。
"""

import json
import statistics
import sys
from collections import Counter, defaultdict

import rosbag2_py
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.msg import BehaviorTreeLog
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Rate:
    def __init__(self):
        self.times = []

    def add(self, t):
        self.times.append(t)

    def summary(self):
        ts = self.times
        if len(ts) < 2:
            return {'count': len(ts), 'rate_hz': 0.0, 'max_gap_s': 0.0,
                    'period_med_s': 0.0}
        gaps = [b - a for a, b in zip(ts, ts[1:]) if b > a]
        span = ts[-1] - ts[0]
        return {
            'count': len(ts),
            'rate_hz': round((len(ts) - 1) / span, 3) if span > 0 else 0.0,
            'max_gap_s': round(max(gaps), 4) if gaps else 0.0,
            'period_med_s': round(statistics.median(gaps), 4) if gaps else 0.0,
        }


def analyze(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    rates = defaultdict(Rate)
    cmd = {
        'count': 0, 'max_vx': 0.0, 'max_wz': 0.0, 'active_s': 0.0,
        'dist': 0.0, 'sum_abs_dvx': 0.0, 'sum_abs_dwz': 0.0, 'sum_dt': 0.0,
        'max_acc_vx': 0.0, 'max_acc_wz': 0.0, 'reversals': 0,
        '_t': None, '_vx': 0.0, '_wz': 0.0,
    }
    odom_path = 0.0
    odom_last = None
    odom_max_v = 0.0
    bt_fail = Counter()
    bt_success = Counter()
    goal_durations = []
    goal_start = None
    min_ranges = []
    scan_periods = []
    first_t = last_t = None
    scan_last_stamp = None
    start_stamp = None
    end_stamp = None

    while reader.has_next():
        topic, data, t = reader.read_next()
        t_s = t / 1e9
        if first_t is None:
            first_t = t_s
        last_t = t_s
        rates[topic].add(t_s)

        if topic in ('/scan', '/scan_filtered'):
            m = deserialize_message(data, LaserScan)
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if topic == '/scan_filtered':
                valid = [r for r in m.ranges
                         if 0.05 < r < 12.0 and r == r]
                if valid:
                    min_ranges.append(min(valid))
            if topic == '/scan':
                if scan_last_stamp is not None:
                    scan_periods.append(stamp - scan_last_stamp)
                scan_last_stamp = stamp
            if start_stamp is None:
                start_stamp = stamp
            end_stamp = stamp
        elif topic == '/cmd_vel_smoothed':
            m = deserialize_message(data, Twist)
            cmd['count'] += 1
            cmd['max_vx'] = max(cmd['max_vx'], abs(m.linear.x))
            cmd['max_wz'] = max(cmd['max_wz'], abs(m.angular.z))
            if cmd['_t'] is not None:
                dt = t_s - cmd['_t']
                if 0 < dt < 0.5:
                    cmd['dist'] += abs(cmd['_vx']) * dt
                    if abs(m.linear.x) > 0.01 or abs(m.angular.z) > 0.01:
                        cmd['active_s'] += dt
                    if dt >= 0.01:
                        dvx = (m.linear.x - cmd['_vx']) / dt
                        dwz = (m.angular.z - cmd['_wz']) / dt
                        cmd['max_acc_vx'] = max(cmd['max_acc_vx'], abs(dvx))
                        cmd['max_acc_wz'] = max(cmd['max_acc_wz'], abs(dwz))
                    cmd['sum_abs_dvx'] += abs(m.linear.x - cmd['_vx'])
                    cmd['sum_abs_dwz'] += abs(m.angular.z - cmd['_wz'])
                    cmd['sum_dt'] += dt
                    if cmd['_vx'] * m.linear.x < -1e-6:
                        cmd['reversals'] += 1
            cmd['_t'] = t_s
            cmd['_vx'] = m.linear.x
            cmd['_wz'] = m.angular.z
        elif topic == '/odom':
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position
            if odom_last is not None:
                dx, dy = p.x - odom_last[0], p.y - odom_last[1]
                odom_path += (dx * dx + dy * dy) ** 0.5
            odom_last = (p.x, p.y)
            odom_max_v = max(odom_max_v, abs(m.twist.twist.linear.x))
        elif topic == '/behavior_tree_log':
            m = deserialize_message(data, BehaviorTreeLog)
            for e in m.event_log:
                stamp = e.timestamp.sec + e.timestamp.nanosec * 1e-9
                if e.current_status == 'FAILURE':
                    bt_fail[e.node_name] += 1
                elif e.current_status == 'SUCCESS':
                    bt_success[e.node_name] += 1
                if e.node_name == 'NavigateWithReplanning':
                    if e.current_status == 'RUNNING' and e.previous_status == 'IDLE':
                        goal_start = stamp
                    elif (e.current_status in ('SUCCESS', 'FAILURE')
                          and goal_start is not None):
                        goal_durations.append({
                            'duration_s': round(stamp - goal_start, 2),
                            'status': e.current_status,
                        })
                        goal_start = None

    def rs(key):
        return rates[key].summary() if key in rates else Rate().summary()

    cmd_summary = {
        'count': cmd['count'],
        'max_vx_mps': round(cmd['max_vx'], 3),
        'max_wz_radps': round(cmd['max_wz'], 3),
        'active_s': round(cmd['active_s'], 2),
        'dist_proxy_m': round(cmd['dist'], 3),
        'max_acc_vx': round(cmd['max_acc_vx'], 3),
        'max_acc_wz': round(cmd['max_acc_wz'], 3),
        'mean_abs_acc_vx': round(
            cmd['sum_abs_dvx'] / cmd['sum_dt'], 3) if cmd['sum_dt'] else 0.0,
        'mean_abs_acc_wz': round(
            cmd['sum_abs_dwz'] / cmd['sum_dt'], 3) if cmd['sum_dt'] else 0.0,
        'vx_reversals': cmd['reversals'],
    }
    return {
        'bag': path,
        'duration_s': round(last_t - first_t, 2) if first_t else 0,
        'scan_period_med_s': round(statistics.median(scan_periods), 4)
        if scan_periods else 0,
        'scan_period_max_s': round(max(scan_periods), 4)
        if scan_periods else 0,
        'scan_period_min_s': round(min(scan_periods), 4)
        if scan_periods else 0,
        'min_range_p1_m': round(sorted(min_ranges)[len(min_ranges) // 100], 3)
        if min_ranges else None,
        'min_range_abs_m': round(min(min_ranges), 3) if min_ranges else None,
        'rates': {k: rates[k].summary() for k in (
            '/scan', '/scan_filtered', '/cmd_vel_smoothed',
            '/local_costmap/published_footprint', '/local_costmap/costmap',
            '/local_costmap/costmap_raw', '/global_costmap/costmap',
            '/odom', '/plan', '/local_plan') if k in rates},
        'cmd_vel': cmd_summary,
        'odom_path_m': round(odom_path, 3),
        'odom_max_vx_mps': round(odom_max_v, 3),
        'bt_failure_counts': dict(bt_fail),
        'bt_success_counts': dict(bt_success),
        'goal_durations': goal_durations,
        'goal_count': sum(1 for _ in goal_durations),
    }


def main():
    bags = sys.argv[1:]
    if not bags:
        print(__doc__)
        return 1
    rows = []
    for bag in bags:
        d = analyze(bag)
        rows.append(d)
        print('=' * 20, bag)
        print(json.dumps(d, indent=2, ensure_ascii=False))

    keys = ['bag', 'duration_s', 'goal_count', 'scan_period_med_s',
            'scan_period_max_s', 'min_range_p1_m',
            'cmd.max_vx_mps', 'cmd.max_wz_radps', 'cmd.mean_abs_acc_vx',
            'cmd.mean_abs_acc_wz', 'cmd.vx_reversals', 'odom_path_m',
            'follow_path_fail', 'goals_ok', 'goals_fail', 'local_update_hz',
            'local_publish_hz']
    print('\n=== COMPARISON ===')
    print('| ' + ' | '.join(keys) + ' |')
    print('|' + '|'.join(['---'] * len(keys)) + '|')
    for d in rows:
        r = d.get('rates', {})
        ok = d.get('bt_success_counts', {}).get('NavigateWithReplanning', 0)
        fail = sum(1 for x in d.get('goal_durations', [])
                   if x.get('status') == 'FAILURE')
        vals = {
            'bag': d['bag'].split('/')[-1],
            'duration_s': d['duration_s'],
            'goal_count': d['goal_count'],
            'scan_period_med_s': d['scan_period_med_s'],
            'scan_period_max_s': d['scan_period_max_s'],
            'min_range_p1_m': d['min_range_p1_m'],
            'cmd.max_vx_mps': d['cmd_vel']['max_vx_mps'],
            'cmd.max_wz_radps': d['cmd_vel']['max_wz_radps'],
            'cmd.mean_abs_acc_vx': d['cmd_vel']['mean_abs_acc_vx'],
            'cmd.mean_abs_acc_wz': d['cmd_vel']['mean_abs_acc_wz'],
            'cmd.vx_reversals': d['cmd_vel']['vx_reversals'],
            'odom_path_m': d['odom_path_m'],
            'follow_path_fail': d['bt_failure_counts'].get('FollowPath', 0),
            'goals_ok': ok,
            'goals_fail': fail,
            'local_update_hz': r.get('/local_costmap/published_footprint', {}).get('rate_hz'),
            'local_publish_hz': r.get('/local_costmap/costmap', {}).get('rate_hz'),
        }
        print('| ' + ' | '.join(str(vals.get(k, '')) for k in keys) + ' |')
    return 0


if __name__ == '__main__':
    sys.exit(main())
