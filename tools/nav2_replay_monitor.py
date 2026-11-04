#!/usr/bin/env python3
"""离线 Nav2 回放的指标采集节点。

订阅回放栈的输出，统计：
  · /scan_filtered、局部/全局代价地图的实际频率与最大间隔；
  · /cmd_vel_nav、/cmd_vel_smoothed 的峰值、活动时间、等效前进距离；
  · /odom 路径长度；
  · /behavior_tree_log 的 FollowPath / NavigateWithReplanning 失败与成功次数；
  · /goal_pose_safe 数量、/goal_sanitizer_status 文本；
  · 各 Nav2 进程的 CPU、系统负载、CPU 温度。

退出（Ctrl-C / SIGTERM）时把汇总写到 --out 指定的 JSON，并定期落盘。
"""

import argparse
import json
import os
import signal
import time
from collections import Counter

import psutil
import rclpy
from geometry_msgs.msg import PolygonStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.msg import BehaviorTreeLog, Costmap
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

PROC_NAMES = (
    'controller_server', 'planner_server', 'smoother_server',
    'behavior_server', 'bt_navigator', 'velocity_smoother',
    'goal_sanitizer', 'bag_replay_relay',
)


def reliable_qos(depth=10):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class RateTracker:
    def __init__(self):
        self.count = 0
        self.first = None
        self.last = None
        self.max_gap = 0.0

    def add(self, now):
        if self.first is None:
            self.first = now
        else:
            self.max_gap = max(self.max_gap, now - self.last)
        self.last = now
        self.count += 1

    def summary(self):
        out = {'count': self.count, 'max_gap_s': round(self.max_gap, 4)}
        if self.first is not None and self.last is not None:
            span = self.last - self.first
            out['span_s'] = round(span, 3)
            out['rate_hz'] = round((self.count - 1) / span, 3) if span > 0 else 0.0
        return out


class Nav2ReplayMonitor(Node):
    def __init__(self, out_path):
        super().__init__('nav2_replay_monitor')
        self.out_path = out_path
        self.started = time.monotonic()
        self.rates = {
            'scan_filtered': RateTracker(),
            'local_costmap': RateTracker(),
            'local_costmap_raw': RateTracker(),
            'local_footprint': RateTracker(),
            'global_costmap': RateTracker(),
            'cmd_vel_nav': RateTracker(),
            'cmd_vel_smoothed': RateTracker(),
            'odom': RateTracker(),
            'plan': RateTracker(),
            'local_plan': RateTracker(),
        }
        self.cmd = {
            'cmd_vel_nav': self._new_cmd(),
            'cmd_vel_smoothed': self._new_cmd(),
        }
        self.odom_path = 0.0
        self._odom_last = None
        self.bt_failures = []
        self.bt_success = Counter()
        self.bt_events = Counter()
        self.goal_start = None
        self.goal_durations = []
        self.goal_safe = 0
        self.sanitizer_status = Counter()
        self.cpu_samples = []
        self.per_proc = {}
        self._proc_cache = {}
        self._last_write = 0.0
        self._temp_max = 0.0
        self._temp_sum = 0.0
        self._temp_n = 0
        self.window_t = 0.0
        self.window_last = {}
        self.windows = []
        self.window_keys = ('scan_filtered', 'local_costmap',
                            'local_costmap_raw', 'local_footprint',
                            'cmd_vel_smoothed')

        qos = reliable_qos(10)
        self.create_subscription(LaserScan, '/scan_filtered',
                                 self.on_scan, qos)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 self.on_local_costmap, qos)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw',
                                 self.on_local_costmap_raw, qos)
        self.create_subscription(PolygonStamped,
                                 '/local_costmap/published_footprint',
                                 self.on_local_footprint, qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.on_global_costmap, qos)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_cmd_nav, qos)
        self.create_subscription(Twist, '/cmd_vel_smoothed',
                                 self.on_cmd_smoothed, qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos)
        self.create_subscription(Path, '/plan', self.on_plan, qos)
        self.create_subscription(Path, '/local_plan', self.on_local_plan, qos)
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log',
                                 self.on_bt, qos)
        from geometry_msgs.msg import PoseStamped
        self.create_subscription(PoseStamped, '/goal_pose_safe',
                                 self.on_goal_safe, qos)
        self.create_subscription(String, '/goal_sanitizer_status',
                                 self.on_status, qos)
        self.create_timer(1.0, self.sample_cpu)
        self.get_logger().info(f'monitor writing {out_path}')

    @staticmethod
    def _new_cmd():
        return {
            'count': 0, 'max_vx': 0.0, 'max_wz': 0.0,
            'active_s': 0.0, 'dist_proxy_m': 0.0,
            'max_acc_vx': 0.0, 'max_acc_wz': 0.0,
            'sum_abs_dvx': 0.0, 'sum_abs_dwz': 0.0, 'sum_dt': 0.0,
            'reversals': 0,
            '_last_t': None, '_last_vx': 0.0, '_last_wz': 0.0,
        }

    def _add_cmd(self, key, msg):
        now = time.monotonic()
        c = self.cmd[key]
        c['count'] += 1
        c['max_vx'] = max(c['max_vx'], abs(msg.linear.x))
        c['max_wz'] = max(c['max_wz'], abs(msg.angular.z))
        if c['_last_t'] is not None:
            dt = min(now - c['_last_t'], 0.5)
            c['dist_proxy_m'] += abs(c['_last_vx']) * dt
            if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
                c['active_s'] += dt
            if dt > 1e-6:
                dvx = (msg.linear.x - c['_last_vx']) / dt
                dwz = (msg.angular.z - c['_last_wz']) / dt
                c['max_acc_vx'] = max(c['max_acc_vx'], abs(dvx))
                c['max_acc_wz'] = max(c['max_acc_wz'], abs(dwz))
                c['sum_abs_dvx'] += abs(msg.linear.x - c['_last_vx'])
                c['sum_abs_dwz'] += abs(msg.angular.z - c['_last_wz'])
                c['sum_dt'] += dt
            if c['_last_vx'] * msg.linear.x < -1e-6:
                c['reversals'] += 1
        c['_last_t'] = now
        c['_last_vx'] = msg.linear.x
        c['_last_wz'] = msg.angular.z

    def on_scan(self, _msg):
        self.rates['scan_filtered'].add(time.monotonic())

    def on_local_costmap(self, _msg):
        self.rates['local_costmap'].add(time.monotonic())

    def on_local_costmap_raw(self, _msg):
        self.rates['local_costmap_raw'].add(time.monotonic())

    def on_local_footprint(self, _msg):
        self.rates['local_footprint'].add(time.monotonic())

    def on_global_costmap(self, _msg):
        self.rates['global_costmap'].add(time.monotonic())

    def on_cmd_nav(self, msg):
        self.rates['cmd_vel_nav'].add(time.monotonic())
        self._add_cmd('cmd_vel_nav', msg)

    def on_cmd_smoothed(self, msg):
        self.rates['cmd_vel_smoothed'].add(time.monotonic())
        self._add_cmd('cmd_vel_smoothed', msg)

    def on_odom(self, msg):
        self.rates['odom'].add(time.monotonic())
        p = msg.pose.pose.position
        if self._odom_last is not None:
            dx = p.x - self._odom_last[0]
            dy = p.y - self._odom_last[1]
            self.odom_path += (dx * dx + dy * dy) ** 0.5
        self._odom_last = (p.x, p.y)

    def on_plan(self, _msg):
        self.rates['plan'].add(time.monotonic())

    def on_local_plan(self, _msg):
        self.rates['local_plan'].add(time.monotonic())

    def on_bt(self, msg):
        for e in msg.event_log:
            self.bt_events[(e.node_name, e.current_status)] += 1
            stamp = e.timestamp.sec + e.timestamp.nanosec * 1e-9
            if e.current_status == 'FAILURE':
                self.bt_failures.append({
                    'node': e.node_name,
                    'uid': int(e.uid),
                    'stamp_s': round(stamp, 3),
                })
            elif e.current_status == 'SUCCESS':
                self.bt_success[e.node_name] += 1
            if e.node_name == 'NavigateWithReplanning':
                if (e.current_status == 'RUNNING'
                        and e.previous_status == 'IDLE'):
                    self.goal_start = stamp
                elif (e.current_status in ('SUCCESS', 'FAILURE')
                      and self.goal_start is not None):
                    self.goal_durations.append({
                        'duration_s': round(stamp - self.goal_start, 2),
                        'status': e.current_status,
                    })
                    self.goal_start = None

    def on_goal_safe(self, _msg):
        self.goal_safe += 1

    def on_status(self, msg):
        self.sanitizer_status[msg.data] += 1

    def _read_temp(self):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp') as f:
                t = int(f.read().strip()) / 1000.0
            self._temp_sum += t
            self._temp_n += 1
            self._temp_max = max(self._temp_max, t)
        except (OSError, ValueError):
            pass

    def sample_cpu(self):
        self._read_temp()
        now = time.monotonic()
        total_pct = 0.0
        total_rss_mb = 0.0
        found = {}
        found_mem = {}
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = proc.info['cmdline'] or []
                text = ' '.join(cmdline) + ' ' + (proc.info['name'] or '')
                if not any(n in text for n in PROC_NAMES):
                    continue
                p = self._proc_cache.get(proc.pid)
                if p is None or not p.is_running():
                    p = psutil.Process(proc.pid)
                    p.cpu_percent(None)
                    self._proc_cache[proc.pid] = p
                    continue
                pct = p.cpu_percent(None)
                rss_mb = p.memory_info().rss / 1048576.0
                key = next(n for n in PROC_NAMES if n in text)
                found[key] = found.get(key, 0.0) + pct
                found_mem[key] = found_mem.get(key, 0.0) + rss_mb
                total_pct += pct
                total_rss_mb += rss_mb
            except (psutil.NoSuchProcess, psutil.AccessDenied, ValueError):
                continue
        self.per_proc.setdefault('samples', 0)
        for key, pct in found.items():
            d = self.per_proc.setdefault(
                key, {'sum': 0.0, 'max': 0.0, 'n': 0,
                      'mem_sum': 0.0, 'mem_max': 0.0})
            d['sum'] += pct
            d['max'] = max(d['max'], pct)
            d['mem_sum'] += found_mem.get(key, 0.0)
            d['mem_max'] = max(d['mem_max'], found_mem.get(key, 0.0))
            d['n'] += 1
        self.per_proc['samples'] += 1
        self.cpu_samples.append({
            't': round(now - self.started, 1),
            'nav2_total_pct': round(total_pct, 1),
            'nav2_mem_mb': round(total_rss_mb, 1),
            'system_pct': psutil.cpu_percent(None),
            'load1': round(os.getloadavg()[0], 2),
        })
        rel = now - self.started
        if rel - self.window_t >= 10.0:
            row = {'t': round(rel, 1)}
            dt = rel - self.window_t
            for key in self.window_keys:
                cur = self.rates[key].count
                prev = self.window_last.get(key, cur)
                row[key + '_hz'] = round((cur - prev) / dt, 2)
                self.window_last[key] = cur
            self.windows.append(row)
            self.window_t = rel
        if now - self._last_write >= 10.0:
            self._write()
            self._last_write = now

    def _summary(self):
        def proc_summary():
            out = {}
            n = max(1, self.per_proc.get('samples', 0))
            for key, d in self.per_proc.items():
                if key == 'samples' or not isinstance(d, dict) or d.get('n', 0) == 0:
                    continue
                out[key] = {
                    'mean_pct': round(d['sum'] / d['n'], 1),
                    'max_pct': round(d['max'], 1),
                    'mean_mem_mb': round(d.get('mem_sum', 0.0) / d['n'], 1),
                    'max_mem_mb': round(d.get('mem_max', 0.0), 1),
                }
            _ = n
            return out

        def cmd_summary(key):
            c = self.cmd[key]
            return {
                'count': c['count'],
                'max_vx_mps': round(c['max_vx'], 4),
                'max_wz_radps': round(c['max_wz'], 4),
                'active_s': round(c['active_s'], 2),
                'dist_proxy_m': round(c['dist_proxy_m'], 3),
                'max_acc_vx': round(c['max_acc_vx'], 3),
                'max_acc_wz': round(c['max_acc_wz'], 3),
                'mean_abs_acc_vx': round(
                    c['sum_abs_dvx'] / c['sum_dt'], 3) if c['sum_dt'] > 0 else 0.0,
                'mean_abs_acc_wz': round(
                    c['sum_abs_dwz'] / c['sum_dt'], 3) if c['sum_dt'] > 0 else 0.0,
                'vx_reversals': c['reversals'],
            }

        total = [s['nav2_total_pct'] for s in self.cpu_samples]
        mem = [s.get('nav2_mem_mb', 0.0) for s in self.cpu_samples]
        return {
            'run': {
                'duration_s': round(time.monotonic() - self.started, 2),
                'cpu_count': psutil.cpu_count(),
                'temp_mean_c': round(self._temp_sum / self._temp_n, 1) if self._temp_n else None,
                'temp_max_c': round(self._temp_max, 1) if self._temp_n else None,
            },
            'rates': {k: v.summary() for k, v in self.rates.items()},
            'windows': self.windows,
            'cmd_vel': {k: cmd_summary(k) for k in self.cmd},
            'odom_path_m': round(self.odom_path, 3),
            'bt_failures': self.bt_failures,
            'bt_failure_counts': dict(Counter(f['node'] for f in self.bt_failures)),
            'bt_success_counts': dict(self.bt_success),
            'goal_durations': self.goal_durations,
            'bt_event_counts': {f'{k[0]}:{k[1]}': v for k, v in self.bt_events.items()},
            'goal_pose_safe_count': self.goal_safe,
            'sanitizer_status': dict(self.sanitizer_status),
            'cpu': {
                'nav2_total_mean_pct': round(sum(total) / len(total), 1) if total else None,
                'nav2_total_max_pct': round(max(total), 1) if total else None,
                'nav2_mem_max_mb': round(max(mem), 1) if mem else None,
                'per_process': proc_summary(),
                'system': self.cpu_samples,
            },
        }

    def _write(self):
        data = self._summary()
        tmp = self.out_path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        os.replace(tmp, self.out_path)

    def finish(self):
        self._write()
        s = self._summary()
        print('=== NAV2 REPLAY SUMMARY ===')
        print(json.dumps({
            'run': s['run'],
            'rates': s['rates'],
            'cmd_vel': s['cmd_vel'],
            'odom_path_m': s['odom_path_m'],
            'bt_failure_counts': s['bt_failure_counts'],
            'bt_success_counts': s['bt_success_counts'],
            'goal_durations': s['goal_durations'],
            'goal_pose_safe_count': s['goal_pose_safe_count'],
            'cpu': {k: v for k, v in s['cpu'].items() if k != 'system'},
        }, indent=2, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Nav2ReplayMonitor(args.out)

    def _stop(*_args):
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _stop)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
