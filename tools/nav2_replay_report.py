#!/usr/bin/env python3
"""汇总多轮离线 Nav2 回放的 metrics.json，输出 Markdown 对比表。

用法：
    python3 tools/nav2_replay_report.py <dir1> <dir2> ...
    python3 tools/nav2_replay_report.py /home/pi/rosbags/nav2_replay/R1_baseline ...
"""

import json
import os
import statistics
import sys


def load(path):
    if os.path.isdir(path):
        path = os.path.join(path, 'metrics.json')
    with open(path) as f:
        return json.load(f)


def window_stats(metrics, key, min_t=15.0):
    vals = [w[key] for w in metrics.get('windows', [])
            if w.get('t', 0) >= min_t and key in w]
    if not vals:
        return None, None, None, None
    return (round(statistics.mean(vals), 2),
            round(statistics.median(vals), 2),
            round(min(vals), 2), round(max(vals), 2))


def summarize(label, d):
    rate = lambda k: d.get('rates', {}).get(k, {}).get('rate_hz', 0.0)
    gap = lambda k: d.get('rates', {}).get(k, {}).get('max_gap_s', 0.0)
    cmd = d.get('cmd_vel', {}).get('cmd_vel_smoothed', {})
    fail = d.get('bt_failure_counts', {})
    ok = d.get('bt_success_counts', {})
    durations = d.get('goal_durations', [])
    cpu = d.get('cpu', {})
    run = d.get('run', {})

    upd_mean, upd_med, upd_min, upd_max = window_stats(
        d, 'local_footprint_hz')
    pub_mean, pub_med, pub_min, pub_max = window_stats(
        d, 'local_costmap_hz')

    def fmt4(vals):
        if not vals or any(v is None for v in vals):
            return '—'
        return '/'.join(f'{v:.2f}' for v in vals)

    return {
        'round': label,
        'goals_ok': ok.get('NavigateWithReplanning', 0),
        'goals_fail': sum(1 for x in durations if x.get('status') == 'FAILURE'),
        'follow_path_fail': fail.get('FollowPath', 0),
        'ctrl_hz': rate('cmd_vel_nav'),
        'smooth_hz': rate('cmd_vel_smoothed'),
        'update_hz_mean/med/min/max': fmt4((upd_mean, upd_med, upd_min, upd_max)),
        'publish_hz_mean/med/min/max': fmt4((pub_mean, pub_med, pub_min, pub_max)),
        'scan_hz': rate('scan_filtered'),
        'scan_gap_s': gap('scan_filtered'),
        'max_vx': cmd.get('max_vx_mps', 0.0),
        'max_wz': cmd.get('max_wz_radps', 0.0),
        'active_s': cmd.get('active_s', 0.0),
        'max_acc_vx': cmd.get('max_acc_vx'),
        'max_acc_wz': cmd.get('max_acc_wz'),
        'mean_acc_vx': cmd.get('mean_abs_acc_vx'),
        'mean_acc_wz': cmd.get('mean_abs_acc_wz'),
        'vx_reversals': cmd.get('vx_reversals'),
        'cpu_mean_pct': cpu.get('nav2_total_mean_pct'),
        'cpu_max_pct': cpu.get('nav2_total_max_pct'),
        'mem_max_mb': cpu.get('nav2_mem_max_mb'),
        'temp_max_c': run.get('temp_max_c'),
        'goal_times_s': [x.get('duration_s') for x in durations],
        'goal_status': [x.get('status') for x in durations],
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    rows = []
    for path in sys.argv[1:]:
        label = os.path.basename(os.path.normpath(path))
        rows.append(summarize(label, load(path)))

    cols = ['round', 'goals_ok', 'goals_fail', 'follow_path_fail',
            'ctrl_hz', 'smooth_hz', 'update_hz_mean/med/min/max',
            'publish_hz_mean/med/min/max', 'scan_hz', 'scan_gap_s',
            'max_vx', 'max_wz', 'active_s', 'max_acc_vx', 'max_acc_wz',
            'mean_acc_vx', 'mean_acc_wz', 'vx_reversals',
            'cpu_mean_pct', 'cpu_max_pct',
            'mem_max_mb', 'temp_max_c', 'goal_times_s', 'goal_status']
    print('| ' + ' | '.join(cols) + ' |')
    print('|' + '|'.join(['---'] * len(cols)) + '|')
    for r in rows:
        print('| ' + ' | '.join(str(r.get(c, '')) for c in cols) + ' |')
    return 0


if __name__ == '__main__':
    sys.exit(main())
