#!/usr/bin/env python3
"""按预设的 8 轮参数矩阵，连续跑离线 Nav2 回放并汇总指标。

每轮都从基准参数出发、累积应用前面所有改动，写入独立的 params_Rn.yaml；
然后调用 tools/run_nav2_replay.sh 跑同一个 bag。

用法：
    python3 tools/nav2_replay_rounds.py                # R1..R8
    python3 tools/nav2_replay_rounds.py --from R2 --to R5
    python3 tools/nav2_replay_rounds.py --dry-run      # 只生成参数文件
"""

import argparse
import copy
import json
import os
import subprocess
import sys
import time

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE_PARAMS = os.path.join(
    ROOT, 'robot_ros2_ws', 'src', 'robot_base_bringup', 'config',
    'nav2_params_prior.yaml')
RUNNER = os.path.join(ROOT, 'tools', 'run_nav2_replay.sh')

ROS_PARAMS = 'ros__parameters'

# 每轮的增量 patch（路径, 值）。路径里不含 ros__parameters 的层级会补全。
ROUNDS = [
    ('R1_baseline', []),
    ('R2_robust', [
        (('controller_server', ROS_PARAMS, 'failure_tolerance'), 1.0),
        (('controller_server', ROS_PARAMS, 'FollowPath',
          'transform_tolerance'), 0.5),
        # 局部窗口只有 3x3 m，8 m 射线清除纯属浪费；R1 实测代价地图被
        # 拖到 6.4 Hz，先把它压到覆盖窗口对角线的 3 m。
        (('local_costmap', 'local_costmap', ROS_PARAMS, 'obstacle_layer',
          'scan', 'raytrace_max_range'), 3.0),
        (('local_costmap', 'local_costmap', ROS_PARAMS, 'obstacle_layer',
          'scan', 'obstacle_max_range'), 3.0),
    ]),
    ('R3_costmap12', [
        (('local_costmap', 'local_costmap', ROS_PARAMS,
          'update_frequency'), 12.0),
        (('local_costmap', 'local_costmap', ROS_PARAMS,
          'publish_frequency'), 12.0),
    ]),
    ('R4_ctrl18_smooth25', [
        (('controller_server', ROS_PARAMS, 'controller_frequency'), 18.0),
        (('velocity_smoother', ROS_PARAMS, 'smoothing_frequency'), 25.0),
    ]),
    ('R5_ctrl20', [
        (('controller_server', ROS_PARAMS, 'controller_frequency'), 20.0),
    ]),
    ('R6_turn', [
        (('controller_server', ROS_PARAMS, 'FollowPath',
          'vx_samples'), 10),
        (('controller_server', ROS_PARAMS, 'FollowPath',
          'vtheta_samples'), 20),
        (('controller_server', ROS_PARAMS, 'FollowPath',
          'sim_time'), 1.7),
        (('controller_server', ROS_PARAMS, 'FollowPath',
          'acc_lim_theta'), 0.9),
        (('controller_server', ROS_PARAMS, 'FollowPath',
          'decel_lim_theta'), -1.1),
        (('controller_server', ROS_PARAMS, 'FollowPath',
          'RotateToGoal.slowing_factor'), 2.0),
        (('controller_server', ROS_PARAMS, 'FollowPath',
          'RotateToGoal.scale'), 15.0),
        (('velocity_smoother', ROS_PARAMS, 'max_accel'), [0.25, 0.0, 0.9]),
        (('velocity_smoother', ROS_PARAMS, 'max_decel'), [-0.35, 0.0, -1.1]),
        (('behavior_server', ROS_PARAMS, 'max_rotational_vel'), 0.55),
        (('behavior_server', ROS_PARAMS, 'rotational_acc_lim'), 0.9),
    ]),
    ('R7_inflation', [
        (('local_costmap', 'local_costmap', ROS_PARAMS,
          'inflation_layer', 'inflation_radius'), 0.40),
        (('local_costmap', 'local_costmap', ROS_PARAMS,
          'inflation_layer', 'cost_scaling_factor'), 6.0),
    ]),
    ('R8_accel', [
        (('controller_server', ROS_PARAMS, 'FollowPath',
          'acc_lim_x'), 0.35),
        (('controller_server', ROS_PARAMS, 'FollowPath',
          'decel_lim_x'), -0.50),
        (('velocity_smoother', ROS_PARAMS, 'max_accel'), [0.35, 0.0, 0.9]),
        (('velocity_smoother', ROS_PARAMS, 'max_decel'), [-0.50, 0.0, -1.1]),
    ]),
]


def set_path(tree, path, value):
    node = tree
    for key in path[:-1]:
        if key not in node:
            raise KeyError(f'params path missing: {".".join(path)} (at {key})')
        node = node[key]
    if path[-1] not in node and not isinstance(node, dict):
        raise KeyError(f'params path missing: {".".join(path)}')
    node[path[-1]] = value


def build_rounds(base, upto_index):
    tree = copy.deepcopy(base)
    applied = []
    for _name, patches in ROUNDS[:upto_index + 1]:
        for path, value in patches:
            set_path(tree, path, value)
            applied.append((path, value))
    return tree, applied


def run_round(name, params_path, out_dir, rate):
    os.makedirs(out_dir, exist_ok=True)
    cmd = ['bash', RUNNER, '--params', params_path, '--out', out_dir,
           '--rate', str(rate)]
    print(f"[rounds] {name}: {' '.join(cmd)}", flush=True)
    t0 = time.time()
    rc = subprocess.run(cmd, cwd=ROOT).returncode
    elapsed = time.time() - t0
    metrics = {}
    try:
        with open(os.path.join(out_dir, 'metrics.json')) as f:
            metrics = json.load(f)
    except (OSError, json.JSONDecodeError):
        pass
    return rc, elapsed, metrics


def row(name, metrics):
    def rate(key):
        return metrics.get('rates', {}).get(key, {}).get('rate_hz', 0.0)

    def gap(key):
        return metrics.get('rates', {}).get(key, {}).get('max_gap_s', 0.0)

    cmd = metrics.get('cmd_vel', {}).get('cmd_vel_smoothed', {})
    fail = metrics.get('bt_failure_counts', {})
    ok = metrics.get('bt_success_counts', {})
    durations = metrics.get('goal_durations', [])
    cpu = metrics.get('cpu', {})
    run = metrics.get('run', {})
    return {
        'round': name,
        'goals_ok': ok.get('NavigateWithReplanning', 0),
        'goals_fail': sum(1 for d in durations if d.get('status') == 'FAILURE'),
        'goal_times_s': [d.get('duration_s') for d in durations],
        'follow_path_fail': fail.get('FollowPath', 0),
        'local_costmap_hz': rate('local_costmap'),
        'local_costmap_gap_s': gap('local_costmap'),
        'scan_hz': rate('scan_filtered'),
        'scan_gap_s': gap('scan_filtered'),
        'max_vx': cmd.get('max_vx_mps', 0.0),
        'max_wz': cmd.get('max_wz_radps', 0.0),
        'active_s': cmd.get('active_s', 0.0),
        'dist_proxy_m': cmd.get('dist_proxy_m', 0.0),
        'max_acc_vx': cmd.get('max_acc_vx'),
        'max_acc_wz': cmd.get('max_acc_wz'),
        'mean_acc_vx': cmd.get('mean_abs_acc_vx'),
        'mean_acc_wz': cmd.get('mean_abs_acc_wz'),
        'vx_reversals': cmd.get('vx_reversals'),
        'nav2_cpu_mean_pct': cpu.get('nav2_total_mean_pct'),
        'temp_max_c': run.get('temp_max_c'),
    }


def write_summary(out_base, rows):
    cols = ['round', 'goals_ok', 'goals_fail', 'follow_path_fail',
            'local_costmap_hz', 'local_costmap_gap_s', 'scan_hz',
            'scan_gap_s', 'max_vx', 'max_wz', 'active_s', 'dist_proxy_m',
            'max_acc_vx', 'max_acc_wz', 'mean_acc_vx', 'mean_acc_wz',
            'vx_reversals', 'nav2_cpu_mean_pct', 'temp_max_c']
    lines = ['| ' + ' | '.join(cols) + ' |',
             '|' + '|'.join(['---'] * len(cols)) + '|']
    for r in rows:
        lines.append('| ' + ' | '.join(str(r.get(c, '')) for c in cols) + ' |')
    md = '\n'.join(lines) + '\n'
    with open(os.path.join(out_base, 'summary.md'), 'w') as f:
        f.write('# Nav2 offline replay rounds\n\n' + md)
    with open(os.path.join(out_base, 'summary.json'), 'w') as f:
        json.dump(rows, f, indent=2, ensure_ascii=False)
    print(md, flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--from', dest='first', default='R1',
                        help='start round key, e.g. R2')
    parser.add_argument('--to', dest='last', default='R8', help='end round key')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--out-base', default=None)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()

    with open(BASE_PARAMS) as f:
        base = yaml.safe_load(f)

    names = [n for n, _ in ROUNDS]

    def resolve(token):
        if token in names:
            return names.index(token)
        matches = [i for i, n in enumerate(names) if n.startswith(token)]
        if len(matches) != 1:
            parser.error(f'round {token!r} ambiguous/unknown; choose from {names}')
        return matches[0]

    i0, i1 = resolve(args.first), resolve(args.last)
    out_base = args.out_base or os.path.join(
        '/home/pi/rosbags/nav2_replay',
        time.strftime('rounds_%Y%m%d_%H%M%S'))
    os.makedirs(out_base, exist_ok=True)

    rows = []
    for idx in range(i0, i1 + 1):
        name = names[idx]
        tree, _applied = build_rounds(base, idx)
        params = os.path.join(out_base, f'params_{name}.yaml')
        with open(params, 'w') as f:
            yaml.safe_dump(tree, f, sort_keys=False, allow_unicode=True)
        if args.dry_run:
            print(f'[dry-run] wrote {params}')
            continue
        out_dir = os.path.join(out_base, name)
        rc, elapsed, metrics = run_round(name, params, out_dir, args.rate)
        print(f'[rounds] {name} rc={rc} elapsed={elapsed:.0f}s', flush=True)
        if rc != 0:
            print(f'[rounds] {name} failed; stop', file=sys.stderr)
            break
        row_data = row(name, metrics)
        row_data['elapsed_s'] = round(elapsed, 1)
        rows.append(row_data)
        print(json.dumps(row_data, ensure_ascii=False), flush=True)

    if rows:
        write_summary(out_base, rows)
    print(f'[rounds] done -> {out_base}')


if __name__ == '__main__':
    main()
