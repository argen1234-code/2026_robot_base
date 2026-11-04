#!/usr/bin/env python3
"""测量雷达自回波（持续性近距离波束）。

用法（在 ros_jazzy 环境、domain 1）：
    python3 tools/self_echo_measure.py --bag /path/to/scan_bag

建议录制时整车静止、周围至少 0.5 m 内没有真实障碍，这样持续性近距离
波束基本就是雷达安装结构/线缆的自回波。输出：
  · 每帧 <0.10/0.15/.../0.50 m 的波束数；
  · 占用率 >= --occupancy、且距离 < --close-range 的波束；
  · 按连续波束合并成角度扇区，给出中位/最大距离和建议 lower_threshold。
"""

import argparse
import math
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--close-range', type=float, default=0.50,
                    help='统计的近距离上限（m），默认 0.50')
    ap.add_argument('--occupancy', type=float, default=0.70,
                    help='判定“持续性”的最小帧占用率，默认 0.70')
    args = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))

    frames = []
    first = None
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, LaserScan)
        if first is None:
            first = m
        frames.append(m)
    if not frames:
        raise SystemExit('no /scan messages')

    n = len(frames)
    beams = len(first.ranges)
    inc, amin = first.angle_increment, first.angle_min
    print(f'frames={n} beams={beams} '
          f'angle=[{math.degrees(amin):+.1f},'
          f'{math.degrees(amin + (beams - 1) * inc):+.1f}] deg '
          f'inc={math.degrees(inc):.4f} deg')

    for lim in (0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50):
        cnt = [sum(1 for r in m.ranges if math.isfinite(r) and 0 < r < lim)
               for m in frames]
        print(f'  per-frame <{lim:.2f} m: mean={statistics.mean(cnt):.1f} '
              f'min={min(cnt)} max={max(cnt)}')

    persistent = []
    for i in range(beams):
        vals = [m.ranges[i] for m in frames
                if math.isfinite(m.ranges[i]) and
                0 < m.ranges[i] < args.close_range]
        if not vals:
            continue
        frac = len(vals) / n
        if frac >= args.occupancy:
            persistent.append((i, frac, statistics.median(vals),
                               min(vals), max(vals)))

    print(f'\npersistent close beams (>= {args.occupancy * 100:.0f}% frames '
          f'finite < {args.close_range:.2f} m): {len(persistent)}')
    if not persistent:
        print('  none (area clear / self-echo below top range)')
        return

    start = prev = persistent[0][0]
    group = [persistent[0]]
    sectors = []
    for item in persistent[1:]:
        if item[0] == prev + 1:
            group.append(item)
        else:
            sectors.append((start, prev, group))
            start = prev = item[0]
            group = [item]
        prev = item[0]
    sectors.append((start, prev, group))

    for s, e, group in sectors:
        med = statistics.median([g[2] for g in group])
        mx = max(g[4] for g in group)
        frac = statistics.mean([g[1] for g in group])
        print(f'  sector beams {s:3d}..{e:3d} '
              f'angle {math.degrees(amin + s * inc):+7.1f}..'
              f'{math.degrees(amin + e * inc):+7.1f} deg '
              f'n={len(group):3d} median={med:.3f} m max={mx:.3f} m '
              f'occupancy={frac * 100:.0f}%')

    mx = max(g[4] for g in persistent)
    print(f'\npersistent close max range = {mx:.3f} m')
    print(f'suggested lower_threshold >= {mx + 0.02:.3f} m (plus margin)')


if __name__ == '__main__':
    main()
