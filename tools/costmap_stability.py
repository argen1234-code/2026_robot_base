#!/usr/bin/env python3
"""Measure lethal-cell stability of a published costmap.

用法（在 ros_jazzy 环境、domain 1）：
    python3 tools/costmap_stability.py                 # /costmap, 6 s
    python3 tools/costmap_stability.py --seconds 10

输出：致命格数量范围、质心标准差、相邻帧 Jaccard。Jaccard 越接近 1、
质心标准差越小，说明静态障碍越稳。
"""

import argparse
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/costmap')
    parser.add_argument('--seconds', type=float, default=6.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('costmap_stability')
    frames = []
    node.create_subscription(OccupancyGrid, args.topic, frames.append, 10)
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if not frames:
        print('no costmap frames')
        raise SystemExit(1)

    m = frames[-1]
    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    vals = sorted(set(m.data))
    print(f'frames={len(frames)} size={w}x{h} res={res:.3f} '
          f'origin=({ox:+.3f},{oy:+.3f})')
    print(f'values in last frame: {vals[:15]}{"..." if len(vals) > 15 else ""}')
    lethal_value = vals[-1] if vals else 0

    def lethal_set(msg):
        cells = []
        for i, v in enumerate(msg.data):
            if v == lethal_value and v > 0:
                x = ox + (i % w + 0.5) * res
                y = oy + (i // w + 0.5) * res
                cells.append((i, x, y))
        return cells

    sets = [lethal_set(msg) for msg in frames]
    origins = [(msg.info.origin.position.x, msg.info.origin.position.y)
               for msg in frames]
    print('origin span x=%.4f y=%.4f' % (
        max(o[0] for o in origins) - min(o[0] for o in origins),
        max(o[1] for o in origins) - min(o[1] for o in origins)))

    counts = [len(s) for s in sets]
    print(f'lethal({lethal_value}) count: min={min(counts)} max={max(counts)} '
          f'mean={sum(counts) / len(counts):.1f}')

    cxs = [sum(c[1] for c in s) / len(s) for s in sets if s]
    cys = [sum(c[2] for c in s) / len(s) for s in sets if s]
    if cxs:
        mx, my = sum(cxs) / len(cxs), sum(cys) / len(cys)
        sx = math.sqrt(sum((x - mx) ** 2 for x in cxs) / len(cxs))
        sy = math.sqrt(sum((y - my) ** 2 for y in cys) / len(cys))
        print(f'centroid mean=({mx:+.4f},{my:+.4f}) '
              f'std=({sx * 1000:.2f},{sy * 1000:.2f}) mm')

    def jaccard(a, b):
        sa, sb = {c[0] for c in a}, {c[0] for c in b}
        union = sa | sb
        return len(sa & sb) / len(union) if union else 1.0

    ref = sets[0]
    jac = [jaccard(ref, s) for s in sets[1:]]
    cons = [jaccard(a, b) for a, b in zip(sets, sets[1:])]
    if jac:
        print(f'Jaccard vs frame0: min={min(jac):.3f} '
              f'mean={sum(jac) / len(jac):.3f} max={max(jac):.3f}')
    if cons:
        print(f'consecutive Jaccard: min={min(cons):.3f} '
              f'mean={sum(cons) / len(cons):.3f} max={max(cons):.3f}')


if __name__ == '__main__':
    main()
