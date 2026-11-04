#!/usr/bin/env python3
"""分析 rosbag 中 /map 的建图稳定性。

指标：
  · 每帧占据格数量；
  · 相邻地图在共同覆盖区域内的 occupied Jaccard；
  · 上一帧 occupied、当前帧 known-free 的“消失格”；
  · 最终地图占据格在历史地图中的保持率。

用法：
  python3 tools/map_stability.py --bag /home/pi/rosbags/xxx
  python3 tools/map_stability.py --bag ... --threshold 65 --csv /tmp/map_stability.csv
"""

import argparse
import csv
import math
import statistics

import rosbag2_py
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message


def cells_of(msg, threshold):
    w, h, res = msg.info.width, msg.info.height, msg.info.resolution
    ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
    occ, free = set(), set()
    for i, v in enumerate(msg.data):
        if v < 0:
            continue
        x = ox + (i % w + 0.5) * res
        y = oy + (i // w + 0.5) * res
        key = (round(x / res), round(y / res))
        if v >= threshold:
            occ.add(key)
        elif v == 0:
            free.add(key)
    return occ, free


def jaccard(a, b):
    if not a and not b:
        return 1.0
    return len(a & b) / len(a | b)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--topic', default='/map')
    parser.add_argument('--threshold', type=int, default=65,
                        help='OccupancyGrid 占据阈值（默认 65）')
    parser.add_argument('--csv', default=None)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    maps = []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != args.topic:
            continue
        try:
            maps.append(deserialize_message(raw, OccupancyGrid))
        except Exception:
            continue

    if not maps:
        print(f'bag 中没有 {args.topic}')
        raise SystemExit(1)

    print(f'maps={len(maps)} topic={args.topic}')
    prev = None
    rows = []
    jac_values = []
    for idx, m in enumerate(maps):
        occ, free = cells_of(m, args.threshold)
        known = occ | free
        print(f'[{idx:02d}] occupied={len(occ):5d} free={len(free):5d} '
              f'origin=({m.info.origin.position.x:+.2f},'
              f'{m.info.origin.position.y:+.2f}) '
              f'size={m.info.width}x{m.info.height}')
        row = {'map': idx, 'occupied': len(occ), 'free': len(free),
               'jaccard': '', 'lost': '', 'gained': ''}
        if prev is not None:
            p_occ, p_free = prev
            common = (p_occ | p_free) & known
            j = jaccard(p_occ & common, occ & common)
            # 只统计共同 known 区域内的翻转，避免地图范围变化误报。
            lost = len((p_occ & common) - occ)
            gained = len((occ & common) - p_occ)
            jac_values.append(j)
            row.update(jaccard=round(j, 4), lost=lost, gained=gained)
            print(f'     common={len(common)} jaccard={j:.4f} '
                  f'lost_occ={lost} gained_occ={gained}')
        rows.append(row)
        prev = (occ, free)

    # 最终地图占据格在历史 known 地图中的保持率。
    final_occ, _ = cells_of(maps[-1], args.threshold)
    stable = 0
    tracked = 0
    for i, m in enumerate(maps[:-1]):
        occ, free = cells_of(m, args.threshold)
        known = occ | free
        for c in final_occ & known:
            tracked += 1
            if c in occ:
                stable += 1
    rate = stable / tracked if tracked else float('nan')
    print(f'\nfinal_occupied={len(final_occ)} history_tracked={tracked} '
          f'stable_occupied={stable} retention={rate:.4f}')
    if jac_values:
        print(f'jaccard mean={statistics.fmean(jac_values):.4f} '
              f'min={min(jac_values):.4f}')

    if args.csv:
        with open(args.csv, 'w', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=['map', 'occupied', 'free',
                                                    'jaccard', 'lost', 'gained'])
            writer.writeheader()
            writer.writerows(rows)
        print(f'csv: {args.csv}')


if __name__ == '__main__':
    main()
