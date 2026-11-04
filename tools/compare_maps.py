#!/usr/bin/env python3
"""比较多次离线回放保存的 map_saver PGM/YAML。

用法：
  python3 tools/compare_maps.py /tmp/map_r1.yaml /tmp/map_r2.yaml ...

输出：
  · occupied/free/unknown 像素数；
  · occupied 连通域数量、孤立小连通域（<=2 像素）、最大连通域；
  · 相对第一张图的 occupied Jaccard；
  · 多图 consensus（>=half 图占据）下每张图的 precision/recall；
  · 只在单张图出现的 unique occupied 格。
"""

import argparse
import collections
import math
import os

import yaml


def read_pgm(path):
    data = open(path, 'rb').read()
    idx = 0
    tokens = []
    w = h = maxv = None
    magic = None

    def next_token():
        nonlocal idx
        while idx < len(data) and data[idx] in b' \t\r\n':
            idx += 1
        if idx < len(data) and data[idx:idx + 1] == b'#':
            while idx < len(data) and data[idx] not in b'\r\n':
                idx += 1
            return next_token()
        start = idx
        while idx < len(data) and data[idx] not in b' \t\r\n':
            idx += 1
        return data[start:idx]

    magic = next_token()
    w = int(next_token())
    h = int(next_token())
    maxv = int(next_token())
    if data[idx:idx + 2] == b'\r\n':
        idx += 2
    elif idx < len(data) and data[idx] in b' \t\r\n':
        idx += 1
    pixels = data[idx:idx + w * h]
    if len(pixels) != w * h:
        raise ValueError(f'{path}: short PGM data {len(pixels)} != {w*h}')
    return magic, w, h, maxv, pixels


def occupied_pixels(pixels, w, h, threshold=50):
    occ = [[False] * w for _ in range(h)]
    free = [[False] * w for _ in range(h)]
    unknown = 0
    for r in range(h):
        row = pixels[r * w:(r + 1) * w]
        for c, v in enumerate(row):
            if v < threshold:
                occ[r][c] = True
            elif v > 250:
                free[r][c] = True
            else:
                unknown += 1
    return occ, free, unknown


def components(occ, w, h):
    seen = [[False] * w for _ in range(h)]
    sizes = []
    for r in range(h):
        for c in range(w):
            if not occ[r][c] or seen[r][c]:
                continue
            q = collections.deque([(r, c)])
            seen[r][c] = True
            n = 0
            while q:
                rr, cc = q.popleft()
                n += 1
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = rr + dr, cc + dc
                        if 0 <= nr < h and 0 <= nc < w and occ[nr][nc] \
                                and not seen[nr][nc]:
                            seen[nr][nc] = True
                            q.append((nr, nc))
            sizes.append(n)
    return sizes


def load_map(path):
    with open(path) as fh:
        meta = yaml.safe_load(fh)
    image = meta['image']
    image_path = image if os.path.isabs(image) else os.path.join(
        os.path.dirname(path), image)
    magic, w, h, maxv, pixels = read_pgm(image_path)
    occ, free, unknown = occupied_pixels(pixels, w, h)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    # 统一到 5 cm 世界格，便于不同 resolution 对比。
    common = 0.05
    occ_keys = set()
    free_keys = set()
    for r in range(h):
        for c in range(w):
            if not (occ[r][c] or free[r][c]):
                continue
            x = ox + (c + 0.5) * res
            y = oy + (h - r - 0.5) * res
            key = (math.floor(x / common), math.floor(y / common))
            if occ[r][c]:
                occ_keys.add(key)
            else:
                free_keys.add(key)
    sizes = components(occ, w, h)
    return {
        'path': path,
        'w': w, 'h': h, 'res': res,
        'occ_px': sum(sum(row) for row in occ),
        'free_px': sum(sum(row) for row in free),
        'unknown_px': unknown,
        'occ_keys': occ_keys,
        'free_keys': free_keys,
        'ncomp': len(sizes),
        'isolated': sum(1 for s in sizes if s <= 2),
        'largest': max(sizes) if sizes else 0,
    }


def jaccard(a, b):
    return len(a & b) / len(a | b) if (a or b) else 1.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('maps', nargs='+')
    args = parser.parse_args()
    maps = [load_map(p) for p in args.maps]
    base = maps[0]['occ_keys']
    all_occ = collections.Counter()
    for m in maps:
        all_occ.update(m['occ_keys'])
    n = len(maps)
    half = max(1, (n + 1) // 2)  # 4 张时 >=2，3 张时 >=2
    consensus = {k for k, v in all_occ.items() if v >= half}

    print(f'maps={n} consensus_cells={len(consensus)} (>= {half}/{n} maps)')
    print('name  occ_px free_px unknown comps isolated largest  '
          'jacc_vs_1 precision recall unique')
    for i, m in enumerate(maps):
        occ = m['occ_keys']
        inter = len(occ & consensus)
        precision = inter / len(occ) if occ else float('nan')
        recall = inter / len(consensus) if consensus else float('nan')
        unique = sum(1 for k in occ if all_occ[k] == 1)
        name = os.path.basename(m['path']).replace('.yaml', '')
        print(f'{name:8s} {m["occ_px"]:6d} {m["free_px"]:7d} '
              f'{m["unknown_px"]:7d} {m["ncomp"]:5d} {m["isolated"]:8d} '
              f'{m["largest"]:7d} {jaccard(base, occ):12.4f} '
              f'{precision:9.4f} {recall:6.4f} {unique:6d}')


if __name__ == '__main__':
    main()
