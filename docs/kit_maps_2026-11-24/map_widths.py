#!/usr/bin/env python3
"""키트 지도의 급행·통로 폭(점유 칸 경계를 칸에 맞춰 읽음).

results/2026-11-23 analysis/map_runs.py 방식이다.

사용: python3 map_widths.py 이름=지도.yaml ...
줄마다 한 지도. 폭은 기준점이 든 빈칸 구간(점유가 아닌 칸: 빈칸과 미지)의 길이다.
"""
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def load(path):
    meta = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    img = np.array(Image.open(Path(path).parent / meta["image"]))
    occ = (255 - img.astype(float)) / 255.0 > meta["occupied_thresh"]
    if meta.get("negate", 0):
        occ = img.astype(float) / 255.0 > meta["occupied_thresh"]
    return occ[::-1], float(meta["resolution"]), meta["origin"][:2]  # 줄 0 = 아래(y 작은 쪽)


def free_run(occ, res, origin, axis, fixed, ref):
    """axis 'y': x=fixed 기둥에서 y=ref가 든 빈칸 구간 [lo, hi]. axis 'x': y=fixed 줄에서."""
    ox, oy = origin
    if axis == "y":
        line = occ[:, int(np.floor((fixed - ox) / res))]
        k, start = int(np.floor((ref - oy) / res)), oy
    else:
        line = occ[int(np.floor((fixed - oy) / res)), :]
        k, start = int(np.floor((ref - ox) / res)), ox
    if line[k]:
        return None
    lo = k
    while lo > 0 and not line[lo - 1]:
        lo -= 1
    hi = k
    while hi < len(line) - 1 and not line[hi + 1]:
        hi += 1
    return start + lo * res, start + (hi + 1) * res


# (이름, 축, 고정 좌표, 기준점, 줄자 m)
ROWS = [
    ("급행 x=0.8", "y", 0.8, 0.868, 0.198),
    ("급행 x=1.1", "y", 1.1, 0.868, 0.198),
    ("급행 x=1.4", "y", 1.4, 0.868, 0.198),
    ("가운데 통로 x=0.8", "y", 0.8, 0.58, 0.36),
    ("가운데 통로 x=1.4", "y", 1.4, 0.58, 0.36),
    ("남쪽 통로 x=0.8", "y", 0.8, 0.20, 0.36),
    ("남쪽 통로 x=1.4", "y", 1.4, 0.20, 0.36),
    ("가운데 칸막이 틈 y=0.385", "x", 0.385, 1.12, 0.25),
    ("진열대 A 서쪽 끝~서쪽 벽 y=0.79", "x", 0.79, 0.30, 0.65),
    ("진열대 A 서쪽 끝~서쪽 벽 y=0.772", "x", 0.772, 0.30, 0.65),
    ("칸막이 서쪽 끝~서쪽 벽 y=0.385", "x", 0.385, 0.30, 0.70),
    ("진열대 A 동쪽 끝~동쪽 벽 y=0.79", "x", 0.79, 1.80, 0.42),
    ("진열대 A 동쪽 끝~동쪽 벽 y=0.772", "x", 0.772, 1.80, 0.42),
    ("칸막이 동쪽 끝~동쪽 벽 y=0.385", "x", 0.385, 1.80, 0.42),
    ("안쪽 세로 x=0.3", "y", 0.3, 0.50, 0.98),
    ("안쪽 가로 y=0.6", "x", 0.6, 1.00, 1.98),
]


def main(argv):
    maps = [(a.split("=", 1)[0], load(a.split("=", 1)[1])) for a in argv]
    print("| 어디 | 줄자 | " + " | ".join(name for name, _ in maps) + " |")
    print("|---|---|" + "---|" * len(maps))
    for label, axis, fixed, ref, tape in ROWS:
        cells = []
        for _, (occ, res, origin) in maps:
            run = free_run(occ, res, origin, axis, fixed, ref)
            cells.append("막힘" if run is None
                         else f"{run[1] - run[0]:.3f} ({run[0]:.3f}~{run[1]:.3f})")
        print(f"| {label} | {tape:.3f} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main(sys.argv[1:])
