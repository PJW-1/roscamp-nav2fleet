#!/usr/bin/env python3
"""지도(pgm) 한 줄의 점유 구간(값 < 128)을 찍는다. 사용: python3 map_runs.py 지도.yaml"""
import sys, yaml
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))  # map_gaps.load
from map_gaps import load  # noqa
occ, res = load(sys.argv[1])
def runs(points):
    out, cur = [], None
    for v, o in points:
        if o and cur is None: cur = [v, v]
        elif o: cur[1] = v
        elif cur is not None: out.append(cur); cur = None
    if cur: out.append(cur)
    return " ".join(f"[{a:.3f}..{b + res:.3f}]" for a, b in out)
n = lambda a, b: [a + i * res for i in range(int(round((b - a) / res)))]
for x in (0.3, 0.8, 1.1, 1.4, 1.8):
    print(f"x={x}: y runs", runs([(y, occ(x, y)) for y in n(-0.05, 1.05)]))
for y in (0.79, 0.776, 0.385, 0.20, 0.60):
    print(f"y={y}: x runs", runs([(x, occ(x, y)) for x in n(-0.05, 2.05)]))
