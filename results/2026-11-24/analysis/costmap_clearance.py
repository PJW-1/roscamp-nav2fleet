#!/usr/bin/env python3
"""PGM 지도에서 한 점부터 가장 가까운 막힌 칸(값 < 100) 가장자리까지 거리를 낸다. 읽기만 한다.
사용: costmap_clearance.py <pgm> <해상도> <origin_x> <origin_y> <x> <y>"""
import math
import sys

path, res, ox, oy, x, y = sys.argv[1], *map(float, sys.argv[2:7])
hdr = open(path, "rb").read().split(maxsplit=4)
w, h = int(hdr[1]), int(hdr[2])
data = hdr[4][-w * h:]
best = min(math.hypot(x - (ox + (i % w + 0.5) * res), y - (oy + (h - 1 - i // w + 0.5) * res))
           for i, v in enumerate(data) if v < 100)
print(f"{path}: ({x}, {y}) nearest occupied cell edge {best - res / 2:.3f} m")
