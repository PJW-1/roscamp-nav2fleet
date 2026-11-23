#!/usr/bin/env python3
"""지도(pgm)에서 경기장 틈과 폭을 잰다. 점유 칸(값 < 128)의 경계로 잰다.
사용: python3 map_gaps.py 지도.yaml"""
import sys, yaml
from pathlib import Path

def load(path):
    meta = yaml.safe_load(open(path)); img = Path(path).parent / meta["image"]
    data = open(img, "rb").read()
    parts, i = [], 0
    while len(parts) < 4:   # P5 머리: 마법수, 가로, 세로, 최댓값(주석 건너뜀)
        while data[i:i+1].isspace(): i += 1
        if data[i:i+1] == b"#":
            while data[i:i+1] != b"\n": i += 1
            continue
        j = i
        while not data[j:j+1].isspace(): j += 1
        parts.append(data[i:j]); i = j
    w, h = int(parts[1]), int(parts[2]); px = data[i+1:]
    res = meta["resolution"]; ox, oy = meta["origin"][:2]
    occ = lambda x, y: px[(h - 1 - int((y - oy) / res)) * w + int((x - ox) / res)] < 128
    return occ, res

def run_y(occ, res, x, y0, y1):   # x 고정, y0에서 y1 쪽으로 첫 점유 칸의 가까운 면
    step = res if y1 > y0 else -res; y = y0
    while (y < y1) if step > 0 else (y > y1):
        if occ(x, y): return y if step > 0 else y + res
        y += step
    return None

def run_x(occ, res, y, x0, x1):
    step = res if x1 > x0 else -res; x = x0
    while (x < x1) if step > 0 else (x > x1):
        if occ(x, y): return x if step > 0 else x + res
        x += step
    return None

if __name__ != "__main__":
    pass
if __name__ == '__main__':
    occ, res = load(sys.argv[1])

    print(sys.argv[1], "res", res)
    for x in (0.8, 1.0, 1.4):
        n = run_y(occ, res, x, 0.88, 1.1); s = run_y(occ, res, x, 0.88, 0.6)
        print(f"  red   express width at x {x}: south face {s:.3f}, north wall {n:.3f} -> {n - s:.3f}")
    for x in (0.85, 1.4):
        a = run_y(occ, res, x, 0.6, 0.85); p = run_y(occ, res, x, 0.6, 0.3)
        p2 = run_y(occ, res, x, 0.2, 0.39); sw = run_y(occ, res, x, 0.2, -0.1)
        print(f"  black at x {x}: shelf A south face {a:.3f} - partition north {p:.3f} = {a - p:.3f}; "
              f"partition south {p2:.3f} - south wall {sw:.3f} = {p2 - sw:.3f}")
    for name, y in (("express wall", None), ("partition", None)):
        pass
    # 진열대 A(급행 벽)와 가운데 칸막이의 y를 찾는다
    ya = (run_y(occ, res, 1.0, 0.88, 0.6) + run_y(occ, res, 1.0, 0.6, 0.85)) / 2 - res / 2
    yp = (run_y(occ, res, 0.85, 0.6, 0.3) + run_y(occ, res, 0.85, 0.2, 0.39)) / 2 - res / 2
    ew = run_x(occ, res, ya, 2.2, 1.0); we = run_x(occ, res, ya, -0.2, 1.0)
    e_end = run_x(occ, res, ya, 1.0, 2.2); w_end = run_x(occ, res, ya, 1.0, -0.2)
    print(f"  express wall at y {ya:.3f}: x {w_end:.3f}..{e_end:.3f}")
    # 같은 줄에서 바깥벽 면: 벽 끝에서 더 나가 다음 점유
    east_wall = run_x(occ, res, ya, e_end + 0.02, 2.2); west_wall = run_x(occ, res, ya, w_end - 0.02, -0.2)
    print(f"  green (express wall east end -> east wall) {east_wall - e_end:.3f}; blue (west end -> west wall) {w_end - west_wall:.3f}")
    p_e = run_x(occ, res, yp, 1.4, 2.2); p_w = run_x(occ, res, yp, 0.85, -0.2)
    g_w = run_x(occ, res, yp, 0.85, 1.4); g_e = run_x(occ, res, yp, 1.4, 0.85)
    ew2 = run_x(occ, res, yp, p_e + 0.02, 2.2); ww2 = run_x(occ, res, yp, p_w - 0.02, -0.2)
    print(f"  partition at y {yp:.3f}: west piece x {p_w:.3f}..{g_w:.3f}, east piece {g_e:.3f}..{p_e:.3f}")
    print(f"  green (partition east end -> east wall) {ew2 - p_e:.3f}; orange gap {g_e - g_w:.3f}; purple (west end -> west wall) {p_w - ww2:.3f}")
