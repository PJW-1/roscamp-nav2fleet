#!/usr/bin/env python3
"""PGM 지도에서 벽 안쪽 막힌 칸 덩어리의 경계 상자를 경기장 좌표(m)로 낸다."""
import sys
def read_pgm(path):
    data = open(path, "rb").read()
    toks, i = [], 0
    while len(toks) < 4:
        while data[i:i+1].isspace(): i += 1
        if data[i:i+1] == b"#":
            while data[i:i+1] != b"\n": i += 1
            continue
        j = i
        while not data[j:j+1].isspace(): j += 1
        toks.append(data[i:j]); i = j
    i += 1
    w, h = int(toks[1]), int(toks[2])
    return w, h, data[i:i+w*h]
path, res, ox, oy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
w, h, px = read_pgm(path)
occ = lambda c, r: px[r*w + c] < 100  # r: 위에서부터
seen = set(); boxes = []
for r in range(h):
    for c in range(w):
        if not occ(c, r) or (c, r) in seen: continue
        stack = [(c, r)]; seen.add((c, r)); cells = []
        while stack:
            a, b = stack.pop(); cells.append((a, b))
            for da, db in ((1,0),(-1,0),(0,1),(0,-1)):
                n = (a+da, b+db)
                if 0 <= n[0] < w and 0 <= n[1] < h and n not in seen and occ(*n):
                    seen.add(n); stack.append(n)
        cs = [a for a, _ in cells]; rs = [b for _, b in cells]
        x0 = ox + min(cs)*res; x1 = ox + (max(cs)+1)*res
        y0 = oy + (h-1-max(rs))*res; y1 = oy + (h-max(rs)+ (max(rs)-min(rs)))*res
        boxes.append((len(cells), round(x0,3), round(x1,3), round(y0,3), round(y1,3)))
boxes.sort(key=lambda b: -b[0])
print(f"{path.split('/')[-1]} {w}x{h} res {res}")
for n, x0, x1, y0, y1 in boxes[:12]:
    print(f"  cells {n:6d}  x {x0:6.3f}..{x1:6.3f}  y {y0:6.3f}..{y1:6.3f}")
