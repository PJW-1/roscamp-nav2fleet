# 급행 판마다: AMCL 지도 북쪽 벽(라이더가 맞추는 벽)과 몸 사이, 급행 줄(y 0.868)에서 벗어난 양
import json, math, sys, yaml
FP = [(0.040, 0.021), (0.036, 0.031), (0.029, 0.056), (-0.029, 0.056), (-0.049, 0.049), (-0.071, 0.042), (-0.076, 0.037),
      (-0.076, -0.037), (-0.071, -0.042), (-0.049, -0.049), (-0.029, -0.056), (0.029, -0.056), (0.036, -0.031), (0.040, -0.021)]
m = yaml.safe_load(open(sys.argv[1]))
raw = open(sys.argv[1].rsplit("/", 1)[0] + "/" + m["image"] if "/" in sys.argv[1] else m["image"], "rb").read()
parts = raw.split(maxsplit=4); W, H = int(parts[1]), int(parts[2]); px = parts[4][-W * H:]
res, ox, oy = m["resolution"], m["origin"][0], m["origin"][1]
occ = lambda c, r: px[(H - 1 - r) * W + c] < 128
def wall_y(x):  # x 열에서 y 0.90 위 첫 막힌 칸의 아래 끝
    c = int((x - ox) / res)
    for r in range(int((0.90 - oy) / res), H):
        if occ(c, r): return oy + r * res
wy = {round(x, 2): wall_y(x) for x in [i / 100 for i in range(10, 195)]}
print("AMCL 지도 북쪽 벽 y:", {x: round(wy[x], 3) for x in (0.2, 0.3, 0.5, 0.8, 1.1, 1.4, 1.6, 1.7, 1.8)})
def gap(x, y, th):
    best = 9
    for fx, fy in FP:
        X, Y = x + fx * math.cos(th) - fy * math.sin(th), y + fx * math.sin(th) + fy * math.cos(th)
        w = wy.get(round(X, 2))
        if w: best = min(best, w - Y)
    return best
for path in sys.argv[2:]:
    rows = [json.loads(l) for l in open(path) if l.strip()]
    rows = [r for r in rows[1:] if isinstance(r, dict) and "t" in r]
    t0 = rows[0]["t"]
    print("==", path.split("/")[-1])
    by = {}
    for r in rows:
        if r.get("in") == "on_robot_state": by.setdefault(r["args"][0], []).append((r["t"] - t0, *r["args"][1:5]))
    for name, s in by.items():
        seg = []; n = 0
        for p in s + [(1e9, 0, 0, 0, "")]:
            if p[2] > 0.78 and p[0] < 1e9: seg.append(p); continue
            if len(seg) > 20:
                n += 1
                mv = [q for q in seg if q[4] == "MOVING"]
                g = min(mv, key=lambda q: gap(q[1], q[2], q[3]))
                turn = [q for q in mv if 1.45 < q[1] < 1.72 and abs(abs(q[3]) - math.pi) < 0.6]
                mid = [q for q in mv if 0.75 < q[1] < 1.40]
                odock = [q for q in seg if q[4] != "MOVING" and q[1] < 0.40]
                print(f"  {name} {n}번째 +{seg[0][0]:.0f}s: 몸-벽 최소 {100*gap(g[1],g[2],g[3]):.1f} cm at x {g[1]:.2f} yaw {math.degrees(g[3]):.0f}°"
                      f" | W_X 턴 뒤 북쪽 최대 {100*(max(q[2] for q in turn)-0.868):+.1f} cm (x {max(turn, key=lambda q:q[2])[1]:.2f})" if turn else "",
                      f"| 가운데 평균 {100*(sum(q[2] for q in mid)/len(mid)-0.868):+.1f} cm" if mid else "",
                      f"| O에 선 y {100*(odock[len(odock)//2][2]-0.868):+.1f} cm, 벽까지 {100*gap(*odock[len(odock)//2][1:4]):.1f}" if odock else "")
            seg = []
