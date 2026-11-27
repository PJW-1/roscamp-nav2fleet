# W 입구 굽은 길(Q2 -> Q1 -> W)에서 로봇이 경로 바깥(동쪽)으로 얼마나 밀렸나 (읽기만 한다)
import json, math, sys

def load(path):
    rows = [json.loads(l) for l in open(path) if l.strip()]
    rows = [r for r in rows[1:] if isinstance(r, dict) and "t" in r]
    return rows

def bend_path(rows):
    q1 = w = None
    for r in rows:
        if r.get("out") == "MoveOrderCmd":
            for s in r.get("base", []) + r.get("horizon", []):
                if s.get("mode") == "follow_path" and s.get("path"):
                    if s["end_node"] == "Q1" and q1 is None: q1 = [tuple(p[:2]) for p in s["path"]]
                    if s["end_node"] == "W" and w is None and s["path"][0][1] < 0.3: w = [tuple(p[:2]) for p in s["path"]]
    return (q1 or []) + (w[1:] if w else [])

def dev(P, x, y):
    best = (9, 1)
    for (ax, ay), (bx, by) in zip(P, P[1:]):
        dx, dy = bx - ax, by - ay; L = dx * dx + dy * dy
        if L == 0: continue
        u = max(0, min(1, ((x - ax) * dx + (y - ay) * dy) / L))
        d = math.hypot(x - ax - u * dx, y - ay - u * dy)
        if d < best[0]: best = (d, 1 if dx * (y - ay) - dy * (x - ax) >= 0 else -1)
    return best[0] * best[1]   # +: 왼쪽(안쪽), -: 오른쪽(바깥)

for path in sys.argv[1:]:
    rows = load(path); t0 = rows[0]["t"]; P = bend_path(rows)
    if len(P) < 4: print(path, "굽은 길 명령 없음"); continue
    st = {}
    for r in rows:
        if r.get("in") == "on_robot_state": a = r["args"]; st.setdefault(a[0], []).append((r["t"] - t0, a[1], a[2], a[3], a[4]))
    print("==", path.split("/")[-3] if path.endswith("fleet_record.jsonl") else path.split("/")[-1], "Q1", P[8] if len(P) > 8 else P[-1])
    for name, pts in sorted(st.items()):
        seg = []
        for p in pts + [(1e9, 0, 0, 0, "")]:
            if p[1] > 1.55 and p[2] < 0.5 and p[0] < 1e9: seg.append(p); continue
            if len(seg) > 10 and seg[0][2] < 0.25:
                after = [q for q in seg if 0.26 <= q[2] <= 0.47 and q[4] == "MOVING"]
                arc = [q for q in seg if q[1] > 1.70 and q[2] < 0.26 and q[4] == "MOVING"]
                if after and len(arc) > 3:
                    out = min(dev(P, q[1], q[2]) for q in after)
                    inn = max(dev(P, q[1], q[2]) for q in arc)
                    xmax = max(q[1] for q in after)
                    q1 = min(seg, key=lambda q: abs(q[2] - 0.26) + abs(q[1] - 1.849))
                    d = math.hypot(arc[-1][1] - arc[0][1], arc[-1][2] - arc[0][2])
                    turn = math.degrees(arc[-1][3] - arc[0][3]); dt = arc[-1][0] - arc[0][0]
                    v = sum(math.hypot(b[1]-a[1], b[2]-a[2]) for a, b in zip(arc, arc[1:])) / dt
                    w = math.radians(turn) / dt
                    print("  %s +%6.1f 호 안쪽 %+.1f cm, Q1 뒤 바깥 %+.1f cm, 최대 x %.3f (벽-몸 %.1f cm), Q1 yaw %.0f°, 호 v %.2f m/s w %.0f°/s 반지름 %.2f"
                          % (name, seg[0][0], inn * 100, out * 100, xmax, (1.98 - xmax - 0.056) * 100, math.degrees(q1[3]), v, math.degrees(w), v / w if w else 0))
            seg = []
