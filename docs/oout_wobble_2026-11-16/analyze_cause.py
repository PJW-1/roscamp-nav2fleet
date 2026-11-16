"""O_OUT 원인 확인(녹화 bag 재계산).
1) collision ahead: RPP 검사를 같은 식으로 다시 계산(덧댄 발자국, 지금 자세 + 회전 투영 0.6 s).
   걸린 칸을 지도 좌표로 바꿔 어디인지 본다.
2) 회전 방향 바뀜: 그 직전/직후 carrot(lookahead_point, 로봇 기준)의 방위."""
import bisect
import csv
import math
import re
import sys
from collections import Counter, defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

S = sys.argv[1]
FOOT = [(0.040, 0.021), (0.036, 0.031), (0.029, 0.056), (-0.029, 0.056), (-0.049, 0.049), (-0.071, 0.042),
        (-0.076, 0.037), (-0.076, -0.037), (-0.071, -0.042), (-0.049, -0.049), (-0.029, -0.056),
        (0.029, -0.056), (0.036, -0.031), (0.040, -0.021)]
PAD = 0.01  # local costmap footprint_padding 기본값
PF = [(x + math.copysign(PAD, x), y + math.copysign(PAD, y)) for x, y in FOOT]
RC = max(math.hypot(x, y) for x, y in PF)
O_OUT = (0.15, 0.62)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---- 로그 ----
log = open(f"{S}/launch.log", encoding="utf-8", errors="replace").read().splitlines()
ts = lambda line: float(re.search(r"\[(\d{10}\.\d+)\]", line).group(1))
seg_out, seg_ex, redirect = [], [], []
for i, line in enumerate(log):
    m = re.search(r"SEGMENT_DONE (pode\d)-\d+ seq \d+ \(follow_path\) to (\S+)", line)
    if m and m.group(2) == "O_OUT":
        seg_out.append((ts(line), m.group(1)))
    elif m and m.group(2).startswith("E-"):
        seg_ex.append((ts(line), m.group(1)))
    if "STANDBY_LEFT" in line and i + 1 < len(log):
        n = re.search(r"ORDER_SENT (pode\d)", log[i + 1])
        if n:
            redirect.append((ts(line), n.group(1)))
cols = [(ts(l), re.search(r"\[(pode\d)\.controller_server\]", l).group(1)) for l in log
        if "detected collision ahead" in l]

# ---- bag ----
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=f"{S}/bag", storage_id=""), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
want = re.compile(r"/(pode\d)/(odom|cmd_vel|lookahead_point|local_costmap/costmap)$")
data = defaultdict(list)
while reader.has_next():
    topic, raw, t = reader.read_next()
    m = want.match(topic)
    if m:
        data[(m.group(1), m.group(2))].append((t * 1e-9, deserialize_message(raw, get_message(types[topic]))))
times = {k: [t for t, _ in v] for k, v in data.items()}


def at(robot, kind, t):
    i = bisect.bisect_right(times[(robot, kind)], t) - 1
    return data[(robot, kind)][i] if i >= 0 else None


def after(robot, kind, t):
    i = bisect.bisect_right(times[(robot, kind)], t)
    return data[(robot, kind)][i] if i < len(data[(robot, kind)]) else None


def tw(m):
    return m.twist if hasattr(m, "twist") else m


# ---- odom -> 지도: 시작점(robots.csv 0초) + O_OUT 도착으로 회전각 ----
spawn = {}
with open(f"{S}/robots.csv") as f:
    for r in csv.DictReader(f):
        if r["robot"] not in spawn:  # 처음 기록된 자리(pode3은 6.46초에 첫 기록)
            spawn[r["robot"]] = (float(r["x"]), float(r["y"]))
th_s = defaultdict(list)
for t0, r in seg_out:
    od = at(r, "odom", t0)[1].pose.pose.position
    th_s[r].append((wrap(math.atan2(O_OUT[1] - spawn[r][1], O_OUT[0] - spawn[r][0]) - math.atan2(od.y, od.x)),
                    math.hypot(od.x, od.y) / math.hypot(O_OUT[0] - spawn[r][0], O_OUT[1] - spawn[r][1])))
theta = {}
print("odom->지도 회전각(시작점 기준, O_OUT 도착으로 추정)")
for r, v in sorted(th_s.items()):
    theta[r] = math.atan2(sum(math.sin(a) for a, _ in v), sum(math.cos(a) for a, _ in v))
    print(f"  {r}: {math.degrees(theta[r]):.1f}deg (도착 {len(v)}번, 각 폭 "
          f"{math.degrees(max(a for a, _ in v) - min(a for a, _ in v)):.1f}deg, 거리비 "
          f"{min(s for _, s in v):.2f}~{max(s for _, s in v):.2f})")


def to_map(r, x, y):
    c, s = math.cos(theta[r]), math.sin(theta[r])
    return spawn[r][0] + c * x - s * y, spawn[r][1] + s * x + c * y


def hits(cm, x, y, yaw):
    info = cm.info
    res, ox, oy, w = info.resolution, info.origin.position.x, info.origin.position.y, info.width
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(x + c * a - s * b, y + s * a + c * b) for a, b in PF]
    out = set()
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
        n = max(1, int(math.hypot(x2 - x1, y2 - y1) / 0.004))
        for k in range(n + 1):
            i = int((x1 + (x2 - x1) * k / n - ox) / res)
            j = int((y1 + (y2 - y1) * k / n - oy) / res)
            if 0 <= i < w and 0 <= j < info.height and cm.data[j * w + i] == 100:
                out.add((ox + (i + 0.5) * res, oy + (j + 0.5) * res))
    return out


def in_out_window(r, t):
    ws = [t0 for t0, rr in seg_out + redirect if rr == r and t0 - 0.5 <= t <= t0 + 10]
    return bool(ws)


# ---- 1) collision 재계산 ----
print(f"\n[1] collision ahead 재계산 (덧댄 발자국, 원 반경 {RC:.3f} m)")
cat = Counter()
mapx, mapy = [], []
for tc, r in cols:
    if not in_out_window(r, tc):
        continue
    od = at(r, "odom", tc)
    seq = [(t, m) for t, m in data[(r, "cmd_vel")] if tc - 0.5 <= t <= tc and abs(tw(m).angular.z) > 0.05]
    found = None
    for pick in (at, after):  # 그 순간 직전과 직후 costmap 둘 다
        cmr = pick(r, "local_costmap/costmap", tc)
        if not (od and cmr):
            continue
        cm = cmr[1]
        p = od[1].pose.pose
        x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        hh = hits(cm, x, y, yaw)
        if hh:
            found = ("지금 자세", hh)
            break
        if seq:
            v, w = tw(seq[-1][1]).linear.x, tw(seq[-1][1]).angular.z
            step = 2 * math.sin((cm.info.resolution / 2) / RC)
            pt = step / abs(w) if abs(v) < 0.01 else cm.info.resolution / max(abs(v), 1e-3)
            i, cx, cy, ct = 1, x, y, yaw
            while i * pt < 0.6:
                i += 1
                cx += pt * v * math.cos(ct)
                cy += pt * v * math.sin(ct)
                ct += pt * w
                hh = hits(cm, cx, cy, ct)
                if hh:
                    found = (f"투영 {i - 1}번째", hh)
                    break
            if found:
                break
    if not found:
        cat["재현 안 됨"] += 1
        continue
    kind, hh = found
    cat[kind] += 1
    for hx, hy in hh:
        mx, my = to_map(r, hx, hy)
        mapx.append(mx)
        mapy.append(my)
for k, v in cat.most_common():
    print(f"  {k}: {v}")
if mapx:
    west = sum(1 for x in mapx if x < 0.04)
    print(f"  걸린 칸(지도) x {min(mapx):.3f}~{max(mapx):.3f}, y {min(mapy):.3f}~{max(mapy):.3f}; "
          f"x<0.04(서쪽 벽 쪽) {west}/{len(mapx)}칸")
    hist = Counter(round(x, 2) for x in mapx)
    print("  x 분포:", ", ".join(f"{k:.2f}:{v}" for k, v in sorted(hist.items())))

# ---- 2) 회전 방향 바뀜과 carrot 방위 ----
print("\n[2] 제자리 회전 방향 바뀜 직전/직후 carrot 방위(로봇 기준, deg; 0=앞, +=왼쪽)")


def bearing(r, t):
    lp = at(r, "lookahead_point", t)
    if not lp:
        return None, None
    pnt = lp[1].point
    return math.degrees(math.atan2(pnt.y, pnt.x)), lp[1].header.frame_id


def uniq(evts):
    out = []
    for e in sorted(evts):
        if not any(u[1] == e[1] and abs(u[0] - e[0]) < 1.5 for u in out):
            out.append(e)
    return out


for name, evts in (("O_OUT", uniq(seg_out + redirect)), ("E-x", uniq(seg_ex))):
    flips = []
    first_b = []
    behind = total = 0
    frame = None
    for t0, r in evts:
        cmd = [(t, tw(m)) for t, m in data[(r, "cmd_vel")] if t0 <= t <= t0 + 10]
        go = next((t for t, m in cmd if m.linear.x > 0.03), t0 + 10)
        spin = [(t, m.angular.z) for t, m in cmd if t <= go and abs(m.linear.x) < 0.01 and abs(m.angular.z) > 0.05]
        b0, frame = bearing(r, t0 + 0.3)
        if b0 is not None:
            first_b.append(round(b0))
        for t, w in spin:
            b, _ = bearing(r, t)
            if b is not None:
                total += 1
                behind += abs(b) > 150
        for (ta, wa), (tb, wb) in zip(spin, spin[1:]):
            if wa * wb < 0:
                ba, _ = bearing(r, ta)
                bb, _ = bearing(r, tb)
                flips.append((r, round(tb - t0, 1), round(ba) if ba is not None else None,
                              round(bb) if bb is not None else None, round(tb - ta, 2)))
    print(f"  {name}: 전환 {len(evts)}번, carrot 틀 {frame}, 전환 0.3초 뒤 carrot 방위 {first_b}")
    print(f"    회전 명령 중 carrot이 뒤쪽(|방위|>150) {behind}/{total}")
    for f in flips:
        print(f"    바뀜 {f[0]} +{f[1]}s: 방위 {f[2]} -> {f[3]} (명령 간격 {f[4]}s)")
