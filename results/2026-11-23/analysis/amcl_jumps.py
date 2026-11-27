# 보고 위치(map->base TF)에서 AMCL이 고친 몫을 뽑는다: 로봇 기준 옆 이동, 앞 이동 튐, 방향 튐
import json, math, sys, statistics as st
def ang(a): return (a + math.pi) % (2 * math.pi) - math.pi
def region(x, y):
    if y > 0.80 and 0.45 < x < 1.70: return "급행 곧은 길"
    if y > 0.78: return "급행 양 끝(돌기)"
    return "나머지"
out = {}
for path in sys.argv[1:]:
    rows = [json.loads(l) for l in open(path) if l.strip()]
    rows = [r for r in rows[1:] if isinstance(r, dict) and "t" in r and r.get("in") == "on_robot_state"]
    by = {}
    for r in rows:
        a = r["args"]; by.setdefault(a[0], []).append((r["t"], a[1], a[2], a[3], a[4]))
    for name, s in by.items():
        for i in range(2, len(s) - 2):
            (t0, x0, y0, h0, s0), (t1, x1, y1, h1, s1) = s[i], s[i + 1]
            if not (0.05 < t1 - t0 < 0.2) or s0 != "MOVING" or s1 != "MOVING": continue
            dx, dy = x1 - x0, y1 - y0
            if math.hypot(dx, dy) > 0.15: continue          # 큰 튐(재위치)은 따로
            hm = h0 + ang(h1 - h0) / 2
            dl = -math.sin(hm) * dx + math.cos(hm) * dy
            # 앞 이동 튐: 이웃 4칸 앞 이동의 가운데값과 비교
            fw = [math.cos(s[j][3]) * (s[j + 1][1] - s[j][1]) + math.sin(s[j][3]) * (s[j + 1][2] - s[j][2]) for j in (i - 2, i - 1, i + 1)]
            df = math.cos(hm) * dx + math.sin(hm) * dy - st.median(fw)
            dh = [ang(s[j + 1][3] - s[j][3]) for j in (i - 2, i - 1, i + 1)]
            dth = ang(h1 - h0) - st.median(dh)
            reg = region((x0 + x1) / 2, (y0 + y1) / 2)
            out.setdefault(reg, []).append((abs(dl), abs(df), abs(dth), dy if reg == "급행 곧은 길" else 0, math.hypot(dx, dy)))
for reg, v in out.items():
    dist = sum(e[4] for e in v)
    lat = [e[0] for e in v if e[0] > 0.004]
    fwd = [e[1] for e in v if e[1] > 0.006]
    yaw = [e[2] for e in v if e[2] > math.radians(1.5)]
    print(f"{reg}: 칸 {len(v)}, 간 거리 {dist:.1f} m")
    print(f"   옆 고침 >4 mm: {len(lat)}번 ({len(lat)/dist:.1f}/m), 가운데 {1000*st.median(lat) if lat else 0:.1f} mm, 90% {1000*sorted(lat)[int(.9*len(lat))] if lat else 0:.1f}, 최대 {1000*max(lat) if lat else 0:.1f} mm")
    print(f"   앞 고침 >6 mm: {len(fwd)}번 ({len(fwd)/dist:.1f}/m), 가운데 {1000*st.median(fwd) if fwd else 0:.1f} mm, 최대 {1000*max(fwd) if fwd else 0:.1f} mm")
    print(f"   방향 고침 >1.5°: {len(yaw)}번 ({len(yaw)/dist:.1f}/m), 최대 {math.degrees(max(yaw)) if yaw else 0:.1f}°")
    if reg == "급행 곧은 길":
        n = [e[3] for e in v if e[0] > 0.004]
        print(f"   옆 고침 방향: 북쪽 {sum(1 for d in n if d > 0)}번, 남쪽 {sum(1 for d in n if d < 0)}번")
