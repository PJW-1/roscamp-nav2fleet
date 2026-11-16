import math, re, sys
src = open("analyze_cause.py").read()
exec(src.split("# ---- odom -> 지도")[0])
STEP = 2 * math.sin(0.0125 / RC)
right_d, right_b, ests, n_all = [], [], [], 0
for t0, r in seg_out:
    cmd = [(t, tw(m)) for t, m in data[(r, "cmd_vel")] if t0 - 1.5 <= t <= t0 + 10]
    go = next((t for t, m in cmd if t > t0 and m.linear.x > 0.03), t0 + 10)
    # (4) 고정 경로 끝 오른쪽 회전 동안 carrot 거리와 방위
    for t, m in cmd:
        if t0 - 1.5 <= t < t0 and abs(m.linear.x) < 0.01 and m.angular.z < -0.05:
            lp = at(r, "lookahead_point", t)[1].point
            right_d.append(math.hypot(lp.x, lp.y)); right_b.append(math.degrees(math.atan2(lp.y, lp.x)))
    # (2) 출발 전 모든 collision: 부호 있는 예상 명령 = clamp(sign(carrot)*1.0, w-0.16, w+0.16)
    for tc, rr in cols:
        if rr != r or not (t0 + 0.1 <= tc <= go):
            continue
        n_all += 1
        w = at(r, "odom", tc)[1].twist.twist.angular.z
        lp = at(r, "lookahead_point", tc)[1].point
        sgn = 1.0 if math.atan2(lp.y, lp.x) > 0 else -1.0
        ests.append(abs(min(max(sgn * 1.0, w - 0.16), w + 0.16)))
def q(v): v = sorted(v); return f"n={len(v)} 최소 {v[0]:.3f} 중앙 {v[len(v)//2]:.3f} 최대 {v[-1]:.3f}"
print("고정 경로 끝 오른쪽 회전 중 carrot 거리 m:", q(right_d))
print("  같은 때 carrot 방위 deg:", q(right_b))
print(f"출발 전 collision 전부 {n_all}건, 부호 있는 예상 |명령|:", q(ests), f"/ {STEP/0.6:.3f} 미만 {sum(1 for e in ests if e < STEP/0.6)}건")
