import math, re, sys
exec(open("analyze_cause.py").read().split("# ---- odom -> 지도")[0])
tot = under = 0
for t0, r in seg_out:
    d = []
    for t, m in data[(r, "cmd_vel")]:
        m = tw(m)
        if t0 - 1.5 <= t < t0 and abs(m.linear.x) < 0.01 and m.angular.z < -0.05:
            lp = at(r, "lookahead_point", t)[1].point; d.append(round(math.hypot(lp.x, lp.y), 3))
    tot += len(d); under += sum(1 for x in d if x < 0.05)
    print(r, round(t0 % 1000, 1), "오른쪽 회전 명령", len(d), "carrot 거리", d[:3], "...", d[-2:])
print(f"0.05 m 미만 {under}/{tot}")
