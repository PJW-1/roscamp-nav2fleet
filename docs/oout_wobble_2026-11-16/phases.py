import math, re, sys
from collections import Counter
src = open("analyze_cause.py").read()
exec(src.split("# ---- odom -> 지도")[0])
exec("def hits" + src.split("def hits")[1].split("def in_out_window")[0])
STEP = 2 * math.sin(0.0125 / RC)
print(f"회전 한 걸음 {math.degrees(STEP):.1f}deg, 투영이 생기는 최소 |w| = {STEP / 0.6:.3f} rad/s")
ph = Counter(); col_w = []; free_w = []; xs = []; per = []
for t0, r in seg_out:
    cmd = [(t, tw(m)) for t, m in data[(r, "cmd_vel")] if t0 - 1.5 <= t <= t0 + 10]
    go = next((t for t, m in cmd if t > t0 and m.linear.x > 0.03), t0 + 10)
    spin = [(t, m.angular.z) for t, m in cmd if abs(m.linear.x) < 0.01 and abs(m.angular.z) > 0.05]
    pre = [w for t, w in spin if t < t0 + 0.1]
    post = [(t, w) for t, w in spin if t0 + 0.1 <= t < go]
    f_start = sum(1 for (ta, a), (tb, b) in zip(spin, spin[1:]) if a * b < 0 and tb < go)
    f_drive = sum(1 for (ta, a), (tb, b) in zip(spin, spin[1:]) if a * b < 0 and tb >= go)
    # 기준 자세: 반시계 회전을 시작하는 첫 명령 때 = 고정 경로 끝 자세(남쪽)로 둔다
    t_ref = post[0][0] if post else t0
    o = at(r, "odom", t_ref)[1].pose.pose
    ox_, oy_, oyaw = o.position.x, o.position.y, yaw_of(o.orientation)
    psi = -math.pi / 2 - oyaw
    turn = math.degrees(wrap(yaw_of(at(r, "odom", go)[1].pose.pose.orientation) - oyaw))
    per.append((r, round(t0 % 1000, 1), "오른쪽" if pre and pre[-1] < 0 else ("왼쪽" if pre else "-"),
                "왼쪽" if post and post[0][1] > 0 else ("오른쪽" if post else "-"), round(turn), round(go - t0, 1), f_start, f_drive))
    ph["출발 전 바뀜"] += f_start; ph["달리기 시작 뒤 바뀜"] += f_drive
    # 이 출고 창의 collision: 직전 명령 크기와 걸린 칸(O_OUT 기준 지도)
    for tc, rr in cols:
        if rr != r or not (t0 <= tc <= go + 0.5):
            continue
        prev = [tw(m).angular.z for t, m in data[(r, "cmd_vel")] if tc - 0.08 <= t < tc - 0.01]
        cmr = at(r, "local_costmap/costmap", tc); od = at(r, "odom", tc)
        p = od[1].pose.pose; x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        hh = hits(cmr[1], x, y, yaw)
        if not hh:
            om = abs(od[1].twist.twist.angular.z); w = min(1.0, om + 0.16)
            if w > STEP / 0.6:
                hh = hits(cmr[1], x, y, yaw + math.copysign(STEP, od[1].twist.twist.angular.z or 1))
        for hx, hy in hh:
            dx, dy = hx - ox_, hy - oy_
            xs.append(0.15 + math.cos(psi) * dx - math.sin(psi) * dy)
    for (ta, a), (tb, b) in zip(cmd, cmd[1:]):
        if t0 + 0.1 <= tb < go and abs(a.linear.x) < 0.01:
            nxt_col = any(rr == r and abs(tc - tb) < 0.02 for tc, rr in cols)
            if abs(a.angular.z) > 0.05:
                (col_w if nxt_col else free_w).append(abs(at(r, "odom", tb)[1].twist.twist.angular.z) + 0.16)
print("출고별: 로봇, 시각, 고정경로 끝 회전, Nav2 첫 회전, 출발까지 돈 각(deg), 출발까지 s, 출발 전 바뀜, 달린 뒤 바뀜")
for p in per: print("  ", p)
print(dict(ph))
def q(v):
    v = sorted(v); return f"n={len(v)} 중앙 {v[len(v)//2]:.2f} 범위 {v[0]:.2f}~{v[-1]:.2f}" if v else "-"
print("다음 주기에 collision이 난 때의 예상 명령(odom|w|+0.16, 1.0 상한):", q([min(1, x) for x in col_w]))
print("다음 주기에 collision이 안 난 때:", q([min(1, x) for x in free_w]))
print(f"걸린 칸 x(O_OUT 기준 지도): {len(xs)}칸, 0.04 미만 {sum(1 for x in xs if x < 0.04)}, 분포", sorted(Counter(round(x, 2) for x in xs).items()))
