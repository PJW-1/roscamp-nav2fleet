# 출고 도크 O를 나설 때마다: O에 선 옆 오차, 30도 넘게 돈 자리, 몸 북쪽 끝과 SLAM 벽(0.965) 사이 (읽기만 한다)
# 쓰기: python3 o_exit.py ../runs/1123_ds_on.jsonl ../runs/1123_ds_off.jsonl
import json, math, sys
FP = [(0.040, 0.021), (0.036, 0.031), (0.029, 0.056), (-0.029, 0.056), (-0.049, 0.049), (-0.071, 0.042), (-0.076, 0.037),
      (-0.076, -0.037), (-0.071, -0.042), (-0.049, -0.049), (-0.029, -0.056), (0.029, -0.056), (0.036, -0.031), (0.040, -0.021)]
top = lambda x, y, th: max(y + fx * math.sin(th) + fy * math.cos(th) for fx, fy in FP)
for f in sys.argv[1:]:
    rows = [json.loads(l) for l in open(f) if l.strip()]
    rows = [r for r in rows[1:] if isinstance(r, dict) and "t" in r]
    for ts, name in [(r["t"], r["robot"]) for r in rows if r.get("out") == "ReportOrderCmd"]:
        s = [(r["t"], *r["args"][1:5]) for r in rows if r.get("in") == "on_robot_state" and r["args"][0] == name and ts <= r["t"] <= ts + 8]
        mv = [p for p in s if p[4] in ("MOVING", "RECOVERING")]
        if not mv:
            continue
        turn = next((p for p in mv if abs(math.degrees(math.atan2(math.sin(p[3] - math.pi), math.cos(p[3] - math.pi)))) > 30), None)
        w = max(mv, key=lambda p: top(p[1], p[2], p[3]))
        print(f"{f.split('/')[-1]} {name} O에 선 y {100 * (s[0][2] - 0.868):+.1f} cm",
              f"| 30도 넘게 돈 자리 x {turn[1]:.3f} y {turn[2]:.3f}" if turn else "",
              f"| 벽까지 최소 {100 * (0.965 - top(w[1], w[2], w[3])):.1f} cm (x {w[1]:.3f}, yaw {math.degrees(w[3]):.0f}도)")
