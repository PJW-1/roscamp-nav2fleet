# 보고 위치가 0.5 s마다 몸 방향대로(앞/뒤) 움직였는지 본다. 옆으로 간 칸은 위치 보정, 밀림, 미끄러짐 중 하나다 (읽기만 한다)
# 쓰기: python3 reported_motion.py ../runs/1123_ds_off.jsonl pode1 360 432
import json, math, sys
path, name, lo, hi = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
rows = [json.loads(l) for l in open(path) if l.strip()]
rows = [r for r in rows[1:] if isinstance(r, dict) and "t" in r]
t0 = rows[0]["t"]
s = [(r["t"] - t0, *r["args"][1:5]) for r in rows if r.get("in") == "on_robot_state" and r["args"][0] == name and lo <= r["t"] - t0 <= hi]
last = None
for p in s:
    if last and p[0] - last[0] < 0.5:
        continue
    if last:
        dx, dy = p[1] - last[1], p[2] - last[2]
        d = math.hypot(dx, dy)
        if d > 0.02:
            mis = (math.degrees(math.atan2(dy, dx)) - math.degrees(last[3]) + 180) % 360 - 180
            kind = "앞" if abs(mis) < 45 else "뒤" if abs(mis) > 135 else "옆"
            fast = " 속도 넘침" if d / (p[0] - last[0]) > 0.30 else ""
            print(f"+{p[0]:.1f} ({p[1]:.2f},{p[2]:.2f}) yaw {math.degrees(p[3]):4.0f} {p[4]:10s} {d * 100:4.1f} cm {mis:+4.0f}도 {kind}{fast}")
    last = p
