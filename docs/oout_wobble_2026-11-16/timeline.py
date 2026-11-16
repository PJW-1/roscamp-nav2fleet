import math, re, sys
src = open("analyze_cause.py").read().split("# ---- odom -> 지도")[0]
exec(src)
for t0, r in seg_out[:3]:
    print(f"\n===== {r} O_OUT 도착 t0 (로그 SEGMENT_DONE) =====")
    ev = []
    for t, m in data[(r, "cmd_vel")]:
        if t0 - 1.5 <= t <= t0 + 4:
            lp = at(r, "lookahead_point", t); b = math.degrees(math.atan2(lp[1].point.y, lp[1].point.x)) if lp else float("nan")
            od = at(r, "odom", t); wo = od[1].twist.twist.angular.z if od else float("nan")
            ev.append((t, f"cmd v {tw(m).linear.x:+.3f} w {tw(m).angular.z:+.2f} | odom w {wo:+.2f} | carrot {b:+.0f}"))
    for line in log:
        if f"[{r}" in line or f" {r}-" in line or f" {r}:" in line:
            try: t = ts(line)
            except Exception: continue
            if t0 - 1.5 <= t <= t0 + 4:
                txt = re.sub(r"^.*?\]: ", "", line)[:110]
                node = re.search(r"\[(pode\d\.[a-z_]+)", line)
                ev.append((t, f"LOG {node.group(1) if node else ''}: {txt}"))
    for line in log:
        if "fleet" in line[:60] and r in line:
            t = ts(line) if re.search(r"\[\d{10}\.\d+\]", line) else None
            if t and t0 - 1.5 <= t <= t0 + 4 and not any(abs(t - e[0]) < 1e-6 for e in ev):
                ev.append((t, "FLEET: " + re.sub(r"^.*?\]: ", "", line)[:110]))
    for t, s in sorted(ev):
        print(f"  {t - t0:+6.2f}  {s}")
