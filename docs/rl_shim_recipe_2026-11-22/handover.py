import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path.home() / "nf_src/pd-compare/tools/nav_bench"))
import metrics as mt  # noqa: E402
import movers as mv  # noqa: E402

body, statics = mt.footprint(), mt.arena_rects()
print("footprint", [tuple(round(c, 4) for c in p) for p in body])
legs = Path.home() / "rl_gz_check/cmp012_025415" / sys.argv[1] / "legs"
for p in sorted(legs.glob(sys.argv[2])):
    r = json.loads(p.read_text())
    t0 = r["t_start"]
    cmds = [c for c in r["cmd"] if c[0] >= t0]
    first = next((c for c in cmds if abs(c[1]) > 1e-6), None)
    if first is None:
        continue
    th = first[0]
    pose = min(r["truth"], key=lambda q: abs(q[0] - th))
    plan = max((pl for pl in r["plans"] if pl[0] <= th), key=lambda pl: pl[0], default=r["plans"][0])
    pts = plan[1]
    sx, sy = pts[0]
    samp = next(((x, y) for x, y in pts[1:] if math.hypot(x - sx, y - sy) >= 0.15), pts[-1])
    _, x, y, yaw = pose
    err = math.atan2(math.sin(math.atan2(samp[1] - y, samp[0] - x) - yaw),
                     math.cos(math.atan2(samp[1] - y, samp[0] - x) - yaw))
    w = [c for c in cmds if c[0] < th][-1][2] if any(c[0] < th for c in cmds) else 0.0
    ahead = []
    sgn = 1.0 if err > 0 else -1.0
    for k in range(1, 21):
        yy = yaw + sgn * 1.0 * 0.05 * k
        shape = mv.to_world(body, x, y, yy)
        ahead.append(min(mv.poly_dist(shape, rr) for rr in statics))
    print(f"{p.name}: handover t {th - t0:.2f} s pose ({x:.3f},{y:.3f},{yaw:.2f}) w_before {w:.2f} "
          f"plan@{plan[0] - t0:.1f}s sample ({samp[0]:.3f},{samp[1]:.3f}) err {err:+.2f} rad; "
          f"footprint clearance over next 1 s of rotation min {min(ahead):.3f} m")
