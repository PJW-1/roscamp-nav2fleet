import json
import sys
from pathlib import Path

for f in sys.argv[1:]:
    r = json.loads(Path(f).read_text())
    t0 = r["t_start"]
    plans = [round(p[0] - t0, 2) for p in r["plans"]]
    runs, cur = [], None
    for t, v, w in r["cmd"]:
        if t < t0 or t > r["t_end"]:
            continue
        zero = v == 0.0 and w != 0.0
        if zero and cur is None:
            cur = [t - t0, t - t0, 0]
        if zero:
            cur[1] = t - t0
            cur[2] += 1
        elif cur is not None:
            runs.append(cur)
            cur = None
    if cur:
        runs.append(cur)
    print(Path(f).name, "plans at", plans[:8])
    print("   v == 0 runs (start s, end s, cmds):", [(round(a, 2), round(b, 2), n) for a, b, n in runs][:12])
