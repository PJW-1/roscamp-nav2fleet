"""Q2에서 실제로 선 시간(sim/truth): Q2 반경 안에서 속도가 기준 아래로 이어진 구간.

사용: python3 q2_dwell.py TRUTH.csv BATCH.log | NAME=TRUTH.csv ... [--radius=0.10 --speed=0.03 --min=0.1]
기본값(0.10 m, 0.03 m/s, 0.1 s)은 기준 판 rush4에서 BASE_END_WAIT 수(11)와 맞았다.
"""
import math
import statistics as st
import sys
from collections import defaultdict

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from express_metrics import read_truth, runs_from_log  # noqa: E402

Q2 = (1.66, 0.15)
OPT = {k: float(v) for k, v in (a[2:].split("=") for a in sys.argv[1:]
                                 if a.startswith("--") and not a.startswith("--date="))}
RADIUS, SPEED, MIN_S = OPT.get("radius", 0.10), OPT.get("speed", 0.03), OPT.get("min", 0.1)


def dwells(rr):
    out, start = [], None
    for (t0, x0, y0, _), (t1, x1, y1, _) in zip(rr, rr[1:]):
        if t1 <= t0:
            continue
        near = math.hypot(x1 - Q2[0], y1 - Q2[1]) <= RADIUS
        slow = math.hypot(x1 - x0, y1 - y0) / (t1 - t0) < SPEED
        if near and slow:
            start = t0 if start is None else start
        elif start is not None:
            out.append(t0 - start)
            start = None
    if start is not None:  # 끝까지 서 있던 구간
        out.append(rr[-1][0] - start)
    return [d for d in out if d >= MIN_S]


args = [a for a in sys.argv[1:] if not a.startswith("--")]
if len(args) == 2 and args[1].endswith(".log"):
    rows = read_truth(args[0])
    runs = [(n, w0, w1 or math.inf, rows) for n, w0, w1 in runs_from_log(args[1])]
else:
    runs = [(a.split("=", 1)[0], -math.inf, math.inf, read_truth(a.split("=", 1)[1])) for a in args]
print(f"| run | Q2 stops (r {RADIUS} m, v < {SPEED} m/s, >= {MIN_S} s) | median s | mean s | max s |")
print("|---|---|---|---|---|")
for name, w0, w1, rows in runs:
    by = defaultdict(list)
    for wall, t, robot, x, y, yaw in rows:
        if w0 <= wall <= w1:
            by[robot].append((t, x, y, yaw))
    ds = [d for rr in by.values() for d in dwells(sorted(rr))]
    if ds:
        print(f"| {name} | {len(ds)} | {st.median(ds):.2f} | {st.mean(ds):.2f} | {max(ds):.2f} |")
    else:
        print(f"| {name} | 0 | - | - | - |")
