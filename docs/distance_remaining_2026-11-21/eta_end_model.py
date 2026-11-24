"""Nav2 goal 끝 보정(nav_end_s)을 넣은 ETA를 이미 돈 판의 기록으로 잰다.

    python3 eta_end_model.py RESULT_DIR ...
"""
from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import eta_error  # noqa: E402
from fleet_core.common.params import load_params  # noqa: E402
from fleet_core.traffic.zone.routes import nav_end_s  # noqa: E402

replay = eta_error.replay
ON = load_params()
PASSING = ("nav_goal", "follow_path")


def samples(record: Path, every: float = 0.5):
    lines, entries, _ = replay._read(record)
    head, *rest = entries
    now = [0.0]
    core = replay.start_core(head, lambda: now[0])
    produced: list[str] = []
    core.record_to(produced.append)
    traffic = core.traffic
    taken, last, done_at = [], {}, {}
    for entry in rest:
        if "in" not in entry:
            continue
        now[0] = entry["t"]
        before = len(produced)
        getattr(core, entry["in"])(*replay._arguments(entry))
        for line in produced[before:]:
            out = json.loads(line)
            if out.get("out") == "EventOut" and out.get("code") == "SEGMENT_DONE":
                order, _, number = out["detail"].split(" ")[:3]
                done_at.setdefault((order, int(number)), out["t"])
        if entry["in"] != "on_robot_state":
            continue
        robot = entry["args"][0]
        m = traffic._robots.get(robot)
        found = eta_error.current_nav(traffic, m) if m is not None else None
        if found is None or now[0] - last.get(robot, -math.inf) < every - 1e-9:
            continue
        last[robot] = now[0]
        order, seq, cut = found
        proj, used, from_distance = eta_error.estimates(traffic, m, cut)
        edges = m.route.edges
        adjust = nav_end_s(ON, traffic.site, edges, cut[2], m.x, m.y, m.yaw)
        after = edges[cut[2]].mode if cut[2] < len(edges) else None
        taken.append((order, seq, now[0], proj, used, from_distance, adjust, after))
    differ = next((json.loads(new).get("t") for old, new in zip(lines[1:], produced[1:],
                                                                strict=False) if old != new),
                  None)
    rows = []
    for order, seq, t, proj, used, from_distance, adjust, after in taken:
        end = done_at.get((order, seq))
        if end is not None and end >= t:
            rows.append({"actual": end - t, "after": after, "from_distance": from_distance,
                         "proj": proj, "used": used,
                         "proj_end": max(0.0, proj + adjust), "used_end": max(0.0, used + adjust)})
    return rows, differ


def line(name, part, key):
    errors = [r[key] - r["actual"] for r in part]
    size = sorted(abs(e) for e in errors)
    p90 = size[min(len(size) - 1, math.ceil(0.9 * len(size)) - 1)]
    cells = [f"{statistics.mean(errors):+.2f}", f"{statistics.mean(size):.2f}", f"{p90:.2f}"]
    for low, high in eta_error.BINS:
        e = [r[key] - r["actual"] for r in part if low <= r["actual"] < high]
        cells.append(f"{statistics.mean(e):+.2f} / {statistics.mean(abs(x) for x in e):.2f}"
                     if e else "-")
    print(f"| {name} | " + " | ".join(cells) + " |")


def main(argv: list[str]) -> int:
    rows = []
    for folder in map(Path, argv):
        if not (folder / "summary.json").exists():
            continue
        found, differ = samples(folder / "fleet_record.jsonl")
        if differ is not None:
            print(f"skipped {folder}: replay differs from t {differ}")
            continue
        rows += found
    if not rows:
        print(__doc__)
        return 2
    bins = [f"{low:g}-{high:g} s" if high != math.inf else f"{low:g}+ s"
            for low, high in eta_error.BINS]
    print(f"nav_pass_m {ON.nav_pass_m}, nav_turn_rps {ON.nav_turn_rps}, "
          f"nav_goal_tolerance_m {ON.nav_goal_tolerance_m}, nav_speed_mps {ON.nav_speed_mps}")
    groups = (("all", rows), ("next piece passes (nav_goal, follow_path)",
                              [r for r in rows if r["after"] in PASSING]),
              ("robot stops at the end (retreat, none)",
               [r for r in rows if r["after"] not in PASSING]))
    for title, part in groups:
        if not part:
            print(f"\n### {title}: no samples")
            continue
        print(f"\n### {title}: {len(part)} samples, distance used in "
              f"{sum(r['from_distance'] for r in part)}")
        print("| estimate | mean s | mean abs s | p90 abs s | " + " | ".join(bins) + " |")
        print("|---" * (4 + len(bins)) + "|")
        for name, key in (("projection", "proj"), ("distance", "used"),
                          ("projection + end", "proj_end"), ("distance + end", "used_end")):
            line(name, part, key)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
