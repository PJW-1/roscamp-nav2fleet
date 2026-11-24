"""nav_goal Segment의 남은 시간 예측 오차를 판의 fleet_record.jsonl 재생으로 잰다.

    python3 eta_error.py NAME=RESULT_DIR ... [--every=0.5] [--per-run]
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))
spec = importlib.util.spec_from_file_location("replay", REPO / "tools" / "replay.py")
replay = importlib.util.module_from_spec(spec)
sys.modules["replay"] = replay
spec.loader.exec_module(replay)

from fleet_core.fleet_map import NAV_GOAL  # noqa: E402
from fleet_core.traffic.zone.routes import left_s  # noqa: E402

BINS = ((0.0, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 8.0), (8.0, math.inf))


def current_nav(traffic, m):
    mo, route = m.move_order, m.route
    if mo is None or route is None or m.goal is None:
        return None
    seq = mo.done_seq + 1
    cut = next((c for c in mo.cuts[:mo.given] if c[0] == seq), None)
    if cut is None or cut[1] != route.done or route.edges[cut[1]].mode != NAV_GOAL:
        return None
    return mo.id, seq, cut


def estimates(traffic, m, cut):
    route = m.route
    proj = left_s(traffic.params, route.edges[route.done:cut[2]], m.x, m.y)
    used_of = getattr(traffic, "_left_s", None)
    if used_of is None:
        return proj, proj, False
    used = used_of(m, route, cut[2])
    return proj, used, traffic._reported_end(m, route) is not None


def samples(record, every=0.5):
    lines, entries, _ = replay._read(record)
    head, *rest = entries
    now = [0.0]
    core = replay.start_core(head, lambda: now[0])
    produced: list[str] = []
    core.record_to(produced.append)
    taken, last = [], {}
    done_at = {}
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
        m = core.traffic._robots.get(robot)
        found = current_nav(core.traffic, m) if m is not None else None
        if found is None or now[0] - last.get(robot, -math.inf) < every - 1e-9:
            continue
        last[robot] = now[0]
        order, seq, cut = found
        proj, used, from_distance = estimates(core.traffic, m, cut)
        taken.append((order, seq, now[0], proj, used, from_distance))
    differ = next((json.loads(new).get("t") for old, new in zip(lines[1:], produced[1:],
                                                                strict=False) if old != new),
                  None)
    out = []
    for order, seq, t, proj, used, from_distance in taken:
        end = done_at.get((order, seq))
        if end is not None and end >= t:
            out.append({"segment": f"{order} seq {seq}", "t": t, "actual": end - t,
                        "proj": proj, "used": used, "from_distance": from_distance})
    return out, differ


def p90(values):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(0.9 * len(ordered)) - 1)]


def summary(rows):
    def stats(part):
        row = {"samples": len(part),
               "segments": len({r["segment"] for r in part}),
               "used_from_distance": (round(sum(r["from_distance"] for r in part) / len(part), 2)
                                      if part else None)}
        for key in ("proj", "used"):
            errors = [abs(r[key] - r["actual"]) for r in part]
            row[f"{key}_mean_abs_s"] = round(sum(errors) / len(errors), 2) if errors else None
            row[f"{key}_p90_abs_s"] = round(p90(errors), 2) if errors else None
            signed = [r[key] - r["actual"] for r in part]
            row[f"{key}_mean_s"] = round(sum(signed) / len(signed), 2) if signed else None
        return row

    table = {"all": stats(rows)}
    for low, high in BINS:
        name = f"{low:g}-{high:g} s" if high != math.inf else f"{low:g}+ s"
        table[name] = stats([r for r in rows if low <= r["actual"] < high])
    return table


def show(title, table):
    keys = ["bin", *next(iter(table.values())).keys()]
    print(f"\n### {title}")
    print("| " + " | ".join(keys) + " |")
    print("|" + "---|" * len(keys))
    for name, row in table.items():
        print("| " + " | ".join([name, *(str(row[k]) for k in keys[1:])]) + " |")


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    opts = dict(a[2:].split("=", 1) if "=" in a else (a[2:], "1")
                for a in argv if a.startswith("--"))
    every = float(opts.get("every", 0.5))
    runs = [a.split("=", 1) for a in argv if not a.startswith("--")]
    if not runs:
        print(__doc__)
        return 2
    everything = []
    for name, folder in runs:
        record = Path(folder) / "fleet_record.jsonl"
        rows, differ = samples(record, every)
        if differ is not None:
            print(f"warning: {name}: this code decides differently from t {differ} s on, "
                  "so the samples after that are not the run's own (see tools/replay.py)")
        everything += rows
        if "per-run" in opts:
            show(name, summary(rows))
    show(f"all {len(runs)} runs", summary(everything))
    return 0


if __name__ == "__main__":
    sys.exit(main())
