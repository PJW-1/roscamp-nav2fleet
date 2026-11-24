"""Gazebo 묶음의 추가 확인(갱신 시점, 남은 거리 뜀, 속도 맞추기). report.sh 다음에 돌린다.

    python3 extra_checks.py BATCH_DIR ...
"""
from __future__ import annotations

import math
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "baseline_sim_2026-11-19"))
import eta_error  # noqa: E402
import step1_report  # noqa: E402

RUSH = ("rush1", "rush2", "rush3", "rush4")
V7 = ("v7a", "v7b", "v7c", "v7d")
BASE_SPEED = 0.12


def leads(batch: Path) -> None:
    for kind, runs in (("rush", RUSH), ("v7", V7)):
        found = []
        for run in runs:
            if not (batch / run / "result" / "summary.json").exists():
                continue
            ev, rec, pos, *_ = step1_report.load(str(batch / run / "result"))
            found += [r["update_lead_s"] for r in step1_report.q2_routes(ev, rec, pos)
                      if r["update_lead_s"] is not None]
        if not found:
            continue
        found.sort()
        print(f"lead {batch.name} {kind}: n {len(found)} median {statistics.median(found):.2f} "
              f"mean {statistics.mean(found):.2f} min {found[0]:.2f} max {found[-1]:.2f}")


def core_jumps(record) -> list[dict]:
    replay = eta_error.replay
    _, entries, _ = replay._read(record)
    head, *rest = entries
    now = [0.0]
    core = replay.start_core(head, lambda: now[0])
    traffic = core.traffic
    first, found = {}, []
    for entry in rest:
        if "in" not in entry:
            continue
        now[0] = entry["t"]
        m = traffic._robots.get(entry["args"][0]) if entry["in"] == "on_move_feedback" else None
        before = m.move_order if m is not None else None
        old = before.left if before is not None else None
        core._time = now[0]
        usable = before is not None and traffic._reported_end(m, m.route) is not None
        getattr(core, entry["in"])(*replay._arguments(entry))
        if before is None or m.move_order is not before or before.left is None:
            continue
        new = before.left
        first.setdefault((m.name, before.id, new.seq), now[0])
        stamped_before = old.jumped_at if old is not None else None
        if new is old or new.jumped_at != now[0] or stamped_before == now[0]:
            continue
        cut = next((c for c in before.cuts[:before.given] if c[0] == new.seq), None)
        straight = mode = None
        if cut is not None and m.route is not None:
            end = traffic.site.node_pose(m.route.edges[cut[2] - 1].b)
            straight = math.dist((m.x, m.y), end)
            mode = m.route.edges[cut[1]].mode
        found.append({"t": now[0], "robot": m.name, "order": before.id, "seq": new.seq,
                      "mode": mode,
                      "old_m": old.m, "new_m": new.m, "straight_m": straight,
                      "old_usable": usable,
                      "since_first_s": now[0] - first[(m.name, before.id, new.seq)]})
    return found


def jumps(batch: Path) -> None:
    for run in (*RUSH, *V7):
        record = batch / run / "result" / "fleet_record.jsonl"
        if not (record.parent / "summary.json").exists():
            continue
        found = core_jumps(record)
        modes = {}
        for j in found:
            modes[str(j["mode"])] = modes.get(str(j["mode"]), 0) + 1
        print(f"jumps {batch.name} {run}: {len(found)} by segment mode {modes}")
        for j in found:
            if j["mode"] != eta_error.NAV_GOAL:
                continue
            straight = "-" if j["straight_m"] is None else f"{j['straight_m']:.3f}"
            print(f"jump {batch.name} {run} t {j['t']:.1f} {j['robot']} {j['order']} "
                  f"seq {j['seq']} {j['old_m']:.3f} -> {j['new_m']:.3f} m, straight {straight} m, "
                  f"old value usable {j['old_usable']}, {j['since_first_s']:.1f} s after the "
                  f"first value of this seq")


def speed_fit(batch: Path) -> None:
    rows = []
    for run in (*RUSH, *V7):
        result = batch / run / "result"
        if not (result / "summary.json").exists():
            continue
        found, differ = eta_error.samples(result / "fleet_record.jsonl", 0.5)
        if differ is not None:
            print(f"warning: {run} replay differs from t {differ}; skipped")
            continue
        rows += found
    if not rows:
        return

    def error(key, speed, part):
        signed = [r[key] * BASE_SPEED / speed - r["actual"] for r in part]
        size = sorted(abs(x) for x in signed)
        return (sum(size) / len(size), size[min(len(size) - 1, math.ceil(0.9 * len(size)) - 1)],
                sum(signed) / len(signed))

    print(f"\nspeed fit {batch.name}: {len(rows)} samples")
    print("| speed m/s | proj mean abs | proj p90 | proj mean "
          "| used mean abs | used p90 | used mean |")
    print("|---|---|---|---|---|---|---|")
    for speed in (0.12, 0.14, 0.16, 0.17, 0.18, 0.19, 0.20, 0.22):
        p, u = error("proj", speed, rows), error("used", speed, rows)
        print(f"| {speed:.2f} | {p[0]:.2f} | {p[1]:.2f} | {p[2]:+.2f} | "
              f"{u[0]:.2f} | {u[1]:.2f} | {u[2]:+.2f} |")
    for key in ("proj", "used"):
        best = min((error(key, s / 1000, rows)[0], s / 1000) for s in range(100, 300))
        print(f"best {key}: {best[1]:.3f} m/s, mean abs {best[0]:.2f} s")
        for low, high in eta_error.BINS:
            part = [r for r in rows if low <= r["actual"] < high]
            if part:
                e = error(key, best[1], part)
                print(f"  {low:g}-{high:g} s: n {len(part)} mean abs {e[0]:.2f} p90 {e[1]:.2f} "
                      f"mean {e[2]:+.2f}")
    ratio = sorted(r["used"] / r["proj"] for r in rows if r["from_distance"] and r["proj"] > 0.5)
    if ratio:
        print(f"used / proj where used came from distance (proj > 0.5 s): "
              f"median {statistics.median(ratio):.3f}, n {len(ratio)}")


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(__doc__)
        return 2
    for batch in map(Path, argv):
        leads(batch)
    for batch in map(Path, argv):
        jumps(batch)
    for batch in map(Path, argv):
        speed_fit(batch)
    return 0


if __name__ == "__main__":
    sys.exit(main())
