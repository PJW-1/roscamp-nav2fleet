"""Gazebo 묶음의 추가 확인 셋. report.sh 다음에 돌린다.

사용: python3 extra_checks.py BATCH_DIR [BATCH_DIR ...]
- 갱신 시점: 판마다가 아니라 묶음(급행 4판, v7 4판)의 Q2 Route를 모두 모은 update_lead_s 분포.
- 남은 거리가 0.10 m 넘게 는 곳: 같은 (로봇, goal, 끝낸 seq)의 연이은 피드백. LEFT_JUMP_M과 같은 기준.
- 속도 맞추기: eta_error.py 표본의 proj와 used를 nav_speed_mps 대신 다른 속도로 나눴다면의 오차.
  두 예측 모두 nav_goal Segment에서는 거리 / nav_speed_mps라 속도만 바꿔 다시 셀 수 있다.
"""
from __future__ import annotations

import json
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
JUMP_M = 0.10


def leads(batch: Path) -> None:
    for kind, runs in (("rush", RUSH), ("v7", V7)):
        found = []
        for run in runs:
            ev, rec, pos, *_ = step1_report.load(str(batch / run / "result"))
            found += [r["update_lead_s"] for r in step1_report.q2_routes(ev, rec, pos)
                      if r["update_lead_s"] is not None]
        if not found:
            continue
        found.sort()
        print(f"lead {batch.name} {kind}: n {len(found)} median {statistics.median(found):.2f} "
              f"mean {statistics.mean(found):.2f} min {found[0]:.2f} max {found[-1]:.2f}")


def rises(batch: Path) -> None:
    for run in (*RUSH, *V7):
        record = batch / run / "result" / "fleet_record.jsonl"
        if not record.exists():
            continue
        last, count = {}, {}
        for line in record.open():
            if '"on_move_feedback"' not in line:
                continue
            entry = json.loads(line)
            args = entry["args"]
            if len(args) < 6 or args[3]:
                continue
            key = tuple(args[:3])
            count[key] = count.get(key, 0) + 1
            if key in last and args[5] > last[key][1] + JUMP_M:
                print(f"rise {batch.name} {run} t {entry['t']:.1f} robot {key[0]} goal {key[1]} "
                      f"done {key[2]} feedback #{count[key]} {last[key][1]:.3f} -> {args[5]:.3f} m")
            last[key] = (entry["t"], args[5])


def speed_fit(batch: Path) -> None:
    rows = []
    for run in (*RUSH, *V7):
        result = batch / run / "result"
        if not (result / "fleet_record.jsonl").exists():
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
    print("| speed m/s | proj mean abs | proj p90 | proj mean | used mean abs | used p90 | used mean |")
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
        rises(batch)
    for batch in map(Path, argv):
        speed_fit(batch)
    return 0


if __name__ == "__main__":
    sys.exit(main())
