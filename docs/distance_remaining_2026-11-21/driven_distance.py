"""nav_goal Segment마다 세 거리 비교: 곧은 거리(투영), Nav2 남은 거리, 실제로 달린 거리.

사용: python3 driven_distance.py RUN_DIR [RUN_DIR ...]   (판 폴더: result/, truth.csv)
- 끝난 판(summary.json 있음)만 본다. 관제 기록의 시각은 시뮬 시각이라 truth.csv의 sim_t와 맞춘다.
- Segment마다 남은 거리를 쓴 첫 표본(투영 2 s 넘음) 하나를 고른다. 그 시각부터 Segment가 끝날 때까지
  로봇이 실제로 달린 길이와, 그 시각의 투영 거리(proj x 0.12), Nav2 거리(used x 0.12)를 비교한다.
"""
import bisect
import csv
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eta_error  # noqa: E402

SPEED = 0.12


def segments(run: Path) -> list[tuple]:
    track = {}
    for r in csv.DictReader(open(run / "truth.csv")):
        track.setdefault(r["robot"], []).append((float(r["sim_t"]), float(r["x"]), float(r["y"])))
    found, differ = eta_error.samples(run / "result" / "fleet_record.jsonl", 0.5)
    if differ is not None:
        print("replay differs", run, differ)
        return []
    seen, out = set(), []
    for s in found:
        if not s["from_distance"] or s["proj"] < 2.0 or s["segment"] in seen:
            continue
        seen.add(s["segment"])
        pts = track[s["segment"].split("-")[0]]
        times = [p[0] for p in pts]
        a = bisect.bisect_left(times, s["t"])
        b = bisect.bisect_left(times, s["t"] + s["actual"])
        seg = pts[a:b + 1]
        driven = sum(math.dist(p[1:], q[1:]) for p, q in zip(seg, seg[1:], strict=False))
        out.append((run.name, s["segment"], s["proj"] * SPEED, s["used"] * SPEED, driven,
                    s["actual"]))
    return out


def main(argv: list[str]) -> int:
    rows = []
    for run in map(Path, argv):
        if (run / "result" / "summary.json").exists():
            rows += segments(run)
    if not rows:
        print(__doc__)
        return 2
    print(f"segments {len(rows)} (first sample per nav_goal segment where the distance was used "
          f"and proj > 2 s)")
    for name, i in (("straight / driven", 2), ("nav2 / driven", 3)):
        ratio = sorted(r[i] / r[4] for r in rows if r[4] > 0.1)
        print(f"{name}: median {statistics.median(ratio):.3f}  p10 {ratio[len(ratio) // 10]:.3f}  "
              f"p90 {ratio[9 * len(ratio) // 10]:.3f}")
    speed = sorted(r[4] / r[5] for r in rows)
    print(f"driven / actual time (m/s): median {statistics.median(speed):.3f}")
    for r in rows[:8]:
        print(f"  {r[0]} {r[1]}: straight {r[2]:.3f} nav2 {r[3]:.3f} driven {r[4]:.3f} m, "
              f"actual {r[5]:.2f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
