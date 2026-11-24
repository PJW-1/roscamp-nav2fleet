#!/usr/bin/env python3
"""Nav2 goal마다 첫 피드백부터 distance_remaining이 0보다 커질 때까지 걸린 시간을 잰다(bag 기준).

쓰기: nav_first_distance.py <판 폴더>... (ROS와 워크스페이스를 source한 셸에서)
"""
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import flip_metrics as fm  # noqa: E402


def gaps_of(bag):
    gaps = []
    for feedback in bag["nav"].values():
        first, positive = {}, {}
        for t, goal, dist in feedback:
            first.setdefault(goal, t)
            if dist > 0 and goal not in positive:
                positive[goal] = t
        gaps += [positive[g] - t0 for g, t0 in first.items() if g in positive]
    return gaps


def main():
    for run in sys.argv[1:]:
        bag = fm.load_run_data(run)[3]
        if not bag:
            print(f"{Path(run).name:18s} bag 없음")
            continue
        g = gaps_of(bag)
        print(f"{Path(run).name:18s} goal {len(g):3d}  중앙 {statistics.median(g):.2f} s  "
              f"평균 {statistics.mean(g):.2f} s  최대 {max(g):.2f} s")


if __name__ == "__main__":
    main()
