#!/usr/bin/env python3
"""관제 기록의 거리 튐을 조각 첫 값 바로 뒤 튐과 나머지로 나눠 센다.

조각 첫 값 뒤에 커지는 튐은 Nav2 첫 거리가 오기 전 pode_fsm이 보낸 직선거리가 경로 길이로 바뀐 것이다.
쓰기: dist_jump_split.py <판 폴더>...
"""
import json
import sys
from pathlib import Path

JUMP_M = 0.10  # flip_metrics.DIST_JUMP_M와 같다


def split(record):
    count, last = {}, {}
    start = other = 0
    for line in open(record):
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if d.get("in") != "on_move_feedback" or len(d["args"]) < 6:
            continue
        robot, goal, seq, _, _, dist = d["args"][:6]
        if dist is None or dist < 0:
            continue
        key = (robot, goal, seq)
        count[key] = count.get(key, 0) + 1
        if key in last and abs(dist - last[key]) > JUMP_M:
            if count[key] == 2 and dist > last[key]:
                start += 1
            else:
                other += 1
        last[key] = dist
    return start, other


def main():
    for run in sys.argv[1:]:
        start, other = split(Path(run) / "result" / "fleet_record.jsonl")
        print(f"{Path(run).name:18s} 첫 값 뒤 {start:2d}  나머지 {other:2d}")


if __name__ == "__main__":
    main()
