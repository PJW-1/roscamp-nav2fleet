#!/usr/bin/env python3
"""flip_metrics.py와 arrival_check.py의 CSV를 보고서 표(markdown)로 묶는다.

판 이름: 1단계 s1_<cm>_<판><회>(기준, RPP), 2단계 s2_<cm>_<안>_<컨트롤러>_<판><회>.
cm: a1 = 1 cm, a = 2.5 cm. 판: v = 장애물(gz_v7_varied), r = 러시(gz_v7_rush). 안: base, A, B, C.
사용: python3 make_tables.py metrics.csv arrival.csv > tables.md
"""
import csv
import re
import sys
from collections import defaultdict

NAME = re.compile(r"^s(?P<stage>[12])_(?P<cm>a1|a)_(?:(?P<plan>base|A|B|C)_(?P<ctrl>rpp|rl)_)?"
                  r"(?P<board>[vr])(?P<i>\d+)$")
CM = {"a1": "1 cm", "a": "2.5 cm"}
BOARD = {"v": "장애물", "r": "러시"}
PLAN = {"base": "기준", "A": "A keeper", "B": "B 관제 길 지정", "C": "C 3 s"}
CTRL = {"rpp": "RPP", "rl": "RL+shim"}
# (열 이름, 표 머리, 합으로 볼지)
COLS = [("pass", "통과", True), ("duration_s", "통과 시간 s", False), ("flips", "뒤집힘", True),
        ("flips_planner", "planner 뒤집힘", True), ("hall_spins", "HALL 회전 수", True),
        ("hall_spin_s", "HALL 회전 s", True), ("recoveries", "복구", True),
        ("blocked_wait", "BLOCKED_WAIT", True), ("path_not_connected", "path_not_connected", True),
        ("dist_jumps_by_seq", "거리 튐", True), ("shim_spins", "제자리 회전(cmd)", True),
        ("slow", "도착 5 s 넘음", True), ("near_fail", "닿은 뒤 실패", True)]


def key_of(run):
    m = NAME.match(run)
    if not m:
        return None
    d = m.groupdict()
    return (d["cm"], d["board"], d["plan"] or "base", d["ctrl"] or "rpp")


def num(v):
    if v in (None, "", "None"):
        return None
    if v in ("Y", "True"):
        return 1.0
    if v in ("N", "False"):
        return 0.0
    try:
        return float(v)
    except ValueError:
        return None


def main():
    rows = {r["run"]: r for r in csv.DictReader(open(sys.argv[1]))}
    for r in csv.DictReader(open(sys.argv[2])):
        if r["run"] in rows:
            rows[r["run"]].update({k: r[k] for k in ("slow", "near_fail")})
    groups = defaultdict(list)
    for run, r in sorted(rows.items()):
        k = key_of(run)
        if k:
            groups[k].append((run, r))
    for cm in ("a1", "a"):
        for board in ("v", "r"):
            keys = [k for k in groups if k[0] == cm and k[1] == board]
            if not keys:
                continue
            order = {"base": 0, "A": 1, "B": 2, "C": 3}
            keys.sort(key=lambda k: (k[3], order[k[2]]))
            print(f"\n#### {CM[cm]}, {BOARD[board]} 판 (판당 평균, 괄호는 판별 값)\n")
            print("| 안 | 컨트롤러 | 판 수 | " + " | ".join(h for _, h, _ in COLS) + " |")
            print("|" + "---|" * (3 + len(COLS)))
            for k in keys:
                runs = groups[k]
                cells = []
                for col, _, _ in COLS:
                    vals = [num(r.get(col)) for _, r in runs]
                    got = [v for v in vals if v is not None]
                    if not got:
                        cells.append("-")
                        continue
                    mean = sum(got) / len(got)
                    each = "/".join("-" if v is None else f"{v:g}" for v in vals)
                    if col == "pass":
                        cells.append(f"{int(sum(got))}/{len(got)}")
                    elif col == "duration_s":
                        cells.append(f"{mean:.0f}")
                    else:
                        cells.append(f"{mean:.1f} ({each})")
                print(f"| {PLAN[k[2]]} | {CTRL[k[3]]} | {len(runs)} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main()
