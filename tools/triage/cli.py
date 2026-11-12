"""끝나지 않은 주문의 원인을 표로 찍는다(지시서 6절).

    python tools/triage record run.jsonl [--obstacles x.yaml]   # fleet_manager --record 기록
    python tools/triage csv rec.csv [--obstacles x.yaml]        # 옛 Gazebo 기록기의 rec.csv
    python tools/triage sim express_obstacle -l d2 --seeds 1 2  # 머리 없는 세계 상황

장애물 yaml은 x, y가 적힌 것만 읽는다(link 항목은 지점 좌표가 필요해 파이썬에서 `nodes`를 준다).
"""
from __future__ import annotations

import argparse

from .classify import classify, markdown
from .headless import run_triaged
from .readers import read_fleet_record, read_obstacles, read_rec_csv


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="kind", required=True)
    for kind in ("record", "csv"):
        p = sub.add_parser(kind)
        p.add_argument("path")
        p.add_argument("--obstacles")
    p = sub.add_parser("sim")
    p.add_argument("situation")
    p.add_argument("-l", "--layout", default="d2")
    p.add_argument("--seeds", nargs="+", type=int, default=[1])
    args = parser.parse_args(argv)
    if args.kind == "sim":
        for seed in args.seeds:
            _, rows = run_triaged(args.situation, args.layout, seed)
            print(f"### {args.situation} {args.layout} 씨드 {seed}\n\n{markdown(rows)}\n")
        return 0
    read = read_fleet_record if args.kind == "record" else read_rec_csv
    obstacles = read_obstacles(args.obstacles) if args.obstacles else ()
    print(markdown(classify(read(args.path), obstacles)))
    return 0
