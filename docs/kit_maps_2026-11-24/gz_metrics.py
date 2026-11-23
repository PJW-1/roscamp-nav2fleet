#!/usr/bin/env python3
"""Gazebo gz_v7 한 판(sim_batch.sh 출력 폴더)의 지표: 통과, 격리·BLOCKED, RPP 경고, 급행 여유.

사용: python3 gz_metrics.py 이름=판폴더 ...
급행 여유는 truth.csv(Gazebo 실제 위치)로 잰다. 로봇 중심이 x 0.70~1.56, y > 0.80인 동안
발자국(nav2_params local 다각형)에서 북쪽 벽(y 0.98)까지, 로봇 중심에서 시뮬 진열대 A
[0.70, 0.77, 1.56, 0.783] 위 면까지의 최소다.
"""
import csv
import json
import math
import sys
from pathlib import Path

WALL_N = 0.98
SHELF_A = (0.70, 0.77, 1.56, 0.783)
FOOT = [(0.040, 0.021), (0.036, 0.031), (0.029, 0.056), (-0.029, 0.056), (-0.049, 0.049),
        (-0.071, 0.042), (-0.076, 0.037), (-0.076, -0.037), (-0.071, -0.042), (-0.049, -0.049),
        (-0.029, -0.056), (0.029, -0.056), (0.036, -0.031), (0.040, -0.021)]


def foot_max_y(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return max(y + s * px + c * py for px, py in FOOT)


def metrics(run):
    run = Path(run)
    summary = json.loads((run / "result" / "summary.json").read_text(encoding="utf-8"))
    log = (run / "launch.out").read_text(encoding="utf-8", errors="replace")
    wall, shelf = (math.inf, None), (math.inf, None)
    with open(run / "truth.csv", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            x, y, yaw = float(row["x"]), float(row["y"]), float(row["yaw"])
            if not (SHELF_A[0] < x < SHELF_A[2] and y > 0.80):
                continue
            d = WALL_N - foot_max_y(x, y, yaw)
            if d < wall[0]:
                wall = (d, (row["robot"], round(x, 3), round(y, 3)))
            d = y - SHELF_A[3]
            if d < shelf[0]:
                shelf = (d, (row["robot"], round(x, 3), round(y, 3)))
    return {"pass": summary.get("pass"),
            "shipped": f"{summary.get('orders_shipped')}/{summary.get('orders_total')}",
            "duration_s": round(summary.get("duration_s", float("nan")), 1),
            "ROBOT_ISOLATED lines": log.count("ROBOT_ISOLATED"),
            "BLOCKED lines": log.count("BLOCKED"),
            "collision ahead lines": log.count("collision ahead"),
            "express wall min (footprint)": (round(wall[0], 4), wall[1]),
            "express shelf A min (centre)": (round(shelf[0], 4), shelf[1])}


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        name, path = arg.split("=", 1)
        print(name, json.dumps(metrics(path), ensure_ascii=False))
