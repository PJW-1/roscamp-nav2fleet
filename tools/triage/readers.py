"""기록 읽기: 관제 기록(JSON 줄), 옛 Gazebo 기록기의 rec.csv, 장애물 yaml.

관제 기록의 형식은 tools/replay.py 머리에 있다. replay는 코어를 불러오지만 여기는 읽기만 하므로
따로 읽는다.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

from .model import Event, Obstacle, Run


def _note_event(run: Run, event: Event) -> None:
    """사건을 넣고, 주문 사건이면 주문판과 끝 보고도 채운다(rec.csv에는 보고 줄이 없다)."""
    run.events.append(event)
    order = event.detail.split()[0] if event.detail.split() else ""
    if event.code in ("ORDER_ACCEPTED", "ORDER_BOUND"):
        run.add_order(order)
    elif event.code in ("ORDER_SHIPPED", "ORDER_PARTIAL", "ORDER_HELD"):
        run.add_order(order)
        run.reports[order] = (event.t, event.code.removeprefix("ORDER_"))
    run.end_t = max(run.end_t, event.t)


def read_fleet_record(source) -> Run:
    """`--record`가 쓴 파일(경로) 또는 줄 목록. 줄 중간에서 끝난 마지막 줄은 cut으로 표시한다."""
    lines = (Path(source).read_text(encoding="utf-8").splitlines()
             if isinstance(source, (str, Path)) else list(source))
    run = Run()
    for number, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            entry = json.loads(line)
        except json.JSONDecodeError as error:
            if number == len(lines):
                run.cut = True
                break
            raise ValueError(f"line {number} is not JSON: {error}") from error
        if "t" not in entry:
            run.params = entry.get("params", {})  # 첫 줄: 코어를 띄운 설정
            continue
        t = float(entry["t"])
        run.end_t = max(run.end_t, t)
        if entry.get("out") == "EventOut":
            _note_event(run, Event(t, entry["code"], entry.get("robot", ""),
                                   entry.get("detail", ""), entry.get("level", "INFO")))
        elif entry.get("out") == "ReportOrderCmd":
            run.add_order(entry["order_id"])
            run.reports[entry["order_id"]] = (t, entry["status"])
        elif entry.get("in") == "on_order_board":
            for order in entry["args"][1]:
                run.add_order(order.get("order_id", "") if isinstance(order, dict) else "")
        elif entry.get("in") == "on_robot_state":
            robot, x, y, _yaw, state = entry["args"][:5]
            run.states.setdefault(robot, []).append((t, x, y, state))
    return run


def read_rec_csv(path) -> Run:
    """rec.csv: `t,event,robot,level,code,detail`, `t,rep,robot,x,y,"<yaw> <STATE> ..."`,
    `t,pose,robot,x,y,...`. 첫 줄이 `t`로 시작하면 머리다. 칸이 더 있어도 무시하고
    못 읽는 줄은 뺀다."""
    run = Run()
    with open(path, newline="", encoding="utf-8") as file:
        for row in csv.reader(file):
            if len(row) < 3 or row[0].strip() == "t":
                continue
            try:
                t, kind, robot = float(row[0]), row[1].strip(), row[2].strip()
                if kind == "event":
                    detail = row[5] if len(row) > 5 else ""
                    _note_event(run, Event(t, row[4].strip(), robot, detail, row[3].strip()))
                elif kind == "rep":
                    state = row[5].split()[1] if len(row) > 5 and len(row[5].split()) > 1 else ""
                    run.states.setdefault(robot, []).append(
                        (t, float(row[3]), float(row[4]), state))
                elif kind == "pose":
                    run.poses.setdefault(robot, []).append((t, float(row[3]), float(row[4])))
                else:
                    continue
            except (ValueError, IndexError):
                continue
            run.end_t = max(run.end_t, t)
    return run


def obstacles_from_injections(injections) -> list[Obstacle]:
    """머리 없는 세계의 `Obstacle(x, y, radius, t0, t1)` 주입(world/injections.py)만 골라 옮긴다."""
    return [Obstacle(i.x, i.y, i.radius, i.t0, i.t1) for i in injections
            if type(i).__name__ == "Obstacle"]


def read_obstacles(path, nodes=None) -> list[Obstacle]:
    """sim/fleet_gz_sim/obstacles/*.yaml. 자리는 `x, y`, 또는 `link: [a, b]`와 `fraction`
    (지점 이름 -> (x, y)인 `nodes`가 있어야 한다). 반지름은 `size_m` 첫 값의 절반이다."""
    import yaml
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    radius = float(data.get("size_m", [0.1])[0]) / 2
    out = []
    for item in data.get("obstacles", ()):
        if "link" in item:
            (ax, ay), (bx, by) = nodes[item["link"][0]], nodes[item["link"][1]]
            f = float(item.get("fraction", 0.5))
            x, y = ax + (bx - ax) * f, ay + (by - ay) * f
        else:
            x, y = float(item["x"]), float(item["y"])
        t0 = float(item.get("at_s", 0.0))
        out.append(Obstacle(x, y, radius, t0, t0 + float(item.get("duration_s", float("inf")))))
    return out
