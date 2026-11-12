"""회피 회귀 시험의 판정(추가 지시서 3의 7-3). tools/avoid_regression.sh가 부른다. ROS 없이 돈다.

    python3 tools/avoid_regression.py drive drive_check.json   # 한 대: 이동마다 끝까지 갔나
    python3 tools/avoid_regression.py game <결과 폴더>          # 세 대 경기: "주행" 원인, keepout

결과 폴더는 gz.launch.py의 result_dir이다(fleet_record:=true로 돌린 것): fleet_record.jsonl과
fleet_check.json을 읽는다. 표를 찍고, 통과면 0, 아니면 1로 끝난다.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from triage import classify, read_fleet_record, read_obstacles  # noqa: E402
from triage.classify import markdown as causes_markdown  # noqa: E402

KEEPOUT_MAX_M = 0.02  # 기준 keepout_intrusion_max_m(회피_약속.md)
DRIVING = "주행"


def drive_rows(summary: dict) -> list[tuple]:
    """이동 종류마다 (이름, 횟수, 성공, 끝 위치 오차 m, 끝 방향 오차 도, 선에서 벗어남 m).

    오차 셋은 그 종류에서 가장 큰 값이고, 재지 못했으면 None이다."""
    kinds: dict[str, list] = {}
    for result in summary.get("results", []):
        kinds.setdefault(result["step"], []).append(result)
    rows = []
    for label, runs in kinds.items():
        def worst(key, runs=runs):
            values = [abs(r[key]) for r in runs if r.get(key) is not None]
            return max(values) if values else None
        rows.append((label, len(runs), sum(1 for r in runs if r["result"] == "success"),
                     worst("pos_error_m"), worst("yaw_error_deg"), worst("max_cross_track_m")))
    return rows


def drive_markdown(summary: dict) -> str:
    def cell(value, digits):
        return "" if value is None else f"{value:.{digits}f}"
    lines = ["| 이동 | 횟수 | 성공 | 끝 위치 오차 m(최대) | 끝 방향 오차 도(최대) "
             "| 선에서 벗어남 m(최대) |",
             "|---|---|---|---|---|---|"]
    lines += [f"| {label} | {runs} | {ok} | {cell(pos, 3)} | {cell(yaw, 1)} | {cell(cross, 3)} |"
              for label, runs, ok, pos, yaw, cross in drive_rows(summary)]
    return "\n".join(lines)


def drive_problems(summary: dict) -> list[str]:
    """한 대 주행(drive_check)이 주행 완료 기준을 못 지킨 것들. 비면 통과."""
    results = summary.get("results", [])
    problems = [f"{r['step']}: {r['result']}" for r in results if r["result"] != "success"]
    planned = summary.get("steps") or 0
    if not results:
        problems.append("이동을 하나도 하지 않았다")
    elif planned <= 0:
        problems.append("요약에 계획한 이동 수(steps)가 없다")
    elif planned != len(results):
        problems.append(f"이동 {planned - len(results)}개는 하지 않았다(첫 실패에서 멈춘다)")
    for r in results:
        if r.get("truth_keepout"):
            problems.append(f"{r['step']}: 로봇 중심이 keepout에 들어갔다 {r['truth_keepout']}")
        if r.get("plan_keepout"):
            problems.append(f"{r['step']}: Nav2 계획이 keepout을 지났다 {r['plan_keepout']}")
    return problems


def record_problems(run, finished: bool) -> list[str]:
    """경기가 끝까지 돌았고 주문을 받았나. run은 관제 기록(triage Run), finished는 시나리오
    모니터가 끝을 적었나(summary.json): 일찍 죽은 launch의 기록에는 흠이 없어 보인다."""
    problems = []
    if not finished:
        problems.append("경기가 끝나지 않았다(summary.json 없음: 시간 한도나 launch 실패)")
    if not run.orders:
        problems.append("관제 기록에 주문이 없다")
    return problems


def game_problems(rows, check: dict | None, keepout_max_m: float = KEEPOUT_MAX_M) -> list[str]:
    """세 대 경기가 못 지킨 것들. 비면 통과.

    rows는 끝나지 않은 주문의 원인(tools/triage), check는 fleet_check.json의 내용이다."""
    problems = [f"{r.order_id}: {r.cause} ({r.event})" for r in rows if r.cause == DRIVING]
    depth = None if check is None else check.get("keepout_depth_m")
    if depth is None:
        problems.append("keepout 침범 깊이를 재지 못했다(fleet_check.json)")
    elif depth > keepout_max_m:
        problems.append(f"몸이 keepout 구역에 {depth:.3f} m 들어갔다(기준 {keepout_max_m} m): "
                        f"{check.get('keepout_robot')}, {check.get('keepout_zone')}, "
                        f"{check.get('keepout_at_s')} s")
    return problems


def _report(problems: list[str]) -> int:
    print("\n통과" if not problems else "\n못 지킨 기준:\n" + "\n".join(f"- {p}" for p in problems))
    return 1 if problems else 0


def _drive(args) -> int:
    summary = json.loads(Path(args.path).read_text(encoding="utf-8"))
    print(drive_markdown(summary))
    return _report(drive_problems(summary))


def _game(args) -> int:
    folder = Path(args.path)
    record, check_file = folder / "fleet_record.jsonl", folder / "fleet_check.json"
    if not record.is_file():
        return _report([f"관제 기록이 없다: {record} (gz.launch.py fleet_record:=true)"])
    obstacles = read_obstacles(args.obstacles) if args.obstacles else ()
    run = read_fleet_record(record)
    rows = classify(run, obstacles)
    check = json.loads(check_file.read_text(encoding="utf-8")) if check_file.is_file() else None
    print(causes_markdown(rows))
    if check is not None and check.get("keepout_depth_m") is not None:
        print(f"\nkeepout 침범 깊이: {check['keepout_depth_m']:.3f} m")
    return _report(record_problems(run, (folder / "summary.json").is_file())
                   + game_problems(rows, check, args.keepout_max))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="kind", required=True)
    sub.add_parser("drive").add_argument("path")
    game = sub.add_parser("game")
    game.add_argument("path")
    game.add_argument("--obstacles", help="장애물 yaml(tools/triage와 같다)")
    game.add_argument("--keepout-max", type=float, default=KEEPOUT_MAX_M)
    args = parser.parse_args(argv)
    return _drive(args) if args.kind == "drive" else _game(args)


if __name__ == "__main__":
    sys.exit(main())
