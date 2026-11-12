"""상황 시험(지시서 6-2)을 상황 x 배치 x 씨드로 돌려 결과 표를 markdown으로 찍는다.

    python3 tools/situations.py                       # 열 상황, d2 d c, 씨드 1 2 3
    python3 tools/situations.py -s order_burst -l d2 --seeds 1 2 --set use_order_model=true
    python3 tools/situations.py --avoid all           # 회피 흉내 none, perfect, mixed 모두
    python3 tools/situations.py --store --avoid all   # 매장 상황(추가 지시서 3의 5절)
    python3 tools/situations.py --store --set blocked_by_position=false   # 4절 전(비교)

--set 이름=값은 플릿 파라미터를 바꾼다(값은 yaml로 읽는다). 못 지킨 기대는 표 아래에 적는다.
주문을 다 못 끝낸 실행은 같은 씨드로 한 번 더 돌려 관제 기록을 받고, 원인 분류(tools/triage)가
"끝나지 않은 주문 원인" 칸을 채운다: `원인 건수 (주문 id들)`.
머리 없는 구역 세계(sim/fake_robots/fake_robots/world)와 FleetManager(traffic_mode zones)를 쓴다.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
for sub in ("pc/fleet_core", "sim/fake_robots", "tools"):
    if str(REPO / sub) not in sys.path:
        sys.path.insert(0, str(REPO / sub))

from fake_robots.world.avoid import AVOID_MODES  # noqa: E402
from fake_robots.world.situations import NAMES, build  # noqa: E402
from fake_robots.world.store_situations import STORE_NAMES, STORE_ROWS, applies  # noqa: E402
from fake_robots.world.table import markdown, run_situation  # noqa: E402
from fleet_core.fleet_map import KNOWN_LAYOUTS, LAYOUTS, load_fleet_map  # noqa: E402
from triage import obstacles_from_injections, recording_factory, rows_of_lines  # noqa: E402


def overrides(pairs) -> dict:
    out = {}
    for pair in pairs or ():
        name, _, value = pair.partition("=")
        if not name or not _:
            raise SystemExit(f"--set wants name=value, not {pair!r}")
        out[name] = yaml.safe_load(value)
    return out


def avoid_modes(chosen) -> list[str]:
    """--avoid 값: all이면 세 모드 모두."""
    return list(AVOID_MODES) if "all" in chosen else list(chosen)


def open_order_causes(name, layout, seed, params, mode) -> str:
    """끝나지 않은 주문들의 원인을 한 칸으로: `주행 2 (O-004 O-005); 회피 1 (O-003)`."""
    lines: list = []
    run_situation(name, layout, seed, overrides=params, avoid_mode=mode,
                  manager_factory=recording_factory(lines))
    injections = build(name, layout).for_mode(mode).injections
    by_cause: dict = {}
    for found in rows_of_lines(lines, obstacles_from_injections(injections)):
        by_cause.setdefault(found.cause, []).append(found.order_id)
    return "; ".join(f"{cause} {len(ids)} ({' '.join(ids)})" for cause, ids in by_cause.items())


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-s", "--situations", nargs="+", default=None,
                        choices=[*NAMES, *STORE_NAMES])
    parser.add_argument("--store", action="store_true",
                        help="매장 상황(추가 지시서 3의 5절)을 돌린다. -s를 주지 않았을 때만")
    parser.add_argument("-l", "--layouts", nargs="+", default=list(LAYOUTS),
                        choices=KNOWN_LAYOUTS, help="기본 d2 d c, 시험용은 d2_shelf")
    parser.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3])
    parser.add_argument("--set", action="append", metavar="NAME=VALUE")
    parser.add_argument("--avoid", nargs="+", default=["perfect"],
                        choices=[*AVOID_MODES, "all"], help="회피 흉내 모드(기본 perfect)")
    args = parser.parse_args(argv)
    params = overrides(args.set)
    names = args.situations or list(STORE_ROWS if args.store else NAMES)
    rows, notes = [], []
    for name in names:
        for layout in args.layouts:
            if not applies(name, layout, load_fleet_map(layout)):
                continue  # 통로 안 상자는 통로가 있는 배치에서만
            for mode in avoid_modes(args.avoid):
                for seed in args.seeds:
                    outcome = run_situation(name, layout, seed, overrides=params,
                                            avoid_mode=mode)
                    central = outcome.world.central
                    causes = (open_order_causes(name, layout, seed, params, mode)
                              if central.finished < central.total else "")
                    rows.append((*outcome.row[:-1], causes))
                    notes += [f"- {name} {layout} {mode} {seed}: {f}"
                              for f in outcome.failures[:5]]
    print(markdown(rows))
    if notes:
        print("\n못 지킨 기대:\n" + "\n".join(notes))
    return 1 if notes else 0


if __name__ == "__main__":
    sys.exit(main())
