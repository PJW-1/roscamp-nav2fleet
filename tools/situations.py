"""상황 시험을 상황 x 배치 x 씨드로 돌려 결과 표를 markdown으로 찍는다.

    python3 tools/situations.py --store --avoid all   # --set 이름=값으로 플릿 파라미터를 바꾼다
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
for sub in ("pc/fleet_core", "sim/fake_robots", "tools", "pode/pode_fsm"):
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


def open_order_causes(name, layout, seed, params, mode, local_yield=False) -> str:
    """끝나지 않은 주문들의 원인을 한 칸으로: `주행 2 (O-004 O-005); 회피 1 (O-003)`."""
    lines: list = []
    run_situation(name, layout, seed, overrides=params, avoid_mode=mode,
                  manager_factory=recording_factory(lines), local_yield=local_yield)
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
                        help="매장 상황을 돌린다. -s를 주지 않았을 때만")
    parser.add_argument("-l", "--layouts", nargs="+", default=list(LAYOUTS),
                        choices=KNOWN_LAYOUTS, help="기본 d2 d c v7, 시험용은 d2_shelf")
    parser.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3])
    parser.add_argument("--set", action="append", metavar="NAME=VALUE")
    parser.add_argument("--avoid", nargs="+", default=["perfect"],
                        choices=[*AVOID_MODES, "all"], help="회피 흉내 모드(기본 perfect)")
    parser.add_argument("--local-yield", action="store_true",
                        help="가짜 로봇이 pode_fsm의 로봇단 양보 사다리를 흉내 낸다(기본 끔)")
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
                                            avoid_mode=mode, local_yield=args.local_yield)
                    central = outcome.world.central
                    causes = (open_order_causes(name, layout, seed, params, mode,
                                                args.local_yield)
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
