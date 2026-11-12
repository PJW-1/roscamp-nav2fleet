"""상황 시험(지시서 6-2)을 상황 x 배치 x 씨드로 돌려 결과 표를 markdown으로 찍는다.

    python3 tools/situations.py                       # 열 상황, d2 d c, 씨드 1 2 3
    python3 tools/situations.py -s order_burst -l d2 --seeds 1 2 --set use_order_model=true
    python3 tools/situations.py --avoid all           # 회피 흉내 none, perfect, mixed 모두

--set 이름=값은 플릿 파라미터를 바꾼다(값은 yaml로 읽는다). 못 지킨 기대는 표 아래에 적는다.
머리 없는 구역 세계(sim/fake_robots/fake_robots/world)와 FleetManager(traffic_mode zones)를 쓴다.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
for sub in ("pc/fleet_core", "sim/fake_robots"):
    if str(REPO / sub) not in sys.path:
        sys.path.insert(0, str(REPO / sub))

from fake_robots.world.avoid import AVOID_MODES  # noqa: E402
from fake_robots.world.situations import LAYOUTS, NAMES  # noqa: E402
from fake_robots.world.table import markdown, run_situation  # noqa: E402


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


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-s", "--situations", nargs="+", default=list(NAMES), choices=NAMES)
    parser.add_argument("-l", "--layouts", nargs="+", default=list(LAYOUTS), choices=LAYOUTS)
    parser.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3])
    parser.add_argument("--set", action="append", metavar="NAME=VALUE")
    parser.add_argument("--avoid", nargs="+", default=["perfect"],
                        choices=[*AVOID_MODES, "all"], help="회피 흉내 모드(기본 perfect)")
    args = parser.parse_args(argv)
    params = overrides(args.set)
    rows, notes = [], []
    for name in args.situations:
        for layout in args.layouts:
            for mode in avoid_modes(args.avoid):
                for seed in args.seeds:
                    outcome = run_situation(name, layout, seed, overrides=params,
                                            avoid_mode=mode)
                    rows.append(outcome.row)
                    notes += [f"- {name} {layout} {mode} {seed}: {f}"
                              for f in outcome.failures[:5]]
    print(markdown(rows))
    if notes:
        print("\n못 지킨 기대:\n" + "\n".join(notes))
    return 1 if notes else 0


if __name__ == "__main__":
    sys.exit(main())
