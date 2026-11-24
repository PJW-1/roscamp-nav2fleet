"""머리 없는 세계의 실행을 기록으로 받아 원인을 분류한다."""
from __future__ import annotations

import sys
from pathlib import Path

from .classify import classify
from .readers import obstacles_from_injections, read_fleet_record

REPO = Path(__file__).resolve().parents[2]


def _paths() -> None:
    for sub in ("pc/fleet_core", "sim/fake_robots"):
        if str(REPO / sub) not in sys.path:
            sys.path.insert(0, str(REPO / sub))


def recording_factory(lines: list, factory=None):
    """세계의 manager_factory. 만든 관제가 모든 입력과 결과를 JSON 줄로 lines에 쓴다."""
    _paths()
    from fake_robots.world.world import default_manager
    make = factory or default_manager

    def build(site, params, robots, clock):
        manager = make(site, params, robots, clock)
        manager.record_to(lines.append)
        return manager
    return build


def rows_of_lines(lines, obstacles=(), **override):
    return classify(read_fleet_record(lines), obstacles, **override)


def run_triaged(name, layout="d2", seed=1, overrides=None, **override):
    """상황 하나를 돌리고 (Outcome, 끝나지 않은 주문의 Row들)."""
    _paths()
    from fake_robots.world.situations import build
    from fake_robots.world.table import run_situation
    lines: list = []
    outcome = run_situation(name, layout, seed, overrides,
                            manager_factory=recording_factory(lines))
    obstacles = obstacles_from_injections(build(name, layout).injections)
    return outcome, rows_of_lines(lines, obstacles, **override)
