"""extra_checks.py: 코어 재생으로 남은 거리의 뜀을 코어가 본 그대로 찾는다.

사용: python3 -m pytest docs/distance_remaining_2026-11-21/test_extra_checks.py -q
"""
import importlib.util
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))
spec = importlib.util.spec_from_file_location("extra_checks", HERE / "extra_checks.py")
extra_checks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extra_checks)

from fleet_core.common.commands import MoveOrderCmd, OrderRecord  # noqa: E402
from fleet_core.common.params import load_params  # noqa: E402
from fleet_core.fleet_map import NAV_GOAL, load_fleet_map  # noqa: E402
from fleet_core.manager import FleetManager  # noqa: E402

ON = replace(load_params(), use_order_model=True)


def record(tmp_path, wait_at=None):
    """pode1이 P-1에서 첫 nav_goal Segment 끝까지 4 s에 곧게 간다. 2 s에 남은 거리가 0.3 m 뛴다."""
    now = [0.0]
    core = FleetManager(load_fleet_map("d2"), ON, ["pode1"], lambda: now[0])
    lines = []
    core.record_to(lines.append)
    start = core.site.node_pose("P-1")
    for _ in range(ON.release_confirm_samples):
        core.on_robot_state("pode1", *start, 0.0, "IDLE")
    core.on_order_board("RUN", [OrderRecord("O-001", {"A": 1})])
    order = None
    while order is None:
        assert now[0] < 5.0, "no order"
        now[0] += 0.1
        order = next((c for c in core.tick() if isinstance(c, MoveOrderCmd)), None)
    end = core.site.node_pose(order.base[0].end_node)
    t0 = now[0]
    for step in range(1, 41):
        now[0] = t0 + 0.1 * step
        k = step / 40
        x, y = start[0] + k * (end[0] - start[0]), start[1] + k * (end[1] - start[1])
        core.on_robot_state("pode1", x, y, 0.0, "MOVING")
        left = math.dist((x, y), end) + (0.3 if step >= 20 else 0.0)
        waiting = wait_at is not None and step == wait_at
        core.on_move_feedback("pode1", order.goal_id, 0, waiting, "", left)
        core.tick()
    path = tmp_path / "fleet_record.jsonl"
    path.write_text("".join(line + "\n" for line in lines), encoding="utf-8")
    return path, t0, start, end


def test_a_rise_over_the_jump_bar_is_reported_once_with_what_the_core_saw(tmp_path):
    path, t0, start, end = record(tmp_path)
    found = extra_checks.core_jumps(path)
    assert len(found) == 1
    jump = found[0]
    assert jump["t"] == pytest.approx(t0 + 2.0)
    assert jump["robot"] == "pode1" and jump["seq"] == 1 and jump["mode"] == NAV_GOAL
    straight = math.dist((start[0] + 0.5 * (end[0] - start[0]),
                          start[1] + 0.5 * (end[1] - start[1])), end)
    assert jump["new_m"] == pytest.approx(straight + 0.3)
    assert jump["old_m"] == pytest.approx(straight + math.dist(start, end) / 40)
    assert jump["straight_m"] == pytest.approx(straight, abs=1e-6)
    assert jump["old_usable"] is True
    assert jump["since_first_s"] == pytest.approx(1.9)


def test_a_waiting_feedback_in_between_clears_the_old_value_so_no_jump_is_seen(tmp_path):
    path, *_ = record(tmp_path, wait_at=19)
    assert extra_checks.core_jumps(path) == []
