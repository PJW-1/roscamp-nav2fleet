"""eta_error.py의 표본, 실제 남은 시간, 두 예측 계산을 시험한다."""
import importlib.util
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))
spec = importlib.util.spec_from_file_location("eta_error", HERE / "eta_error.py")
eta_error = importlib.util.module_from_spec(spec)
spec.loader.exec_module(eta_error)

from fleet_core.common.commands import MoveOrderCmd, OrderRecord  # noqa: E402
from fleet_core.common.params import load_params  # noqa: E402
from fleet_core.fleet_map import load_fleet_map  # noqa: E402
from fleet_core.manager import FleetManager  # noqa: E402

ON = replace(load_params(), use_order_model=True)


def record(tmp_path, with_distance):
    """pode1이 P-1에서 N_W까지 첫 Segment(nav_goal)를 4 s에 곧게 가고, 끝낸 seq 1을 알린다."""
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
        left = math.dist((x, y), end) + 0.12  # Nav2 길은 곧은 거리보다 12 cm 길다
        extra = (left,) if with_distance else ()
        core.on_move_feedback("pode1", order.goal_id, 0, False, "", *extra)
        core.tick()
    now[0] += 0.1
    core.on_move_feedback("pode1", order.goal_id, 1, False, "")
    core.tick()
    path = tmp_path / "fleet_record.jsonl"
    path.write_text("".join(line + "\n" for line in lines), encoding="utf-8")
    return path, t0 + 4.1


@pytest.mark.parametrize("with_distance", [False, True])
def test_samples_carry_the_actual_time_to_segment_done_and_both_estimates(tmp_path,
                                                                          with_distance):
    path, done = record(tmp_path, with_distance)
    rows, differ = eta_error.samples(path, every=0.5)
    assert differ is None, "the record plays back the same"
    assert len(rows) >= 7 and {r["segment"] for r in rows} == {"pode1-1 seq 1"}
    for r in rows:
        assert r["actual"] == pytest.approx(done - r["t"])
        if with_distance and r["from_distance"]:
            # 표본은 위치 보고 때라 남은 거리는 한 걸음(0.1 s) 전 값이다
            assert r["used"] == pytest.approx(r["proj"] + 0.12 / ON.nav_speed_mps, abs=0.15)
        else:
            assert r["used"] == pytest.approx(r["proj"])
    assert any(r["from_distance"] for r in rows) == with_distance


def test_the_summary_bins_by_the_actual_time_left(tmp_path):
    path, _ = record(tmp_path, True)
    rows, _ = eta_error.samples(path)
    table = eta_error.summary(rows)
    assert table["all"]["samples"] == len(rows)
    assert sum(table[name]["samples"] for name in table if name != "all") == len(rows)
    assert table["all"]["used_mean_abs_s"] is not None


def test_the_command_line_prints_one_table_per_run_and_one_for_all(tmp_path, capsys):
    path, _ = record(tmp_path, True)
    assert eta_error.main([f"a={path.parent}", "--per-run"]) == 0
    out = capsys.readouterr().out
    assert "### a" in out and "### all 1 runs" in out and "| 0-1 s |" in out
