"""머리 없는 세계의 low_battery와 random 시나리오: 배터리 값이 관제 입력으로 간다."""
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import baseline  # noqa: E402
from sim_world import SimWorld  # noqa: E402

pytest.importorskip("fake_robots")


def batteries(world, seconds):
    """로봇마다 on_robot_state에 넘긴 battery 값(안 넘겼으면 None)을 차례로."""
    seen: dict[str, list] = {}
    report = world.core.on_robot_state

    def spy(robot, *args, **kwargs):
        seen.setdefault(robot, []).append(kwargs.get("battery"))
        return report(robot, *args, **kwargs)

    world.core.on_robot_state = spy
    world.run(seconds)
    return seen


def test_a_low_battery_reaches_the_core_and_charges_back_at_the_parking_spot():
    """pinky1은 P-6(주차 = 충전 자리)에 서 있다. 1 s에 10이 되고 2 s 동안 90까지 찬다."""
    world = SimWorld(orders=[], faults=[{"type": "low_battery", "robot": "pinky1", "at_s": 1.0,
                                         "duration_s": 2.0}])
    seen = batteries(world, 4.0)
    levels = seen["pinky1"]
    assert levels[0] == 90.0 and min(levels) == pytest.approx(10.0, abs=4.0)
    low = levels.index(min(levels))
    assert levels[low:] == sorted(levels[low:]) and levels[-1] == 90.0
    assert any(20.0 < v < 80.0 for v in levels)


def test_robots_without_a_low_battery_report_as_before():
    world = SimWorld(orders=[], faults=[{"type": "low_battery", "robot": "pinky1", "at_s": 1.0,
                                         "duration_s": 2.0}])
    seen = batteries(world, 2.0)
    assert set(seen["pinky2"]) == {None} and set(seen["pinky3"]) == {None}
    assert set(batteries(SimWorld(orders=[]), 1.0)["pinky1"]) == {None}


def random_scenario(tmp_path) -> str:
    data = baseline.load_scenario("basic_orders")
    data["random"] = {"order_gap_s": [5, 40],
                      "low_battery": {"from_s": 20, "to_s": 150, "charge_s": 30}}
    data["expect"] = {**(data.get("expect") or {}), "orders_shipped_min": 1}
    path = tmp_path / "random_orders.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return str(path)


def test_baseline_runs_also_draw_the_timetable_with_the_run_seed(tmp_path):
    scenario = random_scenario(tmp_path)
    worlds = [baseline.run_with_world("lane_map", scenario, seed, "path",
                                                    timeout_s=0.2)[1] for seed in (1, 2)]
    assert all(w is not None for w in worlds)
    assert worlds[0].orders != worlds[1].orders
    assert all(sum(f["type"] == "low_battery" for f in w.faults) == 3 for w in worlds)
