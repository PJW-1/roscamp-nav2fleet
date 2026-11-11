"""머리 없는 세계의 low_battery: 배터리 값이 관제 입력으로 간다."""
import pytest
from sim_world import SimWorld

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
