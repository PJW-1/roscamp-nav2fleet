"""로봇단 교착 처리(추가 지시서 3의 7-4, 기준(사용자), 계약 0.13-draft)의 관제 쪽.

관제는 MoveTo에 priority를 싣고 /fleet/robots에 마지막으로 보낸 값을 보인다. 로봇이 RECOVERING과
local_yield(wait/backup/retry)를 보고하면 사건 LOCAL_YIELD로 남기기만 한다. 움직임(HALL_YIELD,
막힘 표시)은 BLOCKED 뒤다.
"""
import math
from dataclasses import replace

import pytest
from fleet_core.common.commands import CancelCmd, MoveCmd, MoveOrderCmd
from fleet_core.common.params import load_params
from fleet_core.traffic.deadlock import priority
from zone_harness import Harness


def called(robot="pinky2", **kwargs):
    world = Harness(**kwargs)
    world.step()
    world.call("WORK", robot)
    world.step()
    assert world.moves(robot)
    return world


def reporting(world, robot, said):
    """robot은 앞으로 said[0](state), said[1](local_yield)을 보고한다. said는 바꿀 수 있다."""
    def report():
        for bot in world.bots.values():
            if bot.name in world.silent:
                continue
            if bot.name == robot:
                world.traffic.on_robot_state(bot.name, bot.x, bot.y, bot.yaw, said[0],
                                             local_yield=said[1])
            else:
                world.traffic.on_robot_state(bot.name, bot.x, bot.y, bot.yaw, world._state(bot))
    world.report = report


def test_a_move_carries_the_priority_the_fleet_yields_by_and_the_status_shows_it():
    world = called()
    [cmd] = world.sent((MoveCmd, MoveOrderCmd), "pinky2")
    p = world.params
    m = world.traffic._robots["pinky2"]
    assert cmd.priority == priority(p, False, m.dispatch is not None) >= 0
    assert world.traffic.status_view("pinky2")["priority"] == cmd.priority


def test_a_robot_never_sent_a_move_shows_no_priority():
    world = called()
    assert world.traffic.status_view("pinky1")["priority"] == -1


def test_an_order_goal_carries_the_priority_too():
    world = called(params=replace(load_params(), use_order_model=True))
    [cmd] = world.sent(MoveOrderCmd, "pinky2")
    assert cmd.priority >= 0
    assert world.traffic.status_view("pinky2")["priority"] == cmd.priority


def test_the_robot_side_yield_is_recorded_step_by_step_and_not_acted_on():
    world = called(speed=0.0)
    goal = world.view("pinky2")["goal_id"]
    said = ["RECOVERING", "wait"]
    reporting(world, "pinky2", said)
    world.run(2.0)
    said[1] = "backup"
    world.run(2.0)
    said[1] = "retry"
    world.run(2.0)
    said[:] = ["BLOCKED", ""]
    world.step()
    details = [e.detail for e in world.events if e.code == "LOCAL_YIELD"]
    assert details[:3] == ["wait", "backup", "retry"]
    assert details[3].startswith("ends BLOCKED after 6")
    assert "SELF_RECOVERY" not in world.codes("pinky2")
    assert not world.sent(CancelCmd, "pinky2") and world.view("pinky2")["goal_id"] == goal
    assert world.codes("pinky2").count("BLOCKED_WAIT") == 1  # 관제 처리는 BLOCKED 뒤에 시작한다


def test_recovering_without_a_step_is_still_the_old_self_recovery():
    world = called(speed=0.0)
    said = ["RECOVERING", ""]
    reporting(world, "pinky2", said)
    world.run(1.0)
    assert "SELF_RECOVERY" in world.codes("pinky2")
    assert "LOCAL_YIELD" not in world.codes("pinky2")


# ---- FleetManager가 받는 칸(계약 0.13-draft) ----

def manager():
    from fleet_core.fleet_map import load_fleet_map
    from fleet_core.manager import FleetManager
    lines = []
    core = FleetManager(load_fleet_map("d2"), load_params(), ["pinky1"], lambda: 0.0)
    core.record_to(lines.append)
    return core, lines


def test_the_manager_passes_the_step_and_records_it_after_the_obstacle_fields():
    core, lines = manager()
    core.on_robot_state("pinky1", 0.13, 0.15, 0.0, "RECOVERING", 90.0, "", math.nan, math.nan,
                        "", "wait")
    assert core.traffic._robots["pinky1"].local_yield == "wait"
    assert '"args": ["pinky1", 0.13, 0.15, 0.0, "RECOVERING", 90.0, "", NaN, NaN, "", "wait"]' in (
        lines[1])


@pytest.mark.parametrize("step", ["", 7, "jump"])
def test_an_empty_or_unknown_step_is_no_step_and_an_empty_one_is_recorded_as_before(step):
    core, lines = manager()
    core.on_robot_state("pinky1", 0.13, 0.15, 0.0, "RECOVERING", 90.0, "", math.nan, math.nan,
                        "", step)
    assert core.traffic._robots["pinky1"].local_yield == ""
    if step == "":
        assert '"args": ["pinky1", 0.13, 0.15, 0.0, "RECOVERING", 90.0]}' in lines[1]


def test_the_status_view_shows_the_priority():
    core, _ = manager()
    [view] = core.robots_view()
    assert view.priority == -1
