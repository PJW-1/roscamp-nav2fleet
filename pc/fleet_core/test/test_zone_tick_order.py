"""하네스의 ZoneTraffic.tick과 운영 FleetManager.tick이 교통 결과를 같게 내는지 본다.

하네스에는 주문 층과 팔의 일이 없으므로 두 순서의 차이(_pump로 같은 틱 안에 주문 층과 팔이
교통에 답하는 것)가 드러날 길이 없다. 그 전제가 깨지면 이 시험이 먼저 알려 준다.
"""
from __future__ import annotations

import pytest
from fleet_core.common.commands import (
    ArmFillCmd,
    DispatchCmd,
    OrderRecord,
    ReleaseCmd,
    RobotFaultCmd,
)
from zone_harness import Harness
from zone_scenarios import SCENARIOS

ORDERS = ("zone", "manager")


def _play(name: str, order: str) -> Harness:
    scenario, starts = SCENARIOS[name]
    world = Harness(starts=starts, tick_order=order)
    assert scenario(world), f"{name}/{order} did not finish:\n" + "\n".join(world.log[-30:])
    return world


def _silent_robot(order: str) -> Harness:
    # 통신 끊김: 달리던 로봇이 말을 끊었다가 돌아온다
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"}, tick_order=order)
    world.report()
    call = world.call("OUTBOUND", "pode1")
    world.run(3.0)
    world.silent.add("pode1")
    world.run(20.0)
    world.silent.discard("pode1")
    world.run_until(lambda: world.result_of(call) is not None, 200.0)
    return world


def _fault_and_clear(order: str) -> Harness:
    # 고장 격리, 운영자 해제, 다른 로봇이 호출을 받는다
    world = Harness(tick_order=order)
    world.step()
    first = world.call("WORK")
    world.run(4.0)
    robot = next(r for r in world.bots if world.view(r)["destination"] == "WORK")
    world.traffic.on_robot_fault(robot, "test")
    world.run(2.0)
    world.traffic.confirm_clear(robot, list(world.traffic.grants.held_by(robot)), "moved")
    second = world.call("WORK")
    assert world.run_until(lambda: world.result_of(first) is not None, 100.0)
    world.traffic.on_release(world.result_of(first).robot_id)
    assert world.run_until(lambda: world.result_of(second) is not None, 100.0)
    return world


def _stop_and_run(order: str) -> Harness:
    # 정지는 목표를 취소하고 다시 달리기는 이어 간다
    world = Harness(tick_order=order)
    world.step()
    call = world.call("WORK")
    world.run(4.0)
    world.traffic.on_run_mode("STOP")
    world.run(3.0)
    world.traffic.on_run_mode("RUN")
    world.run_until(lambda: world.result_of(call) is not None, 300.0)
    return world


CASES = {
    **{name: (lambda order, name=name: _play(name, order)) for name in SCENARIOS},
    "silent_robot": _silent_robot,
    "fault_and_clear": _fault_and_clear,
    "stop_and_run": _stop_and_run,
}


def _trace(world: Harness) -> tuple:
    return (world.log, world.problems,
            [(r.goal_id, r.success, r.robot_id, r.reason) for r in world.results])


@pytest.mark.parametrize("case", sorted(CASES))
def test_both_tick_orders_give_the_same_traffic(case):
    zone, manager = (CASES[case](order) for order in ORDERS)
    assert zone.tick_order == "zone" and manager.tick_order == "manager"
    assert zone.problems == []
    assert _trace(manager) == _trace(zone)


def test_the_manager_order_really_runs_fleet_manager_tick():
    world = Harness(tick_order="manager")
    calls = []
    real = type(world.manager).tick

    def counted(core):
        calls.append(core)
        return real(core)

    world.manager.tick = lambda: counted(world.manager)
    world.run(0.5)
    assert len(calls) == 5 and world.traffic is world.manager.traffic


def test_an_unknown_tick_order_is_refused():
    with pytest.raises(ValueError):
        Harness(tick_order="other")


# ---- 의도된 차이: 운영에서는 주문 층의 답이 같은 틱 안에 교통으로 돌아온다 ----

def _inside(world: Harness, kind) -> list:
    return [c for c in world.commands if isinstance(c, kind)]


def test_in_production_an_order_on_the_board_becomes_a_call_inside_the_same_tick():
    world = Harness(tick_order="manager")
    world.step()
    world.manager.on_order_board("RUN", [OrderRecord("o1", {"red": 1})])
    world.step()
    calls = _inside(world, DispatchCmd)
    assert len(calls) == 1
    # 주문 층이 낸 호출을 교통이 같은 틱에 받아 로봇을 정했다
    assert any(world.view(r)["destination"] == "WORK" for r in world.bots)


def test_in_production_the_order_layer_answers_traffic_within_the_tick_it_hears():
    world = Harness(tick_order="manager")
    world.step()
    world.manager.on_order_board("RUN", [OrderRecord("o1", {"red": 1})])
    assert world.run_until(lambda: _inside(world, ArmFillCmd), 300.0), "\n".join(world.log[-20:])
    robot = _inside(world, ArmFillCmd)[0].robot
    # 도착 결과와 팔 명령이 같은 틱 기록에 함께 있다
    arrived = next(i for i, line in enumerate(world.log) if "DispatchResult" in line)
    asked = next(i for i, line in enumerate(world.log) if "ArmFillCmd" in line)
    assert world.log[arrived].split()[0] == world.log[asked].split()[0]
    assert not _inside(world, RobotFaultCmd) and not _inside(world, ReleaseCmd)
    assert world.phase(robot) == "AT_STATION"
