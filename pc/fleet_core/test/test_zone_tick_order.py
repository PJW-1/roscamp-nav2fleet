"""하네스의 ZoneTraffic.tick과 운영 FleetManager.tick이 교통 결과를 같게 내는지 본다.

하네스의 교통 전용 시나리오에는 주문 층과 팔의 일이 없으므로 두 순서의 차이(_pump로 같은 틱
안에 주문 층과 팔이 교통에 답하는 것)가 드러날 길이 없다. 그 전제가 깨지면 이 시험이 먼저 알려 준다.
아래쪽 시험은 운영 순서에서 입력 경계와 틱 경계가 실제로 어디인지 매니저 입력 함수를 거쳐 고정한다.
"""
from __future__ import annotations

import os

import pytest
import zone_harness
from fleet_core.common.commands import (
    ArmFillCmd,
    DispatchCmd,
    DispatchResult,
    MoveCmd,
    OrderRecord,
    ReleaseCmd,
    RobotFaultCmd,
    RobotIsolated,
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
    with pytest.raises(ValueError):
        Harness(tick_order="zone", manager_inputs=True)


@pytest.mark.skipif("ZONE_TICK_ORDER" in os.environ, reason="the order was chosen for this run")
def test_the_harness_runs_the_production_order_unless_told_otherwise():
    assert zone_harness.TICK_ORDER == "manager"
    assert Harness().manager is not None


# ---- 운영 순서에서 입력과 틱의 경계: 매니저 입력 함수(+_pump)를 거쳐 단계별로 본다 ----

def _seen(world: Harness, kind) -> list:
    """(단계, 틱 번호) 목록. 단계는 매니저 입력 중이면 input, FleetManager.tick 안이면 tick."""
    return [(phase, n) for phase, n, item in world.manager.seen if isinstance(item, kind)]


def _ordered_world() -> Harness:
    world = Harness(tick_order="manager", manager_inputs=True)
    world.step()
    world.manager.on_order_board("RUN", [OrderRecord("o1", {"red": 1})])
    return world


def test_in_production_a_board_order_reaches_traffic_at_once_and_the_next_tick_picks_a_robot():
    world = _ordered_world()
    # on_order_board가 바로 _pump하므로 호출은 틱이 아니라 입력 때 교통에 닿는다
    assert _seen(world, DispatchCmd) == [("input", 1)]
    assert not any(world.view(r)["destination"] == "WORK" for r in world.bots)
    world.step()
    assert _seen(world, MoveCmd)[0] == ("tick", 2)
    assert any(world.view(r)["destination"] == "WORK" for r in world.bots)


def test_in_production_the_arm_job_starts_in_the_tick_after_the_dock_result():
    world = _ordered_world()
    assert world.run_until(lambda: _seen(world, ArmFillCmd), 300.0), "\n".join(world.log[-20:])
    [(phase, n)] = [s for s in _seen(world, DispatchResult)]
    # 도킹 결과는 on_dock_result 입력 때 주문 층에 닿고, 팔 명령은 다음 틱의 orchestrator.step이 낸다
    assert phase == "input"
    assert _seen(world, ArmFillCmd) == [("tick", n + 1)]
    assert not _seen(world, RobotFaultCmd) and not _seen(world, ReleaseCmd)


def test_in_production_an_arm_timeout_isolates_the_robot_inside_the_same_tick():
    world = _ordered_world()
    limit = world.params.fill_timeout_s + world.params.arm_cancel_wait_s + 120.0
    # 하네스는 팔에 답하지 않으므로 채우기는 끝내 시간이 다 된다
    assert world.run_until(lambda: _seen(world, RobotIsolated), limit), "\n".join(world.log[-20:])
    [fault] = _seen(world, RobotFaultCmd)
    [isolated] = _seen(world, RobotIsolated)
    # arm.check_timeouts의 고장이 같은 틱의 _pump로 교통에 들어가 그 틱 안에 격리된다
    assert fault[0] == "tick" and isolated == fault
    robot = next(i.robot for _, _, i in world.manager.seen if isinstance(i, RobotFaultCmd))
    assert not world.traffic.available(robot)
