"""하네스의 ZoneTraffic.tick과 운영 FleetManager.tick이 교통 결과를 같게 내는지 본다. 아래쪽 시험은 운영 순서의 입력 경계와 틱 경계를 매니저 입력 함수로 고정한다."""
from __future__ import annotations

import os

import pytest
import zone_harness
from fleet_core.common.commands import (
    FILL,
    ArmFault,
    ArmFillCmd,
    ArmLost,
    DispatchCmd,
    DispatchResult,
    MoveCmd,
    MoveOrderCmd,
    OrderRecord,
    ReleaseCmd,
    ReportOrderCmd,
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
    assert _seen(world, (MoveCmd, MoveOrderCmd))[0] == ("tick", 2)
    assert any(world.view(r)["destination"] == "WORK" for r in world.bots)


def test_in_production_the_arm_job_starts_in_the_tick_after_the_dock_result():
    world = _ordered_world()
    assert world.run_until(lambda: _seen(world, ArmFillCmd), 300.0), "\n".join(world.log[-20:])
    [(phase, n)] = [s for s in _seen(world, DispatchResult)]
    # 도킹 결과는 on_dock_result 입력 때 주문 층에 닿고,
    # 팔 명령은 다음 틱의 orchestrator.step이 낸다
    assert phase == "input"
    assert _seen(world, ArmFillCmd) == [("tick", n + 1)]
    assert not _seen(world, RobotFaultCmd) and not _seen(world, ReleaseCmd)


def _items(world: Harness, kind) -> list:
    """(단계, 틱 번호, 항목) 목록."""
    return [entry for entry in world.manager.seen if isinstance(entry[2], kind)]


def test_in_production_an_arm_timeout_holds_the_order_inside_the_same_tick():
    world = _ordered_world()
    limit = world.params.fill_timeout_s + world.params.arm_cancel_wait_s + 120.0
    # 하네스는 팔에 답하지 않으므로 채우기는 끝내 시간이 다 된다
    assert world.run_until(lambda: _seen(world, ArmFault), limit), "\n".join(world.log[-20:])
    [(_, asked_n, asked)] = _items(world, ArmFillCmd)
    [(lost_phase, lost_n, lost)] = _items(world, ArmLost)
    [(fault_phase, fault_n, fault)] = _items(world, ArmFault)
    robot = asked.robot
    # 앞서 이 로봇에 채우기 팔 작업이 있었고, 그 작업이 답 없이 끝났다
    assert asked_n < fault_n
    assert lost.robot == robot and lost.kind == FILL
    assert fault.robot == robot and fault.kind == FILL
    assert fault.reason == f"{lost.server} goal {lost.goal_id} never reported"
    assert any(e.code == "ARM_NO_RESULT" and e.robot == robot for e in world.events)
    # 팔 고장은 같은 틱 안에 주문을 보류하고 Pode는 격리하지 않는다
    [(report_phase, report_n, report)] = _items(world, ReportOrderCmd)
    assert (report.status, report.hold_reason) == ("HELD", "ARM_FAULT")
    assert lost_phase == fault_phase == report_phase == "tick"
    assert lost_n == fault_n == report_n
    assert not _seen(world, RobotFaultCmd) and not _seen(world, RobotIsolated)
    assert world.traffic.available(robot)
