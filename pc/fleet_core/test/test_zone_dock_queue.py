"""한 도크를 기다리는 로봇이 대기 자리보다 많을 때(추가 지시서 2의 5절). 지금 규칙을 고정한다.

규칙(turns.py의 _candidates, _is_next, _stay): 대기 자리를 늘리지 않는다.
- 도크 차례는 바란 순서다(같은 때면 도크까지 가까운 쪽, 그다음 이름).
- 대기 자리는 비어 있으면 관제가 그 주기에 먼저 보는 로봇(로봇 목록 순서)이 잡는다. 차례 순서가
  아니다. 못 잡은 로봇은 지금 선 주차 자리에서 기다린다(DOCK_QUEUED).
- 도크가 비면 차례인 로봇이 어디 있든 간다. 대기 자리의 로봇이 차례가 아니면 거기 그대로 있다.
"""
from fleet_core.common.commands import MoveCmd
from zone_harness import DT, Harness


def targets(world, robot):
    return [c.target_id for c in world.sent(MoveCmd, robot)]


def test_more_robots_than_wait_spots_take_the_dock_in_the_order_they_asked():
    world = Harness(starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    world.report()
    busy = world.call("WORK", "pinky1")
    assert world.run_until(lambda: world.result_of(busy) is not None, 30.0)
    first = world.call("WORK", "pinky3")
    world.now += DT / 2  # pinky3이 먼저 바랐다
    second = world.call("WORK", "pinky2")
    world.run(30.0)
    assert world.view("pinky2")["node"] == "W_Q", "the wait spot goes to the robot seen first"
    queued = [e.detail for e in world.events if e.code == "DOCK_QUEUED" and e.robot == "pinky3"]
    assert queued == ["WORK is busy, waits at P-3"], "the other one waits where it is parked"
    assert targets(world, "pinky3") == []
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.result_of(first) is not None, 120.0)
    assert world.result_of(first).robot_id == "pinky3", "the first to ask docks first"
    assert world.result_of(second) is None and targets(world, "pinky2") == ["W_Q"]
    world.traffic.on_release("pinky3")
    assert world.run_until(lambda: world.result_of(second) is not None, 200.0)
    assert world.result_of(second).robot_id == "pinky2"
    assert not world.problems
