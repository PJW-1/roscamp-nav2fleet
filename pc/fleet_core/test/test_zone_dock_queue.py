"""한 도크를 기다리는 로봇이 대기 자리보다 많을 때의 규칙(turns.py)을 고정한다.
도크 차례는 바란 순서, 대기 자리는 그 주기에 먼저 보는 로봇이 잡고, 도크가 비면 차례인 로봇이 간다."""
from zone_harness import DT, Harness


def targets(world, robot):
    return [c.target_id for c in world.legs(robot) if c.mode != "retreat"]


def test_more_robots_than_wait_spots_take_the_dock_in_the_order_they_asked():
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"})
    world.report()
    busy = world.call("WORK", "pode1")
    assert world.run_until(lambda: world.result_of(busy) is not None, 30.0)
    first = world.call("WORK", "pode3")
    world.now += DT / 2  # pode3이 먼저 바랐다
    second = world.call("WORK", "pode2")
    world.run(30.0)
    assert world.view("pode2")["node"] == "W_Q", "the wait spot goes to the robot seen first"
    queued = [e.detail for e in world.events if e.code == "DOCK_QUEUED" and e.robot == "pode3"]
    assert queued == ["WORK is busy, waits at P-3"], "the other one waits where it is parked"
    assert targets(world, "pode3") == []
    world.traffic.on_release("pode1")
    assert world.run_until(lambda: world.result_of(first) is not None, 120.0)
    assert world.result_of(first).robot_id == "pode3", "the first to ask docks first"
    assert world.result_of(second) is None and targets(world, "pode2") == ["W_Q"]
    world.traffic.on_release("pode3")
    assert world.run_until(lambda: world.result_of(second) is not None, 200.0)
    assert world.result_of(second).robot_id == "pode2"
    assert not world.problems
