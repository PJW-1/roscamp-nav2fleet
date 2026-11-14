"""v7 급행 서쪽 끝 대기 자리 O_X(사용자 결정 2026-11-14, 안 A).

W에서 출고로 가는 짐 실은 로봇은 출고가 바빠도 급행으로 간다. 도착했을 때 출고가 아직 바쁘면 급행
서쪽 끝 O_X(출고 바로 옆)에서 기다렸다 비면 들어간다. 그 전에는 출고가 바쁘면 넓은 곳의 O_IN으로
돌아가 23~25 s가 걸렸다(급행으로는 17 s, Gazebo). 한 로봇이 O_X에서 기다리는 동안에도 넓은 곳에서
오는 로봇은 O_IN에서 출고로 들어갈 수 있어야 한다(사용자 요청).
"""
from dataclasses import replace

from fleet_core.common.commands import MoveCmd
from fleet_core.common.params import load_params
from zone_harness import DT, Harness


def targets(world, robot):
    return [c.target_id for c in world.sent(MoveCmd, robot)]


def outbound_busy(starts, robots=("pinky1",), params=None):
    """pinky3이 출고 O에서 일하는 중(놓아 주기 전까지 서 있다). robots가 그 차례로 출고를 바란다.
    첫 걸음 전에 부른다: 할 일 없는 로봇은 첫 주기에 주차하러 떠난다."""
    world = Harness(layout="v7", starts=starts, params=params)
    world.report()
    busy = world.call("OUTBOUND", "pinky3")
    calls = []
    for robot in robots:
        world.now += DT / 2
        calls.append(world.call("OUTBOUND", robot))
    assert world.run_until(lambda: world.result_of(busy) is not None, 30.0), world.log[-10:]
    return world, *calls


def stands_at(world, robot, node):
    return lambda: world.view(robot)["node"] == node and world.bots[robot].cmd is None


def test_a_robot_at_work_waits_for_a_busy_outbound_at_the_west_end_of_the_express():
    world, call = outbound_busy({"pinky1": "W", "pinky2": "P-2", "pinky3": "O"})
    assert world.run_until(stands_at(world, "pinky1", "O_X"), 60.0), world.log[-10:]
    world.run(5.0)
    assert targets(world, "pinky1") == ["O_X"], "by the express, not round by the hall"
    queued = [e.detail for e in world.events if e.code == "DOCK_QUEUED" and e.robot == "pinky1"]
    assert queued == ["OUTBOUND is busy, waits at O_X"]
    assert "EXPRESS" in world.traffic.grants.held_by("pinky1")
    assert "OUT_DOCK" not in world.traffic.grants.held_by("pinky1")
    world.traffic.on_release("pinky3")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0), world.log[-10:]
    assert world.result_of(call).success and targets(world, "pinky1") == ["O_X", "O"]
    assert not world.problems


def test_a_robot_in_the_hall_still_waits_for_a_busy_outbound_at_o_in():
    """M_E는 넓은 곳에서 W에 가장 가까운 지점이다. 거기서도 O_X(줄과 W를 지나 급행으로)보다
    O_IN이 싸다."""
    world, _ = outbound_busy({"pinky1": "M_E", "pinky2": "P-2", "pinky3": "O"})
    assert world.run_until(stands_at(world, "pinky1", "O_IN"), 60.0), world.log[-10:]
    assert targets(world, "pinky1") == ["O_IN"]
    assert not world.problems


def test_while_one_robot_waits_at_o_x_a_robot_from_the_hall_enters_outbound_on_its_turn():
    """사용자 요청: O_X에 하나가 기다려도 넓은 곳에서 온 로봇은 O_IN에서 출고로 들어간다. 차례는
    지금 규칙(_ahead, 먼저 바란 로봇)이다: 먼저 바란 O_IN의 로봇이 먼저, 그다음 O_X의 로봇."""
    world, hall, express = outbound_busy({"pinky1": "W", "pinky2": "O_IN", "pinky3": "O"},
                                         ("pinky2", "pinky1"))  # pinky2가 먼저 바랐다
    assert world.run_until(stands_at(world, "pinky1", "O_X"), 60.0), world.log[-10:]
    assert world.view("pinky2")["node"] == "O_IN"
    world.traffic.on_release("pinky3")
    assert world.run_until(lambda: world.result_of(hall) is not None, 60.0), world.log[-10:]
    assert targets(world, "pinky2") == ["O"], "straight in from O_IN by out_in"
    assert world.view("pinky1")["node"] == "O_X" and world.result_of(express) is None
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: world.result_of(express) is not None, 60.0), world.log[-10:]
    assert world.result_of(express).success and targets(world, "pinky1") == ["O_X", "O"]
    assert not world.problems


def test_a_second_robot_from_work_takes_the_hall_while_o_x_is_held_and_nobody_locks_up():
    """O_X에 하나가 서 있으면 W에서 나오는 다음 로봇은 급행에 들어가지 않고(갈 자리가 없다) 넓은
    곳으로 O_IN에 간다. 출고의 로봇은 out_out으로 나가 급행을 쓰지 않으므로 서로 기다려 갇히지
    않는다."""
    # pinky2는 줄 앞자리 Q1에서 출고로 간다: W를 지나야 한다(W가 비면 W로, 거기서 다시 고른다)
    world, first, second = outbound_busy({"pinky1": "W", "pinky2": "Q1", "pinky3": "O"},
                                         ("pinky1", "pinky2"))
    lanes = set()

    def at_o_in():
        lanes.update(z for z in world.traffic.grants.held_by("pinky2") if z == "EXPRESS")
        return stands_at(world, "pinky2", "O_IN")()
    assert world.run_until(at_o_in, 90.0), world.log[-10:]
    assert world.view("pinky1")["node"] == "O_X"
    assert not lanes, "pinky2 never takes the express"
    assert targets(world, "pinky2") == ["W", "M_E", "O_IN"]
    world.traffic.on_release("pinky3")
    assert world.run_until(lambda: world.result_of(first) is not None, 60.0), world.log[-10:]
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.result_of(second) is not None, 60.0), world.log[-10:]
    assert world.result_of(first).success and world.result_of(second).success
    assert targets(world, "pinky1")[:2] == ["O_X", "O"] and targets(world, "pinky2")[3] == "O"
    assert not world.problems


def test_without_the_order_model_a_free_outbound_is_reached_in_one_follow_goal():
    """order 모델이 꺼져 있으면(Gazebo v7 경기) 대기 노드마다 조각을 끊는다(Q2, Q1). 급행 안의 대기
    자리 O_X는 예외다: 도크로 가는 트립이면 O_X에서 끊지 않고 OUT_DOCK까지 한 번에 잡아 한 goal로
    간다(O_X 전과 같다). O_X에서 끊으면 경기마다 출고로 갈 때 O_X에서 섰다 다시 떠난다."""
    world = Harness(layout="v7", starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    world.report()
    call = world.call("OUTBOUND", "pinky1")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0), world.log[-10:]
    assert targets(world, "pinky1") == ["O"]
    assert "DOCK_QUEUED" not in world.codes("pinky1")
    granted = [e.detail for e in world.events if e.code == "ZONE_GRANTED" and e.robot == "pinky1"]
    assert granted[0] == "WORK_DOCK+HALL+J_E+EXPRESS+OUT_DOCK to O"
    assert not world.problems


def test_with_the_order_model_a_robot_drives_through_o_x_when_outbound_frees_in_time():
    """order 모델(쇼): O_X로 가는 트립 끝에 도크가 horizon으로 붙는다. 도착 전에 출고가 비면 O_X에서
    서지 않고 그대로 들어간다."""
    params = replace(load_params(), use_order_model=True)
    world, call = outbound_busy({"pinky1": "W", "pinky2": "P-2", "pinky3": "O"}, params=params)
    assert world.run_until(lambda: world.bots["pinky1"].run is not None, 10.0)
    world.traffic.on_release("pinky3")
    o_x = world.site.node_pose("O_X")
    still = []

    def done():
        bot = world.bots["pinky1"]
        if abs(bot.x - o_x[0]) < 1e-6 and abs(bot.y - o_x[1]) < 1e-6:
            still.append(world.now)
        return world.result_of(call) is not None
    assert world.run_until(done, 60.0), world.log[-10:]
    assert world.result_of(call).success
    assert len(still) <= 1, f"stood at O_X for {len(still) * DT:.1f} s"
    queued = [e.detail for e in world.events if e.code == "DOCK_QUEUED" and e.robot == "pinky1"]
    assert queued == ["OUTBOUND is busy, waits at O_X"], "it set off for O_X while O was busy"
    assert not world.problems


def test_with_the_order_model_a_robot_still_stops_at_o_x_while_outbound_is_busy():
    params = replace(load_params(), use_order_model=True)
    world, call = outbound_busy({"pinky1": "W", "pinky2": "P-2", "pinky3": "O"}, params=params)
    o_x = world.site.node_pose("O_X")
    assert world.run_until(lambda: (world.bots["pinky1"].x, world.bots["pinky1"].y) == o_x,
                           60.0), world.log[-10:]
    world.run(5.0)
    assert (world.bots["pinky1"].x, world.bots["pinky1"].y) == o_x
    world.traffic.on_release("pinky3")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0), world.log[-10:]
    assert world.result_of(call).success and not world.problems
