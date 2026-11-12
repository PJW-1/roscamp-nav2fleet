"""구역 교통 층의 감시: 제자리 멈춤, 로봇의 자가 복구(RECOVERING) 상한, 넓은 구역에서 서로 막음
(HALL_YIELD), 대기 순환의 기록, 할 일 없이 섬(ROBOT_IDLE)."""
from dataclasses import replace

import zone_harness
from fleet_core.common.commands import CancelCmd, DockCmd, MoveCmd
from fleet_core.common.params import load_params
from fleet_core.traffic.zone.lane_policy import Want
from zone_harness import Harness


def called(robot="pinky2", station="WORK", **kwargs):
    """robot이 이름으로 불려 첫 Nav2 goal을 받았다."""
    world = Harness(**kwargs)
    world.step()
    world.call(station, robot)
    world.step()
    assert world.sent(MoveCmd, robot)
    return world


def reports(world, robot, state):
    """robot은 앞으로 state를 보고한다. 다른 로봇은 그대로다."""
    plain = Harness._state.__get__(world)
    world._state = lambda bot: state if bot.name == robot else plain(bot)


def count(world, code, robot=""):
    return world.codes(robot).count(code)


# ---- 제자리 멈춤 ----

def test_a_goal_open_with_no_movement_is_cancelled_and_counted_as_an_attempt():
    world = called(speed=0.0)
    goal = world.view("pinky2")["goal_id"]
    world.run(world.params.stall_s - 0.5)
    assert count(world, "GOAL_STALLED") == 0
    world.run(1.0)
    assert count(world, "GOAL_STALLED", "pinky2") == 1
    assert CancelCmd("pinky2", goal) in world.commands
    assert [e.detail for e in world.events if e.code == "GOAL_RETRY"] == ["STALLED, attempt 1"]


def test_a_slow_robot_that_keeps_moving_is_not_stalled():
    p = load_params()
    slow = p.stall_move_m / (p.stall_s - 1.0)  # 멈춤 시간 안에 stall_move_m을 넘게 간다
    world = called(speed=slow)
    world.run(3 * p.stall_s)
    assert count(world, "GOAL_STALLED") == 0
    assert len(world.sent(MoveCmd, "pinky2")) == 1


def test_a_robot_that_reports_it_is_recovering_by_itself_is_not_stalled():
    world = called(speed=0.0)
    reports(world, "pinky2", "RECOVERING")
    world.run(3 * world.params.stall_s)
    assert count(world, "GOAL_STALLED") == 0 and not world.sent(CancelCmd)


def test_docking_is_given_the_longer_stall_time(monkeypatch):
    monkeypatch.setattr(zone_harness, "DOCK_S", 1000.0)  # 도킹이 끝나지 않는다
    world = Harness(starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    world.report()
    world.call("WORK", "pinky1")
    world.step()
    assert world.sent(DockCmd, "pinky1")
    world.run(world.params.dock_stall_s - 1.0)
    assert count(world, "GOAL_STALLED") == 0, "longer than a move may stand"
    world.run(2.0)
    stalled = [e.detail for e in world.events if e.code == "GOAL_STALLED"]
    assert len(stalled) == 1 and stalled[0].startswith("dock goal")


# ---- 자가 복구 상한 ----

def test_a_short_self_recovery_is_only_noted():
    world = called(speed=0.0)
    reports(world, "pinky2", "RECOVERING")
    goal = world.view("pinky2")["goal_id"]
    world.run(20.0)
    reports(world, "pinky2", "MOVING")
    world.step()
    codes = world.codes("pinky2")
    assert [c for c in codes if c.startswith("SELF_RECOVERY")] == [
        "SELF_RECOVERY", "SELF_RECOVERY_END"]
    assert "BLOCKED_WAIT" not in codes
    assert world.view("pinky2")["goal_id"] == goal


def test_a_self_recovery_longer_than_the_limit_is_handled_as_blocked():
    world = called(speed=0.0)
    p = world.params
    reports(world, "pinky2", "RECOVERING")
    world.run(p.self_recovery_limit_s - 1.0)
    assert count(world, "SELF_RECOVERY", "pinky2") == 1
    assert count(world, "BLOCKED_WAIT") == 0
    world.run(2.0)
    assert count(world, "BLOCKED_WAIT", "pinky2") == 1
    assert count(world, "ZONE_MARKED_BLOCKED") == 0
    world.run(p.blocked_mark_s)
    assert count(world, "ZONE_MARKED_BLOCKED", "pinky2") == 1
    assert "REROUTE" in world.codes("pinky2")


# ---- 넓은 구역에서 서로 막음 ----

def two_close_in_the_yard(second_by_name):
    """P-1과 P-2(0.25 m 떨어짐)의 두 로봇이 마당에서 Nav2 goal을 받고 서 있다."""
    world = Harness(speed=0.0)
    world.step()
    world.call("WORK", "pinky1")
    if second_by_name:
        world.call("OUTBOUND", "pinky2")
    else:
        world.call("WORK")  # 열린 호출: pinky1은 이미 배차가 있어 pinky2가 불린다
    world.step()
    world.step()
    assert world.sent(MoveCmd, "pinky1") and world.sent(MoveCmd, "pinky2")
    return world


def test_two_robots_stuck_close_in_the_yard_one_yields_for_a_while():
    world = two_close_in_the_yard(second_by_name=True)
    p = world.params
    world.run(p.near_stuck_s - 0.5)
    assert count(world, "HALL_YIELD") == 0
    world.run(0.5)
    yields = [(e.robot, e.detail) for e in world.events if e.code == "HALL_YIELD"]
    assert yields == [("pinky1", f"stops {p.near_stuck_s:.0f} s for pinky2")], "same rank: name"
    assert world.sent(CancelCmd, "pinky1") and not world.sent(CancelCmd, "pinky2")
    moves = len(world.sent(MoveCmd, "pinky1"))
    world.run(p.near_stuck_s - 0.5)
    assert len(world.sent(MoveCmd, "pinky1")) == moves, "it stands aside"
    world.run(1.0)
    assert len(world.sent(MoveCmd, "pinky1")) == moves + 1


def test_the_robot_without_a_dispatch_is_the_one_that_yields():
    world = two_close_in_the_yard(second_by_name=False)
    world.run(world.params.near_stuck_s + 0.5)
    assert [e.robot for e in world.events if e.code == "HALL_YIELD"] == ["pinky2"]


# ---- 대기 순환의 기록 ----

def waiting_for_each_other(world):
    for name, node in (("pinky1", "P-3"), ("pinky2", "P-1")):
        world.traffic._wants[name] = Want((), (node,), {}, 0.0)


def test_nobody_gives_way_before_the_grace_time():
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-3"})
    world.step()
    waiting_for_each_other(world)
    world.run(world.params.deadlock_grace_s - 0.5)
    assert count(world, "DEADLOCK_SEEN") == 1, "seen at once"
    assert not world.sent(MoveCmd)
    world.run(1.0)
    assert len(world.sent(MoveCmd)) == 1 and count(world, "DEADLOCK_SEEN") == 2


def test_a_cycle_that_breaks_up_and_forms_again_is_reported_again():
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-3"})
    world.step()
    waiting_for_each_other(world)
    world.step()
    world.step()
    assert count(world, "DEADLOCK_SEEN") == 1, "the same cycle is said once"
    del world.traffic._wants["pinky2"]
    world.step()
    assert world.traffic._cycle == ()
    waiting_for_each_other(world)
    world.step()
    assert count(world, "DEADLOCK_SEEN") == 2
    assert not world.sent(MoveCmd), "the grace time starts again"


def test_a_robot_that_is_not_available_is_left_out_of_the_wait_graph():
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-3"})
    world.step()
    world.traffic.on_robot_fault("pinky2", "MOTOR")
    waiting_for_each_other(world)
    world.run(world.params.deadlock_grace_s + 1.0)
    assert count(world, "DEADLOCK_SEEN") == 0
    assert not world.sent(MoveCmd)


# ---- 할 일 없이 섬 ----

def test_a_robot_with_somewhere_to_go_but_no_goal_and_no_wait_is_reported_idle():
    world = Harness()
    world.step()
    world.traffic._choose = lambda m: None  # 길도 기다릴 것도 없다
    world.call("WORK", "pinky2")
    limit = world.params.idle_watch_s
    world.run(limit - 0.5)
    assert count(world, "ROBOT_IDLE") == 0
    world.run(1.0)
    idle = [(e.level, e.robot, e.detail) for e in world.events if e.code == "ROBOT_IDLE"]
    assert idle == [("WARN", "pinky2", "TO_WORK at P-2: no goal and nothing to wait for")]
    world.run(limit)
    assert count(world, "ROBOT_IDLE") == 2, "said again every idle_watch_s"
    assert not world.sent(MoveCmd), "it only warns"


def test_the_idle_watch_is_off_at_zero():
    world = Harness(params=replace(load_params(), idle_watch_s=0.0))
    world.step()
    world.traffic._choose = lambda m: None
    world.call("WORK", "pinky2")
    world.run(30.0)
    assert count(world, "ROBOT_IDLE") == 0


def test_a_robot_waiting_parked_for_a_busy_dock_is_not_idle():
    world = Harness(starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "W_Q"})
    world.report()
    world.call("WORK", "pinky1")
    world.call("WORK", "pinky3")
    world.call("WORK", "pinky2")
    world.run(3 * world.params.idle_watch_s)
    assert "DOCK_QUEUED" in world.codes("pinky2")
    assert count(world, "ROBOT_IDLE") == 0
