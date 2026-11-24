"""LOCALIZING(위치를 잃음)을 보고한 로봇: 새 일과 goal을 주지 않고 기다린다.

실제 로봇(2026-11-18): 노드 무리가 망에 붙자 라즈베리파이가 약 25 s 포화돼 map->base_link가
끊기고 LOCALIZING을 보고했다. 그 사이 관제가 보낸 order는 pode_fsm이 바로 NAV_ABORTED로
끝냈고, 시도 셋이 5 s 안에 쌓여 pode2가 격리됐다.
"""
import pytest
from fleet_core.common.commands import CancelCmd, MoveCmd, MoveOrderCmd, RobotIsolated
from fleet_core.traffic.zone.base import LOCALIZING_GRACE_S
from fleet_core.traffic.zone.lane_policy import Want
from zone_harness import DT, Harness


def reports(world, robot, state):
    """robot은 앞으로 state를 보고한다. 다른 로봇은 그대로다. state가 None이면 원래대로."""
    if state is None:
        world.__dict__.pop("_state", None)
        return
    plain = Harness._state.__get__(world)
    world._state = lambda bot: state if bot.name == robot else plain(bot)


def last_goal(world, robot="pode2"):
    return world.sent((MoveCmd, MoveOrderCmd), robot)[-1].goal_id


def moves(world, robot="pode2"):
    return len(world.moves(robot))


def isolations(world):
    return [c for c in world.commands if isinstance(c, RobotIsolated)]


def first_for_open_call():
    world = Harness(speed=0.0)
    world.step()
    world.call("WORK")
    world.run(1.0)
    return next(name for name in world.bots if world.moves(name))


# ---- 새 일을 주지 않는다 ----

def test_a_localizing_robot_is_not_picked_for_an_open_call_and_another_robot_takes_it():
    first = first_for_open_call()
    world = Harness(speed=0.0)
    world.step()
    reports(world, first, "LOCALIZING")
    world.step()
    world.call("WORK")
    world.run(3.0)
    assert moves(world, first) == 0
    assert any(world.moves(name) for name in world.bots if name != first), "another robot goes"


def test_a_named_dispatch_to_a_localizing_robot_waits_and_goes_once_it_is_localized():
    world = Harness(speed=0.0)
    world.step()
    reports(world, "pode2", "LOCALIZING")
    world.step()
    world.call("WORK", "pode2")
    world.run(5.0)
    assert moves(world) == 0, "no goal while it reports LOCALIZING"
    assert world.codes("pode2").count("WAIT_LOCALIZING") == 1
    reports(world, "pode2", None)
    world.run(0.3)
    assert moves(world) == 1, "the pending goal goes out at once"
    assert "LOCALIZED" in world.codes("pode2")


def test_a_robot_that_starts_localizing_with_a_goal_gets_no_new_goal_after_it_ends():
    world = Harness(speed=0.0)
    world.step()
    world.call("WORK", "pode2")
    world.step()
    assert moves(world) == 1
    reports(world, "pode2", "LOCALIZING")
    world.step()
    world.traffic.on_goal_not_sent("pode2", last_goal(world))  # goal을 비우고 다시 보낼 사유
    assert world.view("pode2")["goal_id"] is None
    world.run(world.params.move_retry_wait_s + 3.0)
    assert moves(world) == 1, "not sent again while LOCALIZING"


# ---- LOCALIZING 중의 NAV_ABORTED는 시도로 세지 않는다 ----

def test_nav_aborted_while_localizing_is_no_attempt_and_the_move_goes_again_after():
    world = Harness(speed=0.0)
    world.step()
    call = world.call("WORK", "pode2")
    world.step()
    p = world.params
    for _ in range(p.move_retry_limit + 3):
        reports(world, "pode2", "LOCALIZING")
        world.step()
        before = moves(world)
        world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
        world.run(p.move_retry_wait_s + 1.0)
        assert moves(world) == before, "no resend while LOCALIZING"
        assert world.view("pode2")["attempts"] == 0
        reports(world, "pode2", None)
        world.run(0.3)
        assert moves(world) == before + 1, "sent again when it is localized"
    codes = world.codes("pode2")
    assert "GOAL_RETRY" not in codes and "GOAL_FAILED" not in codes
    assert world.result_of(call) is None and not isolations(world)
    assert world.view("pode2")["state"] == "TO_WORK"


def test_after_localizing_the_retry_still_waits_out_move_retry_wait_s():
    world = Harness(speed=0.0)
    world.step()
    world.call("WORK", "pode2")
    world.step()
    p = world.params
    reports(world, "pode2", "LOCALIZING")
    world.step()
    world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
    world.run(0.5)
    reports(world, "pode2", None)
    world.run(p.move_retry_wait_s - 1.0)
    assert moves(world) == 1, "the retry wait is not over yet"
    world.run(1.0)
    assert moves(world) == 2


def test_the_reported_case_sends_nothing_while_localizing():
    """기록: LOCALIZING 1.3 s 뒤 ORDER_SENT. 이제 LOCALIZING이면 order가 나가지 않는다."""
    world = Harness(speed=0.0)
    world.step()
    reports(world, "pode2", "LOCALIZING")
    world.run(1.3)
    world.call("WORK", "pode2")
    world.run(3 * 2.5)
    assert moves(world) == 0 and not isolations(world)
    reports(world, "pode2", None)
    world.run(0.3)
    assert moves(world) == 1


@pytest.mark.parametrize("first", ["state", "result"])
def test_orders_sent_while_localized_then_aborted_by_lost_pose_are_never_counted(first):
    """order는 위치가 있을 때 나갔고, 그 뒤 위치를 잃어 NAV_ABORTED. 세 번 되풀이해도 세지 않는다.
    first: LOCALIZING 보고가 먼저인가, 결과가 먼저(1.0 s 안)인가."""
    world = Harness(speed=0.0)
    world.step()
    call = world.call("WORK", "pode2")
    world.step()
    p = world.params
    aborted = 0
    for _ in range(3):
        sent = moves(world)
        if first == "state":
            reports(world, "pode2", "LOCALIZING")
            world.run(0.5)
            world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
        else:
            world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
            world.run(0.5)
            reports(world, "pode2", "LOCALIZING")
        aborted += 1
        world.run(p.move_retry_wait_s + 1.0)
        assert world.view("pode2")["attempts"] == 0
        assert moves(world) == sent, "nothing sent while LOCALIZING"
        reports(world, "pode2", None)
        world.run(0.3)
        assert moves(world) == sent + 1
    assert aborted == 3
    assert "GOAL_FAILED" not in world.codes("pode2") and not isolations(world)
    assert world.result_of(call) is None


# ---- 그대로인 것 ----

def test_nav_aborted_from_a_localized_robot_still_counts():
    world = Harness(speed=0.0)
    world.step()
    world.call("WORK", "pode2")
    world.step()
    world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
    world.step()
    assert world.view("pode2")["attempts"] == 1
    assert "GOAL_RETRY" in world.codes("pode2")


def test_another_failure_while_localizing_still_counts():
    world = Harness(speed=0.0)
    world.step()
    world.call("WORK", "pode2")
    world.step()
    reports(world, "pode2", "LOCALIZING")
    world.step()
    world.traffic.on_move_result("pode2", last_goal(world), False, "FAILED")
    assert world.view("pode2")["attempts"] == 1


def test_a_robot_that_stays_localizing_keeps_its_zones_and_is_noted_by_the_usual_watches():
    """새 시간 한도는 없다. WAIT_LOCALIZING 한 번, 그 뒤로는 있던 ROBOT_IDLE이 idle_watch_s마다
    알린다(그 사건이 SILENT_STALL을 매번 새로 시작한다). 구역은 그대로 쥔다."""
    world = Harness(speed=0.0)
    world.step()
    reports(world, "pode2", "LOCALIZING")
    world.step()
    held = world.view("pode2")["held"]
    world.call("WORK", "pode2")
    p = world.params
    world.run(p.silent_limit_s + 5.0)
    codes = world.codes("pode2")
    assert codes.count("WAIT_LOCALIZING") == 1
    assert codes.count("ROBOT_IDLE") >= (p.silent_limit_s // p.idle_watch_s) - 1
    assert "SILENT_STALL" not in codes
    assert moves(world) == 0 and not isolations(world)
    view = world.view("pode2")
    assert (view["state"], view["held"]) == ("TO_WORK", held)


def test_a_new_localizing_episode_is_noted_again():
    world = Harness(speed=0.0)
    world.step()
    for _ in range(2):
        reports(world, "pode2", "LOCALIZING")
        world.run(0.5)
        reports(world, "pode2", None)
        world.run(0.5)
    assert world.codes("pode2").count("WAIT_LOCALIZING") == 2
    assert world.codes("pode2").count("LOCALIZED") == 2


# ---- 결과와 LOCALIZING 보고의 순서 (앞뒤 LOCALIZING_GRACE_S 안이면 LOCALIZING 중으로 본다) ----

def sent_and_open():
    world = Harness(speed=0.0)
    world.step()
    world.call("WORK", "pode2")
    world.step()
    assert moves(world) == 1
    return world


def test_nav_aborted_heard_just_before_the_localizing_report_gives_the_attempt_back():
    world = sent_and_open()
    world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
    assert world.view("pode2")["attempts"] == 1, "counted: the robot still looked localized"
    reports(world, "pode2", "LOCALIZING")
    world.step()
    assert world.view("pode2")["attempts"] == 0
    assert "GOAL_ABORTED_LOCALIZING" in world.codes("pode2")
    world.run(5.0)
    assert moves(world) == 1, "no resend while LOCALIZING"
    reports(world, "pode2", None)
    world.run(0.3)
    assert moves(world) == 2


def test_a_localizing_report_long_after_the_failure_gives_nothing_back():
    world = sent_and_open()
    world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
    world.run(1.5)
    reports(world, "pode2", "LOCALIZING")
    world.step()
    assert world.view("pode2")["attempts"] == 1


def test_nav_aborted_heard_just_after_the_robot_is_localized_again_is_no_attempt():
    world = sent_and_open()
    reports(world, "pode2", "LOCALIZING")
    world.run(1.0)
    reports(world, "pode2", None)
    world.step()
    world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
    world.step()
    assert world.view("pode2")["attempts"] == 0
    assert "GOAL_RETRY" not in world.codes("pode2")


def test_nav_aborted_well_after_the_robot_is_localized_again_counts():
    world = sent_and_open()
    reports(world, "pode2", "LOCALIZING")
    world.run(1.0)
    reports(world, "pode2", None)
    world.run(1.5)
    world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
    world.step()
    assert world.view("pode2")["attempts"] == 1


# ---- 제자리 멈춤 감시 ----

def test_an_open_goal_while_localizing_is_not_judged_stalled():
    world = sent_and_open()
    reports(world, "pode2", "LOCALIZING")
    world.run(world.params.stall_s + 5.0)
    assert "GOAL_STALLED" not in world.codes("pode2")
    assert world.view("pode2")["attempts"] == 0


# ---- 1.0 s 경계: 안쪽은 LOCALIZING 중, 바깥은 시도 ----

@pytest.mark.parametrize(("gap", "attempts"), [(LOCALIZING_GRACE_S - 0.01, 0),
                                               (LOCALIZING_GRACE_S + 0.01, 1)])
def test_nav_aborted_after_the_robot_is_localized_again_at_the_edge(gap, attempts):
    world = sent_and_open()
    reports(world, "pode2", "LOCALIZING")
    world.step()
    last = world.now  # 마지막 LOCALIZING 보고
    bot = world.bots["pode2"]
    world.now = round(last + gap, 6)
    world.traffic.on_robot_state("pode2", bot.x, bot.y, bot.yaw, "IDLE")
    world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
    assert world.view("pode2")["attempts"] == attempts


@pytest.mark.parametrize(("gap", "attempts"), [(LOCALIZING_GRACE_S - 0.01, 0),
                                               (LOCALIZING_GRACE_S + 0.01, 1)])
def test_a_localizing_report_after_nav_aborted_at_the_edge(gap, attempts):
    world = sent_and_open()
    world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
    assert world.view("pode2")["attempts"] == 1
    bot = world.bots["pode2"]
    world.now = round(world.now + gap, 6)
    world.traffic.on_robot_state("pode2", bot.x, bot.y, bot.yaw, "LOCALIZING")
    assert world.view("pode2")["attempts"] == attempts


# ---- 한도에 닿은 뒤의 LOCALIZING 중 NAV_ABORTED ----

def test_at_the_retry_limit_a_nav_aborted_while_localizing_does_not_isolate():
    world = sent_and_open()
    p = world.params
    for attempt in range(1, p.move_retry_limit + 1):  # 위치가 있는 진짜 실패로 한도까지
        world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
        world.run(p.move_retry_wait_s + 0.3)
        assert world.view("pode2")["attempts"] == attempt
    assert world.view("pode2")["attempts"] == p.move_retry_limit
    reports(world, "pode2", "LOCALIZING")
    world.step()
    world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
    world.run(p.move_retry_wait_s + 1.0)
    assert not isolations(world) and world.view("pode2")["state"] == "TO_WORK"
    assert world.view("pode2")["attempts"] == p.move_retry_limit, "the count is kept"
    reports(world, "pode2", None)
    sent = moves(world)
    world.run(LOCALIZING_GRACE_S + 0.3)
    assert moves(world) == sent + 1
    world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
    world.step()
    assert isolations(world), "the next real failure still ends in isolation"


# ---- 감시와 양보가 LOCALIZING 로봇을 움직이지 않는다 ----

def waiting_for_each_other(world):
    for name, node in (("pode1", "P-3"), ("pode2", "P-1")):
        world.traffic._wants[name] = Want((), (node,), {}, 0.0)


def cycle_world():
    world = Harness(starts={"pode1": "P-1", "pode2": "P-3"})
    world.step()
    waiting_for_each_other(world)
    return world


def test_a_localizing_robot_in_a_wait_cycle_is_not_sent_away_the_other_one_gives_way():
    world = cycle_world()
    world.run(world.params.deadlock_grace_s + 0.5)
    [victim] = [name for name in world.bots if world.moves(name)]
    other = next(name for name in world.bots if name != victim)
    world = cycle_world()
    reports(world, victim, "LOCALIZING")
    world.run(world.params.deadlock_grace_s + 0.5)
    assert not world.moves(victim)
    assert len(world.moves(other)) == 1
    gave = [e.detail for e in world.events if e.code == "DEADLOCK_SEEN" and e.robot == other]
    assert any("gives way to" in detail for detail in gave)


def test_a_localizing_robot_in_the_way_of_a_blocked_robot_is_not_sent_to_parking():
    """test_zone_make_way의 세계: 막힌 pode3의 길 위에 선 pode2가 LOCALIZING이면 보내지 않는다."""
    from test_zone_make_way import queued_in_the_way
    from test_zone_make_way import run_until as run_with_body

    world, _, _ = queued_in_the_way()
    world.report()  # 옮긴 자리를 한 번 알린다
    reports(world, "pode2", "LOCALIZING")
    assert run_with_body(world, lambda: "ZONE_MARKED_BLOCKED" in world.codes("pode3"), 40.0)
    world.run(DT * 5)
    assert not world.moves("pode2")
    assert not any(e.code == "HALL_YIELD" and "goes to" in e.detail
                   for e in world.events if e.robot == "pode2")


def test_two_close_robots_one_localizing_are_not_parted_by_the_near_watch():
    world = Harness(speed=0.0)
    world.step()
    world.call("WORK", "pode1")
    world.call("OUTBOUND", "pode2")
    world.step()
    world.step()
    assert world.moves("pode1") and world.moves("pode2")
    reports(world, "pode1", "LOCALIZING")
    world.run(world.params.near_stuck_s + 1.0)
    assert "HALL_YIELD" not in world.codes()
    assert not world.sent(CancelCmd)
