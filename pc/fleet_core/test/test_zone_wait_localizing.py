"""LOCALIZING(위치를 잃음)을 보고한 로봇: 새 일과 goal을 주지 않고 기다린다.

실제 로봇(2026-11-18): 노드 무리가 망에 붙자 라즈베리파이가 약 25 s 포화돼 map->base_link가
끊기고 LOCALIZING을 보고했다. 그 사이 관제가 보낸 order는 pode_fsm이 바로 NAV_ABORTED로
끝냈고, 시도 셋이 5 s 안에 쌓여 pode2가 격리됐다.
"""
from fleet_core.common.commands import MoveCmd, MoveOrderCmd, RobotIsolated
from zone_harness import Harness


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
    world.traffic.on_goal_lost("pode2", last_goal(world))  # 바로 다시 보낼 사유
    world.run(5.0)
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


def test_the_reported_case_does_not_isolate_the_robot():
    """기록: LOCALIZING 1.3 s 뒤 ORDER_SENT, NAV_ABORTED 세 번에 격리. 이제 격리하지 않는다."""
    world = Harness(speed=0.0)
    world.step()
    reports(world, "pode2", "LOCALIZING")
    world.run(1.3)
    world.call("WORK", "pode2")
    for _ in range(3):
        world.run(2.5)
        if moves(world):
            world.traffic.on_move_result("pode2", last_goal(world), False, "NAV_ABORTED")
    assert moves(world) == 0 and not isolations(world)
    reports(world, "pode2", None)
    world.run(0.3)
    assert moves(world) == 1


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
