"""위치 의심 때 구역 붙잡기(suspect_hold = check, 사용자 결정 B, 2026-11-16).

보고 위치가 튀거나(LOCALIZATION_SUSPECT) 쥔 구역 밖에 머물면(OUTSIDE_ZONE) 로봇을 의심한다.
의심하는 동안 쥔 구역을 하나도 놓지 않는다. 로봇은 멈추지 않고 지금 base 끝까지 간다.
base 끝에서 보고 위치가 goal 끝과 arrival_tolerance_m 안이면 의심을 푼다(DOUBT_CLEARED).
아니면 arrival_settle_s 뒤 취소하고 NOT_AT_TARGET으로 재시도한다. 보통의 goal 성공도 의심을
푼다. 격리되면 의심은 pose_doubt가 된다. 키가 ""이면 예전과 같다(기록만).
"""
from dataclasses import replace

from fleet_core.common.commands import CancelCmd
from fleet_core.common.params import load_params
from fleet_core.traffic.mission import NOT_AT_TARGET, RUN, STALLED, STOP
from zone_harness import Harness

OFF = replace(load_params(), suspect_hold="")
ON = replace(load_params(), suspect_hold="check")
JUMP = (0.0, 0.3)  # 0.1 s에 0.3 m: 로봇이 갈 수 없는 거리


def robot(world, name):
    return world.traffic._robots[name]


def queued(params):
    """pode2가 출고 도크를 쓰는 동안 pode1이 작업 도크에서 불린다. base는 O_Q까지(대기 자리)."""
    world = Harness(starts={"pode1": "W", "pode2": "O", "pode3": "P-3"}, params=params)
    world.report()
    world.call("OUTBOUND", "pode2")
    return world, world.call("OUTBOUND", "pode1")


def run_to(world, t):
    while world.now < t - 1e-6:
        world.step()


def glitch(world, name, at, back=None, jump=JUMP):
    """at에 보고 위치가 jump만큼 튄다. back이 있으면 그때 제자리로 돌아온다."""
    run_to(world, at)
    world.bias[name] = jump
    if back is not None:
        run_to(world, back)
        world.bias.pop(name)


def event(world, code, name):
    return next(e for e in world.events if e.code == code and e.robot == name)


def held_while(world, name, done, timeout_s=120.0):
    """done이 될 때까지 걸음마다 쥔 구역 집합을 모은다."""
    seen = [set(world.view(name)["held"])]
    while not done():
        assert world.now < timeout_s, "timed out"
        world.step()
        seen.append(set(world.view(name)["held"]))
    return seen


def test_the_shipped_file_runs_with_check_and_the_code_default_is_off():
    assert load_params().suspect_hold == "check"
    assert replace(load_params(), suspect_hold="").suspect_hold == ""


# ---- 끔: 예전과 같다 ----

def test_with_the_key_off_a_jump_is_only_recorded_and_the_zones_behind_are_released():
    world, _ = queued(OFF)
    glitch(world, "pode1", 4.0, back=5.0)
    run_to(world, 10.0)
    assert "LOCALIZATION_SUSPECT" in world.codes("pode1")
    assert "ZONES_HELD_SUSPECT" not in world.codes() and "DOUBT_CLEARED" not in world.codes()
    assert robot(world, "pode1").doubt == ""
    assert world.view("pode1")["held"] == ["HALL"], "WORK_DOCK and J_W were released behind"


def test_with_the_key_off_staying_outside_the_held_zones_raises_no_doubt():
    world = Harness(params=OFF)
    drift_out(world, "pode1")
    assert "OUTSIDE_ZONE" in world.codes("pode1")
    assert robot(world, "pode1").doubt == ""


# ---- 켬: 튄 뒤 base 끝까지 붙잡고 끝에서 확인 ----

def test_a_jump_holds_every_zone_until_the_base_end_check_passes():
    world, call = queued(ON)
    glitch(world, "pode1", 4.0, back=5.0)
    assert robot(world, "pode1").doubt == "LOCALIZATION_SUSPECT"
    assert world.codes("pode1").count("ZONES_HELD_SUSPECT") == 1, "the jump back adds none"
    seen = held_while(world, "pode1", lambda: "DOUBT_CLEARED" in world.codes("pode1"))
    assert all(zones >= {"WORK_DOCK", "J_W", "HALL"} for zones in seen), "nothing is released"
    assert world.codes("pode1").index("BASE_END_WAIT") < world.codes("pode1").index(
        "DOUBT_CLEARED"), "cleared by the check at the base end"
    assert robot(world, "pode1").doubt == ""
    assert world.run_until(lambda: world.view("pode1")["held"] == ["HALL"], 5.0), (
        "release resumes after the check")
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert world.result_of(call).success and not world.problems
    assert "GOAL_RETRY" not in world.codes("pode1")


def test_a_wrong_pose_at_the_base_end_fails_with_not_at_target_after_arrival_settle_s():
    world, _ = queued(ON)
    glitch(world, "pode1", 4.0)  # 보고 위치가 틀린 채로 남는다
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pode1"), 60.0)
    wait_at = world.now
    assert world.run_until(lambda: "GOAL_RETRY" in world.codes("pode1"), 30.0)
    took = world.now - wait_at
    assert ON.arrival_settle_s <= took <= ON.arrival_settle_s + 0.5
    assert event(world, "GOAL_RETRY", "pode1").detail.startswith(NOT_AT_TARGET)
    assert world.sent(CancelCmd, "pode1"), "the order is cancelled"
    assert "DOUBT_CLEARED" not in world.codes("pode1")
    assert {"WORK_DOCK", "J_W", "HALL"} <= set(world.view("pode1")["held"])
    assert robot(world, "pode1").doubt == "LOCALIZATION_SUSPECT", "the retry stays in doubt"


def test_a_pose_that_stays_wrong_ends_in_isolation_with_pose_doubt():
    world, _ = queued(ON)
    glitch(world, "pode1", 4.0, jump=(-0.3, 0.0))  # 틀린 보고 위치가 HALL 안에 남는다
    assert world.run_until(lambda: "ROBOT_ISOLATED" in world.codes("pode1"), 120.0)
    assert world.codes("pode1").count("GOAL_RETRY") == ON.move_retry_limit
    m = robot(world, "pode1")
    assert (m.pose_doubt, m.doubt) == (True, "")
    assert "DOUBT_CLEARED" not in world.codes("pode1")
    assert {"WORK_DOCK", "J_W"} <= set(world.view("pode1")["held"]), "kept through isolation"


def test_the_same_wrong_pose_frees_a_zone_the_robot_is_in_only_with_the_key_off():
    """도크가 비어 order가 O까지 이어진다. 끄면 보고 위치를 믿고 OUT_DOCK을 놓는다(I3)."""
    for params, broken in ((OFF, True), (ON, False)):
        world, _ = queued(params)
        glitch(world, "pode1", 4.0, jump=(-0.3, 0.0))
        world.traffic.on_release("pode2")
        world.run(40.0)
        assert bool(world.problems) is broken, (params.suspect_hold, world.problems[:1])


# ---- 서 있을 때 튐: 다음 goal에서 확인한다(영영 붙잡지 않는다) ----

def test_a_jump_while_standing_still_is_checked_on_the_next_goal():
    world = Harness(params=ON)
    world.run(1.0)
    before = set(world.view("pode1")["held"])
    glitch(world, "pode1", 2.0, back=2.5)
    world.run(5.0)
    assert robot(world, "pode1").doubt == "LOCALIZATION_SUSPECT", "no goal, nothing to check yet"
    assert set(world.view("pode1")["held"]) == before
    call = world.call("WORK", "pode1")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert world.result_of(call).success
    assert "DOUBT_CLEARED" in world.codes("pode1") and robot(world, "pode1").doubt == ""
    assert world.run_until(lambda: world.view("pode1")["held"] == ["WORK_DOCK"], 2.0), (
        "release works again after the goal")
    assert not world.problems


def test_a_jump_while_an_order_update_is_unanswered_is_checked_at_the_new_base_end():
    world, call = queued(ON)
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pode1"), 60.0)
    world.hold_answers = True
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: "ORDER_UPDATED" in world.codes("pode1"), 30.0)
    assert robot(world, "pode1").move_order.before is not None, "the update is unanswered"
    start = world.now
    glitch(world, "pode1", start + 0.2, back=start + 0.6)
    assert robot(world, "pode1").doubt == "LOCALIZATION_SUSPECT"
    world.hold_answers = False
    held = set(world.view("pode1")["held"])
    seen = held_while(world, "pode1", lambda: "DOUBT_CLEARED" in world.codes("pode1"))
    assert all(zones >= held for zones in seen), "nothing is released while in doubt"
    assert "ORDER_UPDATE_REJECTED" not in world.codes("pode1")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert world.result_of(call).success and not world.problems


# ---- 보통의 goal 성공도 의심을 푼다(order를 끈 MoveCmd) ----

def test_a_normal_goal_success_clears_the_doubt():
    world = Harness(params=replace(ON, use_order_model=False))
    world.step()
    world.call("WORK", "pode2")
    glitch(world, "pode2", 1.0, back=1.5)
    assert robot(world, "pode2").doubt == "LOCALIZATION_SUSPECT"
    assert world.run_until(lambda: "DOUBT_CLEARED" in world.codes("pode2"), 60.0)
    assert robot(world, "pode2").doubt == ""
    assert world.run_until(lambda: world.view("pode2")["held"] == ["WORK_DOCK"], 60.0)
    assert not world.problems


# ---- 격리되면 의심은 pose_doubt가 된다 ----

def test_isolation_turns_the_doubt_into_pose_doubt():
    world = Harness(params=ON)
    world.run(1.0)
    glitch(world, "pode1", 2.0, back=2.5)
    m = robot(world, "pode1")
    world.traffic._fail(m, STALLED)
    assert (m.pose_doubt, m.doubt, m.doubt_by) == (True, "", None)


def test_with_the_key_off_isolation_for_another_reason_has_no_pose_doubt():
    world = Harness(params=OFF)
    world.run(1.0)
    glitch(world, "pode1", 2.0, back=2.5)
    m = robot(world, "pode1")
    world.traffic._fail(m, STALLED)
    assert m.pose_doubt is False


# ---- 쥔 구역 밖에 머문 보고(OUTSIDE_ZONE)도 의심이다 ----

def drift_out(world, name, step=0.015, limit_s=20.0):
    """보고 위치가 걸음마다 step씩(튐 문턱 아래로) 흘러 쥔 구역 밖으로 나간다."""
    world.run(1.0)
    dy = 0.0
    while "OUTSIDE_ZONE" not in world.codes(name):
        assert world.now < limit_s, "never left the held zones"
        dy += step
        world.bias[name] = (0.0, dy)
        world.step()


def test_staying_outside_the_held_zones_raises_a_doubt_without_a_jump():
    world = Harness(params=ON)
    drift_out(world, "pode1")
    assert "LOCALIZATION_SUSPECT" not in world.codes("pode1")
    assert robot(world, "pode1").doubt == "OUTSIDE_ZONE"
    assert event(world, "ZONES_HELD_SUSPECT", "pode1").detail.startswith("OUTSIDE_ZONE")


# ---- 확인에 쓰는 위치와 시간(Codex 리뷰 9309265) ----

def reports(world, name, state):
    """name은 앞으로 state를 보고한다. 다른 로봇은 그대로다."""
    plain = Harness._state.__get__(world)
    world._state = lambda bot: state if bot.name == name else plain(bot)


def test_a_robot_that_reports_localizing_at_the_base_end_is_not_cleared_by_its_old_pose():
    """LOCALIZING 보고는 위치를 바꾸지 않는다. 그 전의 위치로 의심을 풀지 않는다."""
    world, _ = queued(ON)
    glitch(world, "pode1", 4.0, back=5.0)
    run_to(world, 19.0)  # O_Q 0.12 m 앞: 옛 위치로 재면 허용 오차 안이다
    reports(world, "pode1", "LOCALIZING")
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pode1"), 10.0)
    world.run(2.0)
    assert "DOUBT_CLEARED" not in world.codes("pode1")
    assert "GOAL_RETRY" not in world.codes("pode1"), "no check while it localizes"
    assert robot(world, "pode1").doubt == "LOCALIZATION_SUSPECT"
    del world._state
    assert world.run_until(lambda: "DOUBT_CLEARED" in world.codes("pode1"), 2.0)


def test_a_new_goal_gets_its_own_arrival_settle_s_at_the_base_end():
    """확인을 기다리던 goal이 취소되면(STOP) 그 기한은 다음 goal에 남지 않는다."""
    world, _ = queued(ON)
    glitch(world, "pode1", 4.0, jump=(-0.3, 0.0))
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pode1"), 60.0)
    world.run(0.5)
    world.traffic.on_run_mode(STOP)
    world.run(3.0)
    world.traffic.on_run_mode(RUN)
    assert world.run_until(lambda: world.codes("pode1").count("BASE_END_WAIT") == 2, 30.0)
    wait_at = world.now
    assert world.run_until(lambda: "GOAL_RETRY" in world.codes("pode1"), 30.0)
    assert world.now - wait_at >= ON.arrival_settle_s - 1e-6


# ---- 도킹 성공도 위치를 보고 의심을 푼다 ----

def docking_with_a_glitch(params, jump=JUMP, back=True):
    world = Harness(params=params)
    world.step()
    world.call("WORK", "pode2")
    assert world.run_until(lambda: world.phase("pode2") == "DOCKING", 60.0)
    start = world.now
    glitch(world, "pode2", start + 0.3, back=start + 0.6 if back else None, jump=jump)
    world.step()
    assert robot(world, "pode2").doubt == "LOCALIZATION_SUSPECT"
    assert world.run_until(lambda: world.phase("pode2") == "AT_STATION", 30.0)
    world.run(1.0)
    return world


def test_a_dock_success_at_the_dock_clears_the_doubt():
    world = docking_with_a_glitch(ON)
    assert "DOUBT_CLEARED" in world.codes("pode2") and robot(world, "pode2").doubt == ""
    assert world.view("pode2")["held"] == ["WORK_DOCK"], "release works again at the station"


def test_a_dock_success_with_the_pose_still_off_keeps_the_doubt():
    world = docking_with_a_glitch(ON, back=False)
    assert "DOUBT_CLEARED" not in world.codes("pode2")
    assert robot(world, "pode2").doubt == "LOCALIZATION_SUSPECT"
