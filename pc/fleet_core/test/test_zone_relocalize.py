"""격리 전에 위치 다시 찾기(relocalize, 2026-11-16).

위치 탓일 수 있는 실패(B의 base 끝 확인 실패, DOCK_FAILED, NOT_AT_TARGET이나 DOCK_FAILED로 격리하기
직전)에서 관제는 선 로봇에게 Relocalize를 시킨다. 첫 번째는 제자리 회전까지(allow_global 거짓),
실패하면 두 번째는 전역 재초기화까지 허락한다. 둘 다 실패하면 예전처럼 격리한다.
다시 찾는 동안 쥔 구역을 모두 쥐고 새 goal, 배차, order 늘리기, 재시도를 하지 않는다.
성공하면 새 위치로 다시 놓는다. 쥔 구역 안이면 의심을 풀고 하던 이동이나 도킹을 다시 보낸다
(다시 찾기는 재시도 횟수를 쓰지 않는다). 밖이면 그 자리 구역이 비었을 때 잡고, 아니면 격리한다.
"""
import math
from dataclasses import replace

import pytest
from fleet_core.common.commands import (
    CancelCmd,
    DockCmd,
    MoveCmd,
    MoveOrderCmd,
    RelocalizeCmd,
    RetreatCmd,
)
from fleet_core.common.params import FleetParams, load_params
from fleet_core.traffic.mission import NOT_AT_TARGET
from zone_harness import Harness

RELOC = replace(load_params(), suspect_hold="check", relocalize=True)
NO_RELOC = replace(RELOC, relocalize=False)
STAYS_WRONG = (-0.3, 0.0)  # 틀린 보고 위치가 HALL 안에 남는다
MOTION = (MoveCmd, MoveOrderCmd, DockCmd, RetreatCmd)


def robot(world, name):
    return world.traffic._robots[name]


def queued(params):
    """pode2가 출고 도크를 쓰는 동안 pode1이 작업 도크에서 불린다. base는 O_Q까지(대기 자리)."""
    world = Harness(starts={"pode1": "W", "pode2": "O", "pode3": "P-3"}, params=params)
    world.report()
    world.call("OUTBOUND", "pode2")
    return world, world.call("OUTBOUND", "pode1")


def wrong_from(world, name, at, jump=STAYS_WRONG):
    while world.now < at - 1e-6:
        world.step()
    world.bias[name] = jump


def event(world, code, name):
    return next(e for e in world.events if e.code == code and e.robot == name)


def first(world, code, name):
    return world.codes(name).index(code)


# ---- 파라미터 ----

def test_the_shipped_file_relocalizes_and_the_code_default_is_off():
    shipped = load_params()
    assert shipped.relocalize is True
    assert shipped.relocalize_attempts == 2 and shipped.relocalize_timeout_s > 0
    fields = FleetParams.__dataclass_fields__
    assert fields["relocalize"].default is False, "old records replay without relocalize"


def test_relocalize_attempts_must_be_at_least_one():
    with pytest.raises(ValueError, match="relocalize_attempts"):
        replace(RELOC, relocalize_attempts=0)


# ---- 1. B의 base 끝 확인이 실패하면 재시도 전에 다시 찾는다 ----

def test_a_failed_base_end_check_relocalizes_before_any_not_at_target_retry():
    world, call = queued(RELOC)
    wrong_from(world, "pode1", 4.0)
    assert world.run_until(lambda: "RELOCALIZE_SENT" in world.codes("pode1"), 60.0)
    assert "GOAL_RETRY" not in world.codes("pode1"), "no NOT_AT_TARGET retry before it"
    assert event(world, "RELOCALIZE_SENT", "pode1").detail.startswith(NOT_AT_TARGET)
    (cmd,) = world.sent(RelocalizeCmd, "pode1")
    assert (cmd.allow_spin, cmd.allow_global) == (True, False)
    assert cmd.timeout_s == RELOC.relocalize_timeout_s
    assert world.run_until(lambda: "RELOCALIZED" in world.codes("pode1"), 10.0)
    assert "0.30 m" in event(world, "RELOCALIZED", "pode1").detail, "the pose shift"
    assert robot(world, "pode1").doubt == "", "the new pose clears the doubt"
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert world.result_of(call).success and not world.problems
    assert "GOAL_RETRY" not in world.codes("pode1"), "the relocalize used no retry"
    assert "ROBOT_ISOLATED" not in world.codes("pode1")


def test_the_robot_is_asked_only_after_it_stands_with_no_goal():
    world, _ = queued(RELOC)
    wrong_from(world, "pode1", 4.0)
    assert world.run_until(lambda: world.sent(RelocalizeCmd, "pode1"), 60.0)
    cancel = world.sent(CancelCmd, "pode1")[-1]
    at = {id(c): i for i, c in enumerate(world.commands)}
    assert at[id(cancel)] < at[id(world.sent(RelocalizeCmd, "pode1")[0])]
    assert robot(world, "pode1").goal is None


# ---- 2. DOCK_FAILED: 다시 찾고 다시 도킹 ----

def failing_dock_once(params):
    world = Harness(params=params)
    world.step()
    call = world.call("WORK", "pode2")
    assert world.run_until(lambda: world.phase("pode2") == "DOCKING", 60.0)
    world.dock_fails.add("pode2")
    assert world.run_until(lambda: "RELOCALIZE_SENT" in world.codes("pode2")
                           or "GOAL_RETRY" in world.codes("pode2"), 10.0)
    world.dock_fails.discard("pode2")
    return world, call


def test_dock_failed_relocalizes_then_docks_again():
    world, call = failing_dock_once(RELOC)
    assert "GOAL_RETRY" not in world.codes("pode2")
    assert event(world, "RELOCALIZE_SENT", "pode2").detail.startswith("DOCK_FAILED")
    docks_before = len(world.sent(DockCmd, "pode2"))
    assert docks_before == 1, "no redock while it relocalizes"
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    assert world.result_of(call).success
    codes = world.codes("pode2")
    assert first(world, "RELOCALIZED", "pode2") < len(codes) - 1
    assert len(world.sent(DockCmd, "pode2")) == 2
    assert "GOAL_RETRY" not in codes and not world.problems
    m = robot(world, "pode2")
    assert (m.attempts, m.dock_tries) == (0, 0)


def test_with_relocalize_off_dock_failed_retries_as_before():
    world, call = failing_dock_once(NO_RELOC)
    assert "RELOCALIZE_SENT" not in world.codes("pode2")
    assert event(world, "GOAL_RETRY", "pode2").detail.startswith("DOCK_FAILED")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    assert world.result_of(call).success and not world.sent(RelocalizeCmd)


# ---- 3. 두 번 실패하면 격리 ----

def test_two_failed_relocalizes_isolate_the_robot_as_before():
    world, _ = queued(RELOC)
    world.reloc_fails["pode1"] = "NOT_CONVERGED"
    wrong_from(world, "pode1", 4.0)
    assert world.run_until(lambda: "ROBOT_ISOLATED" in world.codes("pode1"), 90.0)
    first_try, second_try = world.sent(RelocalizeCmd, "pode1")
    assert (first_try.allow_spin, first_try.allow_global) == (True, False)
    assert (second_try.allow_spin, second_try.allow_global) == (True, True)
    codes = world.codes("pode1")
    assert codes.count("RELOCALIZE_FAILED") == 2
    assert "NOT_CONVERGED" in event(world, "RELOCALIZE_FAILED", "pode1").detail
    assert first(world, "RELOCALIZE_FAILED", "pode1") < first(world, "ROBOT_ISOLATED", "pode1")
    assert event(world, "ROBOT_ISOLATED", "pode1").detail.startswith(NOT_AT_TARGET)
    assert "GOAL_RETRY" not in codes
    assert {"WORK_DOCK", "J_W", "HALL"} <= set(world.view("pode1")["held"])
    assert robot(world, "pode1").pose_doubt


def test_one_attempt_in_the_params_isolates_after_the_first_failure():
    world, _ = queued(replace(RELOC, relocalize_attempts=1))
    world.reloc_fails["pode1"] = "SCAN_MISMATCH"
    wrong_from(world, "pode1", 4.0)
    assert world.run_until(lambda: "ROBOT_ISOLATED" in world.codes("pode1"), 90.0)
    assert len(world.sent(RelocalizeCmd, "pode1")) == 1


def test_no_answer_in_time_is_a_failed_attempt_and_the_goal_is_cancelled():
    params = replace(RELOC, relocalize_timeout_s=3.0)
    world, _ = queued(params)
    world.reloc_silent.add("pode1")
    wrong_from(world, "pode1", 4.0)
    assert world.run_until(lambda: "RELOCALIZE_FAILED" in world.codes("pode1"), 90.0)
    first_try = world.sent(RelocalizeCmd, "pode1")[0]
    assert "TIMEOUT" in event(world, "RELOCALIZE_FAILED", "pode1").detail
    assert any(c.goal_id == first_try.goal_id for c in world.sent(CancelCmd, "pode1"))
    world.reloc_silent.discard("pode1")
    assert world.run_until(lambda: "RELOCALIZED" in world.codes("pode1"), 30.0)
    assert len(world.sent(RelocalizeCmd, "pode1")) == 2


def test_a_robot_without_the_relocalize_server_falls_back_to_the_old_retries():
    world, _ = queued(RELOC)
    world.reloc_fails["pode1"] = "NOT_SENT"
    wrong_from(world, "pode1", 4.0)
    assert world.run_until(lambda: "ROBOT_ISOLATED" in world.codes("pode1"), 120.0)
    assert len(world.sent(RelocalizeCmd, "pode1")) == 1, "asked once, then never again"
    assert world.codes("pode1").count("GOAL_RETRY") == RELOC.move_retry_limit


# ---- 4. 다시 찾는 동안: 구역을 쥐고 아무것도 새로 보내지 않는다 ----

def test_nothing_new_is_sent_and_no_zone_is_released_while_relocalizing():
    world, _ = queued(RELOC)
    world.reloc_s = 8.0
    wrong_from(world, "pode1", 4.0)
    assert world.run_until(lambda: "RELOCALIZE_SENT" in world.codes("pode1"), 60.0)
    held = set(world.view("pode1")["held"])
    sent = len(world.sent(MOTION, "pode1"))
    world.traffic.on_release("pode2")  # 갈 길이 열려도
    for _ in range(70):
        world.step()
        assert set(world.view("pode1")["held"]) == held
        assert len(world.sent(MOTION, "pode1")) == sent
    assert "RELOCALIZED" not in world.codes("pode1")
    assert world.run_until(lambda: "RELOCALIZED" in world.codes("pode1"), 3.0)
    assert world.run_until(lambda: len(world.sent(MOTION, "pode1")) > sent, 5.0)


def test_a_relocalizing_robot_takes_no_call_and_no_dispatch():
    world = Harness(params=RELOC)
    world.run(1.0)
    m = robot(world, "pode1")
    assert world.traffic._relocalize_first(m, NOT_AT_TARGET, (0, 0))
    world.run(0.5)
    assert world.sent(RelocalizeCmd, "pode1")
    goal = 777
    world.traffic.on_dispatch(goal, "OUTBOUND", "pode1")
    world.run(0.1)
    refused = world.result_of(goal)
    assert refused is not None and not refused.success and refused.reason == "LOCALIZING"
    world.call("WORK")  # 열린 호출
    world.run(1.0)
    assert not m.called and not world.sent(MOTION, "pode1")


def test_a_relocalizing_robot_is_not_reported_idle_or_silent():
    world = Harness(params=replace(RELOC, idle_watch_s=1.0, silent_limit_s=2.0))
    world.reloc_s = 6.0
    world.step()
    world.call("WORK", "pode2")
    assert world.run_until(lambda: world.phase("pode2") == "DOCKING", 60.0)
    world.dock_fails.add("pode2")
    assert world.run_until(lambda: "RELOCALIZE_SENT" in world.codes("pode2"), 10.0)
    world.dock_fails.discard("pode2")
    world.run(5.5)
    assert "ROBOT_IDLE" not in world.codes("pode2")
    assert "SILENT_STALL" not in world.codes("pode2")


# ---- 5. 새 위치가 쥔 구역 밖: 비었으면 잡고 아니면 격리 ----

def test_a_new_pose_outside_the_held_zones_takes_the_free_zone_there():
    world, call = queued(RELOC)
    wrong_from(world, "pode1", 4.0)
    world.reloc_pose["pode1"] = world.site.node_pose("X_E")  # J_E: 비어 있다
    assert world.run_until(lambda: "RELOCALIZED" in world.codes("pode1"), 60.0)
    assert "J_E" in world.view("pode1")["held"]
    assert "J_E" in event(world, "RELOCALIZED", "pode1").detail
    assert "ROBOT_ISOLATED" not in world.codes("pode1") and not world.problems
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert world.result_of(call).success and not world.problems


def test_a_new_pose_in_a_zone_someone_else_holds_isolates_the_robot():
    world, _ = queued(RELOC)
    wrong_from(world, "pode1", 4.0)
    ox, oy = world.site.node_pose("O")
    world.reloc_pose["pode1"] = (ox - 0.05, oy)  # OUT_DOCK: pode2가 쥐었다
    assert world.run_until(lambda: "ROBOT_ISOLATED" in world.codes("pode1"), 60.0)
    assert "RELOCALIZED" in world.codes("pode1")
    assert "OUT_DOCK" not in world.view("pode1")["held"]
    assert world.view("pode2")["held"] == ["OUT_DOCK"] or "OUT_DOCK" in world.view("pode2")["held"]


# ---- 격리 직전(재시도를 다 쓴 뒤)에도 다시 찾는다 ----

def test_not_at_target_retries_end_in_a_relocalize_instead_of_isolation():
    """B를 끈 MoveCmd: 성공 결과마다 보고 위치가 끝에서 벗어나 NOT_AT_TARGET을 되풀이한다."""
    params = replace(RELOC, suspect_hold="", use_order_model=False)
    world = Harness(params=params)
    world.step()
    call = world.call("WORK", "pode2")
    world.bias["pode2"] = STAYS_WRONG
    assert world.run_until(lambda: "RELOCALIZE_SENT" in world.codes("pode2")
                           or "ROBOT_ISOLATED" in world.codes("pode2"), 120.0)
    assert world.codes("pode2").count("GOAL_RETRY") == params.move_retry_limit
    assert "ROBOT_ISOLATED" not in world.codes("pode2")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert world.result_of(call).success and "ROBOT_ISOLATED" not in world.codes("pode2")


# ---- 6. 옛 기록(키 없음)은 예전과 같다 ----

def test_without_the_keys_a_failed_check_retries_as_before():
    old = replace(RELOC, relocalize=False)
    world, _ = queued(old)
    wrong_from(world, "pode1", 4.0)
    assert world.run_until(lambda: "ROBOT_ISOLATED" in world.codes("pode1"), 120.0)
    assert world.codes("pode1").count("GOAL_RETRY") == old.move_retry_limit
    assert not world.sent(RelocalizeCmd) and "RELOCALIZE_SENT" not in world.codes()


def test_a_success_without_a_valid_pose_counts_as_a_failure():
    """pose_valid가 거짓이면 자세를 쓰지 않는다: 성공이라도 실패로 본다."""
    world, _ = queued(RELOC)
    wrong_from(world, "pode1", 4.0)
    assert world.run_until(lambda: world.sent(RelocalizeCmd, "pode1"), 60.0)
    first_try = world.sent(RelocalizeCmd, "pode1")[0]
    world.reloc_silent.add("pode1")
    world.step()
    world.traffic.on_relocalize_result("pode1", first_try.goal_id, True, "", None, None)
    world.step()
    assert "RELOCALIZE_FAILED" in world.codes("pode1")
    assert "RELOCALIZED" not in world.codes("pode1")
    assert "no valid pose" in event(world, "RELOCALIZE_FAILED", "pode1").detail


def test_busy_is_not_a_failure_and_the_same_attempt_is_sent_again():
    """BUSY: 로봇이 아직 움직이는 중이라 받자마자 끝냈다. 시도 수를 쓰지 않고 다시 보낸다."""
    world, _ = queued(RELOC)
    wrong_from(world, "pode1", 4.0)
    assert world.run_until(lambda: world.sent(RelocalizeCmd, "pode1"), 60.0)
    first_try = world.sent(RelocalizeCmd, "pode1")[0]
    world.bots["pode1"].reloc = None  # 로봇이 이 goal을 BUSY로 끝냈다
    world.traffic.on_relocalize_result("pode1", first_try.goal_id, False, "BUSY", None, None)
    assert world.run_until(lambda: len(world.sent(RelocalizeCmd, "pode1")) == 2, 5.0)
    again = world.sent(RelocalizeCmd, "pode1")[1]
    assert (again.allow_spin, again.allow_global) == (True, False), "still the first attempt"
    assert "RELOCALIZE_FAILED" not in world.codes("pode1")
    assert world.run_until(lambda: "RELOCALIZED" in world.codes("pode1"), 10.0)


# ---- 운영 입력(FleetManager)도 같은 길을 지난다 ----

def test_the_manager_passes_the_relocalize_result_on():
    world = Harness(starts={"pode1": "W", "pode2": "O", "pode3": "P-3"}, params=RELOC,
                    tick_order="manager", manager_inputs=True)
    world.report()
    world.call("OUTBOUND", "pode2")
    world.call("OUTBOUND", "pode1")
    wrong_from(world, "pode1", 4.0)
    assert world.run_until(lambda: "RELOCALIZED" in world.codes("pode1"), 60.0)
    assert not world.problems


def test_the_manager_refuses_a_relocalize_result_for_an_unknown_robot():
    world = Harness(params=RELOC, tick_order="manager", manager_inputs=True)
    world.manager.on_relocalize_result("nobody", 1, True, "", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    out = world.manager.tick()
    assert any(getattr(e, "code", "") == "INPUT_REJECTED" for e in out)


def test_a_result_for_another_goal_is_ignored():
    world = Harness(params=RELOC)
    world.run(1.0)
    world.traffic.on_relocalize_result("pode1", 12345, True, "", (1.0, 1.0, 0.0), None)
    m = robot(world, "pode1")
    assert m.reloc is None and not math.isclose(m.x, 1.0)
