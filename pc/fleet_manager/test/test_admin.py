"""관리 명령 판단과 같은 번호 처리 (W-30 · W-32 · W-34 · W-53, IR-20 · IR-60 · IR-63)."""
from pathlib import Path

from fleet_core.common.contract import load_contract
from fleet_manager.admin import (
    AdminCommands,
    Codes,
    Facts,
    Reply,
    RequestCache,
    Targets,
    judge_set_lock,
    judge_target,
    judge_test_only,
)

CONTRACT = Path(__file__).parents[3] / "interfaces" / "fleet_interfaces" / "contract.yaml"
CODES = Codes(load_contract(CONTRACT).reason_codes)
TARGETS = Targets(
    set_lock=frozenset({"pode1", "arm_left", "arm_ship"}),
    fault_resolved=frozenset({"arm_left", "arm_right"}),
    charge_end=frozenset({"pode1"}),
    force_input=frozenset({"pode1", "arm_left"}))


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def admin(clock=None, size=4, keep_s=60.0):
    reports = []
    a = AdminCommands(CODES, lambda *r: reports.append(r), clock or Clock(), size, keep_s)
    return a, reports


# ---- set_lock 판단 ----
def facts(stopped=True, holding=False):
    return lambda _robot: Facts(stopped, holding, "" if stopped else "moving")


def test_an_unknown_robot_is_unknown_target():
    reply = judge_set_lock(CODES, TARGETS, "pode9", False, False, facts())
    assert (reply.accepted, reply.reason) == (False, "UNKNOWN_TARGET")


def test_phony_takes_set_lock_by_spec_v02():
    assert judge_set_lock(CODES, TARGETS, "arm_ship", False, False, facts()).accepted


def test_a_robot_that_is_not_stopped_is_refused():
    reply = judge_set_lock(CODES, TARGETS, "pode1", False, False, facts(stopped=False))
    assert (reply.accepted, reply.reason, reply.detail) == (False, "NOT_STOPPED", "moving")


def test_an_arm_holding_an_item_needs_confirm_drop_to_release():
    held = facts(holding=True)
    assert judge_set_lock(CODES, TARGETS, "arm_left", False, False, held).reason == "HOLDING_ITEM"
    assert judge_set_lock(CODES, TARGETS, "arm_left", False, True, held).accepted
    # 잠글 때는 쥔 상품과 상관없다
    assert judge_set_lock(CODES, TARGETS, "arm_left", True, False, held).accepted


# ---- 로봇 대상 단순 명령 ----
def test_fault_resolved_goes_only_to_the_phody_arms():
    assert judge_target(CODES, TARGETS.fault_resolved, "arm_right").accepted
    assert judge_target(CODES, TARGETS.fault_resolved, "arm_ship").reason == "UNKNOWN_TARGET"
    assert judge_target(CODES, TARGETS.fault_resolved, "pode1").reason == "UNKNOWN_TARGET"


def test_test_only_commands_need_the_test_mode():
    assert judge_test_only(CODES, TARGETS.charge_end, "pode1", "TEST").accepted
    for mode in ("NORMAL", None):
        reply = judge_test_only(CODES, TARGETS.charge_end, "pode1", mode)
        assert (reply.accepted, reply.reason) == (False, "NOT_TEST_MODE")
    # 대상 확인이 먼저다
    assert judge_test_only(CODES, TARGETS.charge_end, "arm_left", "NORMAL").reason == \
        "UNKNOWN_TARGET"


# ---- 같은 번호, ReportCommand ----
def test_a_local_command_reports_once_and_the_same_id_gets_the_same_reply():
    a, reports = admin()
    runs = []

    def apply():
        runs.append(1)
        return Reply(True, "", "", ("Z1",))

    first = a.local("confirm_clear", "id-1", "pode1", apply)
    again = a.local("confirm_clear", "id-1", "pode1", apply)
    assert first == again and first.extra == ("Z1",)
    assert len(runs) == 1
    assert reports == [("id-1", "confirm_clear", "pode1", "ACCEPTED", "", "")]


def test_a_refused_local_command_reports_rejected_with_its_code():
    a, reports = admin()
    a.local("reinstate", "id-2", "pode1", lambda: Reply(False, "LOCK_RELEASED", "lock is off"))
    assert reports == [("id-2", "reinstate", "pode1", "REJECTED", "LOCK_RELEASED", "lock is off")]


def test_a_robot_command_is_sent_once_when_accepted_and_reported_when_refused():
    a, reports = admin()
    sent = []
    ok = a.robot("set_lock", "id-3", "pode1", lambda: Reply(True), lambda: sent.append(1))
    a.robot("set_lock", "id-3", "pode1", lambda: Reply(True), lambda: sent.append(1))
    assert ok.accepted and sent == [1] and reports == []
    a.robot("set_lock", "id-4", "pode1", lambda: Reply(False, "NOT_STOPPED", "x"),
            lambda: sent.append(2))
    assert sent == [1]
    assert reports == [("id-4", "set_lock", "pode1", "REJECTED", "NOT_STOPPED", "x")]


def test_an_empty_request_id_is_not_remembered():
    a, reports = admin()
    a.local("reinstate", "", "pode1", lambda: Reply(True))
    a.local("reinstate", "", "pode1", lambda: Reply(True))
    assert len(reports) == 2


def test_the_cache_forgets_by_count_and_by_age():
    cache = RequestCache(size=2, keep_s=10.0)
    cache.put("a", Reply(True), 0.0)
    cache.put("b", Reply(True), 1.0)
    cache.put("c", Reply(True), 2.0)
    assert cache.get("a", 2.0) is None and cache.get("b", 2.0) is not None
    assert cache.get("b", 11.5) is None and cache.get("c", 11.5) is not None


def test_an_id_seen_again_after_it_was_forgotten_runs_again():
    clock = Clock()
    a, reports = admin(clock, keep_s=5.0)
    a.local("reinstate", "id-5", "pode1", lambda: Reply(True))
    clock.t = 6.0
    a.local("reinstate", "id-5", "pode1", lambda: Reply(True))
    assert len(reports) == 2
