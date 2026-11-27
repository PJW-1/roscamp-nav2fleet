"""로봇까지 가는 관리 명령: 2초 한도 · 1초 간격 · 5번, 결과 모름과 확정 (W-31 · IR-40 · IR-62)."""
from pathlib import Path

import pytest
from fleet_core.common.contract import load_contract
from fleet_manager.admin import Codes
from fleet_manager.robot_calls import CallParams, RobotCalls

CONTRACT = Path(__file__).parents[3] / "interfaces" / "fleet_interfaces" / "contract.yaml"
CODES = Codes(load_contract(CONTRACT).reason_codes)


class Box:
    def __init__(self):
        self.reports, self.events, self.then = [], [], []

    def report(self, request_id, command, target, result, reason="", detail=""):
        self.reports.append((request_id, command, target, result, reason))

    def event(self, level, code, detail, source):
        self.events.append((level, code, source))


def make(params=None, rand=lambda: 0.5):
    box = Box()
    calls = RobotCalls(params or CallParams(), CODES, box.report, box.event, rand)
    return calls, box


def add(calls, box, now=0.0, observe=None, rid="r1"):
    calls.add(rid, "set_lock", "pode1", "C", f"req-{rid}", now, observe=observe,
              then=box.then.append)


def run(calls, until, ready=True, dt=0.1, start=0.0):
    """future 없이 시간을 흘린다. 보낸 시각을 돌려준다."""
    sent, drops, i = [], [], 0
    while (t := round(start + i * dt, 6)) <= until:
        i += 1
        step = calls.step(t, lambda _c: ready)
        for request_id, _client, _req in step.send:
            sent.append(t)
            calls.track(request_id, object())
        drops += step.drop
    return sent, drops


def test_an_answer_on_the_first_try_is_reported_once_as_accepted():
    calls, box = make()
    add(calls, box)
    step = calls.step(0.0, lambda _c: True)
    assert step.send == [("r1", "C", "req-r1")]
    future = object()
    calls.track("r1", future)
    assert calls.on_answer("r1", future, True) == []
    assert box.reports == [("r1", "set_lock", "pode1", "ACCEPTED", "")]
    assert box.then == [True]
    assert calls.on_answer("r1", future, True) == []  # 두 번째 답은 버린다
    assert len(box.reports) == 1


def test_a_refusal_is_rejected_with_robot_rejected_and_the_robot_reason_in_detail():
    calls, box = make()
    seen = []
    calls._report = lambda *a: seen.append(a)
    add(calls, box)
    calls.step(0.0, lambda _c: True)
    calls.on_answer("r1", None, False, "NOT_STOPPED", "the robot has a goal")
    assert seen == [("r1", "set_lock", "pode1", "REJECTED", "ROBOT_REJECTED",
                     "NOT_STOPPED the robot has a goal")]
    assert box.then == [False]


def test_five_tries_two_seconds_apart_plus_one_then_unconfirmed():
    calls, box = make()
    add(calls, box)
    sent, drops = run(calls, 14.5)
    assert sent == [0.0, 3.0, 6.0, 9.0, 12.0]
    # 앞 네 번은 한도에서 버리고, 마지막은 늦은 답을 받으려고 쥔다
    assert len(drops) == 4
    assert box.reports == [("r1", "set_lock", "pode1", "UNCONFIRMED", "ROBOT_TIMEOUT")]
    assert box.then == []


def test_the_gap_takes_a_jitter_of_twenty_percent():
    calls, box = make(rand=lambda: 1.0)
    add(calls, box)
    sent, _ = run(calls, 4.0)
    assert sent == [0.0, 3.2]


def test_a_late_answer_after_unconfirmed_confirms_once():
    calls, box = make()
    add(calls, box)
    run(calls, 14.5)
    calls.on_answer("r1", None, True)
    assert [r[3] for r in box.reports] == ["UNCONFIRMED", "ACCEPTED"]
    assert box.then == [True]
    run(calls, 40.0, start=14.6)
    assert len(box.reports) == 2 and box.events == []


def test_an_observation_confirms_an_unconfirmed_call():
    calls, box = make()
    add(calls, box, observe=lambda view: view == "unlocked")
    run(calls, 14.5)
    assert calls.observe("pode1", "locked") == []
    drops = calls.observe("pode1", "unlocked")
    assert len(drops) == 1  # 쥐고 있던 요청을 버린다
    assert [r[3] for r in box.reports] == ["UNCONFIRMED", "ACCEPTED"]


def test_an_observation_does_not_end_a_call_still_sending():
    calls, box = make()
    add(calls, box, observe=lambda view: True)
    calls.step(0.0, lambda _c: True)
    assert calls.observe("pode1", "x") == []
    assert box.reports == []


def test_no_result_thirty_seconds_after_the_first_request_is_timeout_with_an_event():
    calls, box = make()
    add(calls, box)
    _, drops = run(calls, 30.1)
    assert [r[3] for r in box.reports] == ["UNCONFIRMED", "TIMEOUT"]
    assert box.reports[-1][4] == "ROBOT_TIMEOUT"
    assert box.events == [("WARN", "LATE_COMMAND_RESULT", "pode1")]
    assert len(drops) == 5 and box.then == [None]
    assert "r1" not in calls


def test_a_missing_server_uses_up_tries_without_sending():
    calls, box = make()
    add(calls, box)
    sent, _ = run(calls, 14.5, ready=False)
    assert sent == []
    assert [r[3] for r in box.reports] == ["UNCONFIRMED"]


def test_the_number_of_requests_waiting_is_capped():
    calls, box = make(CallParams(max_in_flight=2))
    for rid in ("a", "b", "c"):
        add(calls, box, rid=rid)
    step = calls.step(0.0, lambda _c: True)
    assert [s[0] for s in step.send] == ["a", "b"]
    for rid, _, _ in step.send:
        calls.track(rid, object())
    calls.on_answer("a", None, True)
    step = calls.step(0.1, lambda _c: True)
    assert [s[0] for s in step.send] == ["c"]


def test_a_failed_future_frees_its_place_and_the_try_runs_out():
    calls, box = make()
    add(calls, box)
    calls.step(0.0, lambda _c: True)
    future = object()
    calls.track("r1", future)
    calls.on_failed("r1", future)
    step = calls.step(2.0, lambda _c: True)
    assert step.drop == [] and step.send == []
    assert calls.step(3.0, lambda _c: True).send


@pytest.mark.parametrize("bad", ["NOPE", "_hidden"])
def test_codes_only_give_names_of_the_contract(bad):
    with pytest.raises(AttributeError):
        getattr(CODES, bad)
    assert CODES.LOCK_RELEASED == "LOCK_RELEASED"


def test_a_held_last_request_still_counts_against_the_cap():
    calls, box = make(CallParams(max_in_flight=1))
    add(calls, box, rid="a")
    run(calls, 14.5)  # a는 결과 모름, 마지막 요청을 쥔다
    add(calls, box, now=14.5, rid="b")
    sent, _ = run(calls, 20.0, start=14.6)
    assert sent == []


def test_a_call_starved_by_the_cap_still_times_out_at_thirty_seconds():
    calls, box = make(CallParams(max_in_flight=1))
    add(calls, box, rid="a")
    add(calls, box, rid="b")
    run(calls, 30.1)
    by_id = {}
    for rid, _c, _t, result, _r in box.reports:
        by_id.setdefault(rid, []).append(result)
    assert by_id["b"] == ["TIMEOUT"] and "b" not in calls


def test_an_answer_or_observation_past_the_deadline_does_not_confirm():
    calls, box = make()
    add(calls, box, observe=lambda view: True)
    run(calls, 14.5)
    assert calls.observe("pode1", "x", now=30.5) == []
    assert calls.on_answer("r1", None, True, now=30.5) == []
    calls.step(30.5, lambda _c: True)
    assert [r[3] for r in box.reports] == ["UNCONFIRMED", "TIMEOUT"]


def test_pending_names_the_open_call_of_a_command_on_a_target():
    calls, box = make()
    assert calls.pending("set_lock", "pode1") is None
    add(calls, box)
    assert calls.pending("set_lock", "pode1") == "r1"
    assert calls.pending("set_lock", "pode2") is None
    calls.on_answer("r1", None, True)
    assert calls.pending("set_lock", "pode1") is None


def test_one_request_id_can_call_two_robots_under_their_own_keys_and_report_sinks():
    calls, box = make()
    seen = []
    for arm in ("arm_left", "arm_right"):
        calls.add("r1", "set_pause", arm, arm, f"req-{arm}", 0.0, key=("r1", arm),
                  report=lambda *r: seen.append(r[:4]))
    assert ("r1", "arm_left") in calls and ("r1", "arm_right") in calls and "r1" not in calls
    step = calls.step(0.0, lambda _c: True)
    assert [s[0] for s in step.send] == [("r1", "arm_left"), ("r1", "arm_right")]
    assert calls.pending("set_pause", "arm_right") == "r1"
    calls.on_answer(("r1", "arm_left"), None, True)
    calls.on_answer(("r1", "arm_right"), None, False, "NOT_PAUSED")
    assert seen == [("r1", "set_pause", "arm_left", "ACCEPTED"),
                    ("r1", "set_pause", "arm_right", "REJECTED")]
    assert box.reports == [], "a call with its own sink does not use the node report"


def test_a_per_robot_key_never_collides_with_a_plain_request_id():
    calls, box = make()
    calls.add("abc@arm_left", "set_lock", "arm_left", "C", "req", 0.0)
    calls.add("abc", "set_pause", "arm_left", "C", "req", 0.0, key=("abc", "arm_left"),
              report=lambda *r: None)
    assert "abc@arm_left" in calls and ("abc", "arm_left") in calls


# ---- 받았다는 답도 상태로 확정 (fault_resolved: 보낼 때 ERROR였던 팔) ----
def add_confirm(calls, box, observe, rid="f1"):
    calls.add(rid, "fault_resolved", "arm_left", "C", f"req-{rid}", 0.0, observe=observe,
              confirm_accepted=True)


def left_error(view):
    return view != "ERROR"


def test_an_accepted_answer_waits_for_the_state_then_is_accepted_once():
    calls, box = make()
    add_confirm(calls, box, left_error)
    calls.step(0.0, lambda _c: True)
    calls.on_answer("f1", None, True, now=0.5, view="ERROR")
    run(calls, 5.0, start=0.6)
    assert box.reports == []
    calls.observe("arm_left", "ERROR", 5.5)
    assert box.reports == []
    calls.observe("arm_left", "IDLE", 6.0)
    assert box.reports == [("f1", "fault_resolved", "arm_left", "ACCEPTED", "")]


def test_an_accepted_answer_whose_state_never_confirms_times_out_at_thirty_seconds():
    calls, box = make()
    add_confirm(calls, box, left_error)
    calls.step(0.0, lambda _c: True)
    calls.on_answer("f1", None, True, now=0.5, view="ERROR")
    run(calls, 30.5, start=0.6)
    assert box.reports == [("f1", "fault_resolved", "arm_left", "TIMEOUT", "ROBOT_TIMEOUT")]


def test_a_state_already_out_of_error_at_the_answer_confirms_at_once():
    calls, box = make()
    add_confirm(calls, box, left_error)
    calls.step(0.0, lambda _c: True)
    calls.on_answer("f1", None, True, now=0.5, view="IDLE")
    assert box.reports == [("f1", "fault_resolved", "arm_left", "ACCEPTED", "")]


def test_a_late_accepted_answer_after_unconfirmed_still_waits_for_the_state():
    calls, box = make()
    add_confirm(calls, box, left_error)
    run(calls, 16.0)
    assert [r[3] for r in box.reports] == ["UNCONFIRMED"]
    calls.on_answer("f1", None, True, now=16.5, view="ERROR")
    assert [r[3] for r in box.reports] == ["UNCONFIRMED"]
    calls.observe("arm_left", "IDLE", 17.0)
    assert [r[3] for r in box.reports] == ["UNCONFIRMED", "ACCEPTED"]


def test_a_refusal_still_ends_rejected_and_no_observer_means_accepted_at_once():
    calls, box = make()
    add_confirm(calls, box, left_error)
    calls.step(0.0, lambda _c: True)
    calls.on_answer("f1", None, False, "NOT_READY", now=0.5, view="ERROR")
    add_confirm(calls, box, None, rid="f2")
    calls.step(1.0, lambda _c: True)
    calls.on_answer("f2", None, True, now=1.5, view="IDLE")
    assert [(r[0], r[3]) for r in box.reports] == [("f1", "REJECTED"), ("f2", "ACCEPTED")]


def test_without_confirm_accepted_an_accepted_answer_ends_at_once_even_with_an_observer():
    calls, box = make()
    calls.add("p1", "arm_set_pause", "arm_left", "C", "req", 0.0, observe=left_error)
    calls.step(0.0, lambda _c: True)
    calls.on_answer("p1", None, True, now=0.5, view="ERROR")
    assert [r[3] for r in box.reports] == ["ACCEPTED"]


def test_an_exit_seen_while_sending_counts_even_if_the_arm_is_back_in_error_at_the_answer():
    calls, box = make()
    add_confirm(calls, box, left_error)
    calls.step(0.0, lambda _c: True)
    calls.observe("arm_left", "IDLE", 0.3)
    assert box.reports == [], "still sending: no end state yet"
    calls.on_answer("f1", None, True, now=0.5, view="ERROR")
    assert box.reports == [("f1", "fault_resolved", "arm_left", "ACCEPTED", "")]


def test_an_exit_seen_while_sending_confirms_when_no_answer_ever_comes():
    calls, box = make()
    add_confirm(calls, box, left_error)
    calls.step(0.0, lambda _c: True)
    calls.track("f1", object())
    calls.observe("arm_left", "IDLE", 0.3)
    run(calls, 16.0, start=0.1)
    assert [r[3] for r in box.reports] == ["ACCEPTED"]


def test_an_exit_seen_before_the_request_ever_went_out_does_not_count():
    calls, box = make()
    add_confirm(calls, box, left_error)
    run(calls, 0.2, ready=False)
    calls.observe("arm_left", "IDLE", 0.3)  # 서비스가 없어 아직 한 번도 못 보냄
    run(calls, 16.0, ready=False, start=0.4)
    assert [r[3] for r in box.reports] == ["UNCONFIRMED"]
