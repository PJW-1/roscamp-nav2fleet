"""래퍼의 goal 관리. 순수 파이썬이라 ROS 없이도 돈다."""
from fleet_manager.goal_tracker import GoalTracker, ReportOutbox

RETRY_S = 1.0
WARN_AFTER = 3
LIMIT = 4


def middle():
    # 지연 배수 1.0: 기존 시각 시험이 지터 없이 돈다
    return 0.5


class _Handle:
    def __init__(self):
        self.cancels = 0

    def cancel_goal_async(self):
        self.cancels += 1


def running(key="k"):
    goals = GoalTracker(rand=middle)
    handle = _Handle()
    goals.sent(key)
    goals.accepted(key, handle)
    return goals, handle


def retry(goals, now):
    return goals.retry_cancels(now, RETRY_S, WARN_AFTER)


def test_cancel_of_an_accepted_goal_goes_to_its_handle():
    goals, handle = running()
    goals.cancel("k", 0.0)
    assert handle.cancels == 1


def test_cancel_before_the_server_accepted_is_applied_on_acceptance():
    goals = GoalTracker(rand=middle)
    handle = _Handle()
    goals.sent("k")
    goals.cancel("k", 0.0)
    assert handle.cancels == 0
    goals.accepted("k", handle)
    assert handle.cancels == 1


def test_cancel_of_an_unknown_or_finished_goal_is_ignored():
    goals = GoalTracker(rand=middle)
    handle = _Handle()
    goals.cancel("never-sent", 0.0)
    goals.sent("k")
    goals.accepted("k", handle)
    goals.finished("k")
    goals.cancel("k", 0.0)
    assert handle.cancels == 0
    goals.sent("never-sent")
    goals.accepted("never-sent", handle)
    assert handle.cancels == 0, "the old cancel must not hit a later goal"
    assert retry(goals, 10.0) == []
    assert handle.cancels == 0


def test_a_goal_is_canceling_from_the_cancel_until_its_end():
    goals, handle = running()
    assert not goals.canceling("k")
    goals.cancel("k", 0.0)
    assert goals.canceling("k")
    goals.finished("k")
    assert not goals.canceling("k")


def test_a_lost_cancel_is_warned_about_once_and_then_dropped():
    """결과를 잃은 취소는 결과가 다시 오지 않는다. 경고까지만 다시 보내고 그만둔다."""
    goals, handle = running()
    goals.cancel("k", 0.0)
    goals.lost("k")
    assert [retry(goals, now) for now in (1.0, 2.0, 3.0)] == [[], [], ["k"]]
    assert handle.cancels == 4
    assert retry(goals, 4.0) == []
    assert not goals.canceling("k") and handle.cancels == 4


def test_a_dropped_lost_cancel_is_handed_out_once_so_the_core_can_hear_of_it():
    """그만둔 취소는 한 번 돌려준다. 노드가 코어에 on_goal_lost로 알린다."""
    goals, handle = running()
    goals.cancel("k", 0.0)
    goals.lost("k")
    for now in (1.0, 2.0, 3.0):
        retry(goals, now)
        assert goals.take_dropped() == []
    retry(goals, 4.0)
    assert goals.take_dropped() == ["k"]
    assert goals.take_dropped() == []


def test_a_cancel_ended_by_its_result_is_not_handed_out_as_dropped():
    goals, handle = running()
    goals.cancel("k", 0.0)
    goals.finished("k")
    for now in (1.0, 2.0, 3.0, 4.0, 5.0):
        retry(goals, now)
    assert goals.take_dropped() == []


def test_a_goal_not_being_canceled_is_not_kept_as_lost():
    goals, handle = running()
    goals.lost("k")
    goals.cancel("k", 0.0)
    assert [retry(goals, now) for now in (1.0, 2.0, 3.0, 4.0)] == [[], [], ["k"], []]
    assert goals.canceling("k")


def test_rejected_goal_forgets_its_pending_cancel():
    goals = GoalTracker(rand=middle)
    goals.sent("k")
    goals.cancel("k", 0.0)
    goals.finished("k")
    handle = _Handle()
    goals.sent("k")
    goals.accepted("k", handle)
    assert handle.cancels == 0
    assert retry(goals, 10.0) == []


def test_a_goal_nobody_asked_to_cancel_gets_no_cancel():
    goals, handle = running()
    assert retry(goals, 10.0) == []
    assert handle.cancels == 0


def test_an_unfinished_cancel_goes_out_again_every_retry_period():
    goals, handle = running()
    goals.cancel("k", 10.0)
    retry(goals, 10.9)
    assert handle.cancels == 1, "not due yet"
    retry(goals, 11.0)
    assert handle.cancels == 2
    retry(goals, 11.5)
    assert handle.cancels == 2, "the period runs from the last try"
    retry(goals, 12.0)
    assert handle.cancels == 3


def test_the_warning_comes_once_after_the_last_of_the_tries_went_unanswered():
    goals, handle = running()
    goals.cancel("k", 0.0)
    assert retry(goals, 1.0) == []
    assert retry(goals, 2.0) == [], "the third try has only just gone out"
    assert handle.cancels == 3
    assert retry(goals, 3.0) == ["k"]
    assert handle.cancels == 4, "it keeps trying"
    assert retry(goals, 4.0) == [], "one warning"
    assert retry(goals, 5.0) == []
    assert handle.cancels == 6


def test_the_result_ends_the_retries_and_the_warning():
    goals, handle = running()
    goals.cancel("k", 0.0)
    retry(goals, 1.0)
    goals.finished("k")
    assert retry(goals, 5.0) == []
    assert handle.cancels == 2


def test_asking_again_does_not_start_the_count_over():
    goals, handle = running()
    goals.cancel("k", 0.0)
    goals.cancel("k", 0.9)
    assert handle.cancels == 1
    assert retry(goals, 1.0) == []
    assert handle.cancels == 2, "the period runs from the first ask"
    assert retry(goals, 2.0) == []
    assert retry(goals, 3.0) == ["k"]


def test_each_goal_has_its_own_count():
    goals, first = running("a")
    second = _Handle()
    goals.sent("b")
    goals.accepted("b", second)
    goals.cancel("a", 0.0)
    assert retry(goals, 1.0) == []
    goals.cancel("b", 1.5)
    assert retry(goals, 2.0) == []
    assert retry(goals, 3.0) == ["a"]
    assert (first.cancels, second.cancels) == (4, 2)
    assert retry(goals, 4.0) == []
    assert retry(goals, 5.0) == ["b"]


def test_a_cancel_waiting_for_an_acceptance_that_never_comes_is_warned_about_too():
    """핸들이 없으면 보낼 수 없지만 goal은 로봇에서 계속 돌 수 있다."""
    goals = GoalTracker(rand=middle)
    goals.sent("k")
    goals.cancel("k", 0.0)
    assert retry(goals, 1.0) == []
    assert retry(goals, 2.0) == []
    assert retry(goals, 3.0) == ["k"]
    assert retry(goals, 4.0) == []
    handle = _Handle()
    goals.accepted("k", handle)
    assert handle.cancels == 1
    retry(goals, 5.0)
    assert handle.cancels == 2, "accepted late, and it keeps being asked"


def due(outbox, now, ready=True):
    return [item for _, item in outbox.due(now, ready, RETRY_S, LIMIT)]


def test_outbox_keeps_reports_until_the_service_is_ready():
    outbox = ReportOutbox(rand=middle)
    outbox.add("a")
    outbox.add("b")
    assert due(outbox, 0.0, ready=False) == []
    assert due(outbox, 0.0) == ["a", "b"]


def test_outbox_sends_a_report_again_only_after_the_retry_period():
    outbox = ReportOutbox(rand=middle)
    outbox.add("a")
    assert due(outbox, 10.0) == ["a"]
    assert due(outbox, 10.9) == []
    assert due(outbox, 11.0) == ["a"]
    assert due(outbox, 11.5) == [], "the period runs from the last send"
    assert due(outbox, 12.0) == ["a"]


def test_outbox_sends_nothing_while_the_service_is_gone_and_at_once_when_it_is_back():
    outbox = ReportOutbox(rand=middle)
    outbox.add("a")
    assert due(outbox, 0.0) == ["a"]
    assert due(outbox, 5.0, ready=False) == []
    assert due(outbox, 5.1) == ["a"]


def test_outbox_drops_a_report_once_it_is_done():
    outbox = ReportOutbox(rand=middle)
    outbox.add("a")
    outbox.add("b")
    (first, _), (second, _) = outbox.due(0.0, True, RETRY_S, LIMIT)
    assert outbox.finish(first, None) == []
    assert due(outbox, 5.0) == ["b"]
    assert outbox.finish(second, None) == []
    assert due(outbox, 10.0) == []


def test_outbox_says_when_a_second_reply_finds_the_report_already_gone():
    """두 번 보낸 보고는 답도 두 번 올 수 있다. 이벤트는 첫 답만 낼 수 있다."""
    outbox = ReportOutbox(rand=middle)
    outbox.add("a")
    [(report_id, _)] = outbox.due(0.0, True, RETRY_S, LIMIT)
    assert outbox.finish(report_id, None) == []
    assert outbox.finish(report_id, None) is None


def test_outbox_takes_reports_that_cannot_be_hashed():
    outbox = ReportOutbox(rand=middle)
    outbox.add({"placed": {"A": 1}})
    outbox.add({"placed": {"A": 1}})
    assert due(outbox, 0.0) == [{"placed": {"A": 1}}, {"placed": {"A": 1}}]


# ---- IR-41: 재전송 지연 ±20%, 동시 보고 상한, 앞선 요청 지우기 ----
class _Draws:
    """정해 둔 난수를 차례로 낸다."""

    def __init__(self, *values):
        self.values = list(values)
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.values.pop(0) if self.values else 0.5


def test_the_cancel_retry_waits_at_least_0_8_of_the_period_at_the_low_end():
    goals = GoalTracker(rand=lambda: 0.0)
    handle = _Handle()
    goals.sent("k")
    goals.accepted("k", handle)
    goals.cancel("k", 10.0)
    retry(goals, 10.79)
    assert handle.cancels == 1
    retry(goals, 10.8)
    assert handle.cancels == 2


def test_the_cancel_retry_waits_at_most_1_2_of_the_period_at_the_high_end():
    goals = GoalTracker(rand=lambda: 1.0)
    handle = _Handle()
    goals.sent("k")
    goals.accepted("k", handle)
    goals.cancel("k", 10.0)
    retry(goals, 11.19)
    assert handle.cancels == 1
    retry(goals, 11.201)
    assert handle.cancels == 2


def test_each_cancel_retry_draws_its_own_delay():
    draws = _Draws(0.0, 1.0)  # 첫 기다림 0.8, 다음 1.2
    goals = GoalTracker(rand=draws)
    handle = _Handle()
    goals.sent("k")
    goals.accepted("k", handle)
    goals.cancel("k", 0.0)
    retry(goals, 0.8)
    assert handle.cancels == 2
    retry(goals, 1.99)
    assert handle.cancels == 2
    retry(goals, 2.001)
    assert handle.cancels == 3
    assert draws.calls == 3


def test_the_default_cancel_delay_stays_within_20_percent():
    for _ in range(50):
        goals = GoalTracker()
        handle = _Handle()
        goals.sent("k")
        goals.accepted("k", handle)
        goals.cancel("k", 0.0)
        retry(goals, 0.7999)
        assert handle.cancels == 1
        retry(goals, 1.201)
        assert handle.cancels == 2


def test_the_report_retry_waits_between_0_8_and_1_2_of_the_period():
    for value, early, late in ((0.0, 10.79, 10.8), (1.0, 11.19, 11.201)):
        outbox = ReportOutbox(rand=lambda v=value: v)
        outbox.add("a")
        assert due(outbox, 10.0) == ["a"]
        assert due(outbox, early) == []
        assert due(outbox, late) == ["a"]


def test_each_report_resend_draws_its_own_delay():
    outbox = ReportOutbox(rand=_Draws(1.0, 0.0))  # 첫 기다림 1.2, 다음 0.8
    outbox.add("a")
    assert due(outbox, 0.0) == ["a"]
    assert due(outbox, 1.19) == []
    assert due(outbox, 1.201) == ["a"]
    assert due(outbox, 1.99) == []
    assert due(outbox, 2.01) == ["a"]


def test_the_default_report_delay_stays_within_20_percent():
    for _ in range(50):
        outbox = ReportOutbox()
        outbox.add("a")
        assert due(outbox, 0.0) == ["a"]
        assert due(outbox, 0.7999) == []
        assert due(outbox, 1.201) == ["a"]


def test_no_more_requests_than_the_limit_wait_for_an_answer():
    """IR-41: 상한은 답을 기다리는 요청 수다. 앞선 요청이 한도 안에 남은 보고는
    다시 보낼 때도 자리를 하나 더 차지한다."""
    outbox = ReportOutbox(rand=middle)
    for name in "abcde":
        outbox.add(name)
    sent = outbox.due(0.0, True, RETRY_S, 2)
    assert [item for _, item in sent] == ["a", "b"]
    assert due_limited(outbox, 0.5, 2) == [], "the others wait for a free place"
    outbox.finish(sent[0][0], None)
    assert due_limited(outbox, 0.6, 2) == ["c"]
    assert due_limited(outbox, 1.0, 2) == [], "b still waits for its first answer"
    outbox.expire(3.0, 3.0)  # 0.0에 보낸 b의 요청이 한도를 넘었다
    assert due_limited(outbox, 3.0, 2) == ["b"]


def due_limited(outbox, now, limit):
    return [item for _, item in outbox.due(now, True, RETRY_S, limit)]


def test_only_requests_past_the_reply_limit_are_handed_back():
    """IR-41: 응답 한도를 넘긴 요청만 버린다. 한도 안의 앞선 요청은 늦게 와도 보고를 끝낸다."""
    outbox = ReportOutbox(rand=middle)
    outbox.add("a")
    [(report_id, _)] = outbox.due(0.0, True, RETRY_S, LIMIT)
    first, second = object(), object()
    outbox.track(report_id, first)
    assert due(outbox, 2.0) == ["a"]
    outbox.track(report_id, second)
    assert outbox.expire(2.9, 3.0) == [], "first is inside the 3 s limit"
    assert outbox.expire(3.0, 3.0) == [first]
    assert outbox.expire(3.5, 3.0) == []
    assert outbox.finish(report_id, second) == []
    assert outbox.finish(report_id, second) is None, "a done report is finished once"


def test_finishing_a_report_hands_back_its_other_waiting_requests():
    outbox = ReportOutbox(rand=middle)
    outbox.add("a")
    [(report_id, _)] = outbox.due(0.0, True, RETRY_S, LIMIT)
    first, second = object(), object()
    outbox.track(report_id, first)
    assert due(outbox, 1.0) == ["a"]
    outbox.track(report_id, second)
    assert outbox.finish(report_id, first) == [second]


def test_a_failed_request_frees_its_place():
    outbox = ReportOutbox(rand=middle)
    outbox.add("a")
    outbox.add("b")
    [(report_id, _)] = outbox.due(0.0, True, RETRY_S, 1)
    failed = object()
    outbox.track(report_id, failed)
    outbox.forget(report_id, failed)
    assert due_limited(outbox, 0.1, 1) == ["b"]
