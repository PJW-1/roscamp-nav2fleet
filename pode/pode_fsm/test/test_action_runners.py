"""세 Nav2 액션 실행기(FollowPath, NavigateToPose, BackUp)의 거절, 취소, 결과 처리.

ROS 노드와 실행기 없이 가짜 ActionClient, 가짜 future로 콜백 순서만 본다.
"""
import importlib
from types import SimpleNamespace

import pytest

pytest.importorskip("action_msgs.srv")
pytest.importorskip("nav2_msgs.action")


# ---- 가짜 future, 핸들, 클라이언트, 노드 ----
class FakeFuture:
    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, cb):
        self.callbacks.append(cb)

    def resolve(self, value):
        self.result = lambda: value
        for cb in self.callbacks:
            cb(self)

    def fail(self, exc):
        def boom():
            raise exc
        self.result = boom
        for cb in self.callbacks:
            cb(self)


class FakeHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancels = []
        self.result_future = FakeFuture()

    def cancel_goal_async(self):
        self.cancels.append(FakeFuture())
        return self.cancels[-1]

    def get_result_async(self):
        return self.result_future


class FakeClient:
    def __init__(self, node, action_type, name, callback_group=None):
        self.sent, self.goals = [], []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        self.sent.append(FakeFuture())
        return self.sent[-1]

    def destroy(self):
        pass


class FakeNode:
    def __init__(self):
        self.logs = []
        log = self.logs.append
        self._logger = SimpleNamespace(warn=log, info=log, error=log, debug=log, warning=log)

    def get_logger(self):
        return self._logger

    def get_clock(self):
        from builtin_interfaces.msg import Time
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))


def _patch_client(monkeypatch, module):
    for name in (module, "pode_fsm.action_client"):
        try:
            mod = importlib.import_module(name)
        except ImportError:
            continue
        if hasattr(mod, "ActionClient"):
            monkeypatch.setattr(mod, "ActionClient", FakeClient)


# ---- 실행기 셋을 같은 사건 목록으로 맞추기 ----
class Rig:
    """kind별 실행기를 만들고 콜백을 ('rejected', seq), ('result', seq, outcome, lost),
    ('cancel_ok', seq)로 모은다."""

    def __init__(self, kind, monkeypatch):
        self.kind, self.node, self.events = kind, FakeNode(), []
        ev = self.events.append
        if kind == "follow":
            _patch_client(monkeypatch, "pode_fsm.follow_ros")
            from pode_fsm.follow_ros import FollowPathRunner
            self.runner = FollowPathRunner(
                self.node, "follow_path", "map", ("FollowPath", "goal", "progress"), "",
                ("FollowPathReverse", ""), "", lambda s: ev(("rejected", s)),
                lambda s, o, c, lost: ev(("result", s, o, lost)),
                lambda s: ev(("cancel_ok", s)))
        elif kind == "nav_goal":
            _patch_client(monkeypatch, "pode_fsm.nav_goal_ros")
            from pode_fsm.nav_goal_ros import NavGoalRunner
            self.runner = NavGoalRunner(
                self.node, "navigate_to_pose", "map", lambda s, d: None,
                lambda s, o, lost: ev(("rejected", s) if o == "rejected"
                                      else ("result", s, o, lost)),
                lambda s: ev(("cancel_ok", s)))
        else:
            _patch_client(monkeypatch, "pode_fsm.backup_ros")
            from pode_fsm.backup_ros import BackUpRunner
            self.runner = BackUpRunner(
                self.node, "backup",
                lambda s, o, c, t: ev(("rejected", s) if o == "rejected"
                                      else ("result", s, o, False)))
        self.client = self.runner._client

    def send(self, seq):
        if self.kind == "follow":
            assert self.runner.send(seq, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], True, False)
        elif self.kind == "nav_goal":
            assert self.runner.send(seq, (1.0, 0.0, 0.0))
        else:
            assert self.runner.send(seq, 0.1, 0.05, 6.0)
        return self.client.sent[-1]

    def accepted(self, seq):
        handle = FakeHandle()
        self.send(seq).resolve(handle)
        return handle


KINDS = ("follow", "nav_goal", "backup")


def _status(status, code=0):
    from action_msgs.msg import GoalStatus
    names = {"succeeded": GoalStatus.STATUS_SUCCEEDED, "aborted": GoalStatus.STATUS_ABORTED,
             "canceled": GoalStatus.STATUS_CANCELED}
    return SimpleNamespace(status=names[status],
                           result=SimpleNamespace(error_code=code, error_msg=""))


def _cancel_yes():
    from action_msgs.msg import GoalInfo
    from action_msgs.srv import CancelGoal
    return CancelGoal.Response(goals_canceling=[GoalInfo()])


def _cancel_code(code):
    from action_msgs.srv import CancelGoal
    return CancelGoal.Response(return_code=getattr(CancelGoal.Response, code))


# ---- goal 거절과 보내기 실패 ----
@pytest.mark.parametrize("kind", KINDS)
def test_a_rejected_goal_reports_rejected(kind, monkeypatch):
    rig = Rig(kind, monkeypatch)
    rig.send(3).resolve(FakeHandle(accepted=False))
    assert rig.events == [("rejected", 3)]


@pytest.mark.parametrize("kind", KINDS)
def test_a_goal_whose_send_future_fails_reports_rejected(kind, monkeypatch):
    """BackUp은 future.result() 예외로 콜백이 죽어 FSM이 시간 초과까지 아무것도 못 받았다."""
    rig = Rig(kind, monkeypatch)
    rig.send(3).fail(RuntimeError("lost"))
    assert rig.events == [("rejected", 3)] and rig.node.logs


@pytest.mark.parametrize("kind", KINDS)
def test_a_goal_rejected_after_a_cancel_request_reports_rejected_once(kind, monkeypatch):
    rig = Rig(kind, monkeypatch)
    fut = rig.send(3)
    rig.runner.cancel_all()
    handle = FakeHandle(accepted=False)
    fut.resolve(handle)
    assert rig.events == [("rejected", 3)] and handle.cancels == []


# ---- 수락 전 취소 요청 뒤 늦은 수락 ----
@pytest.mark.parametrize("kind", KINDS)
def test_a_goal_accepted_after_cancel_all_is_cancelled_at_once(kind, monkeypatch):
    rig = Rig(kind, monkeypatch)
    fut = rig.send(3)
    rig.runner.cancel_all()
    handle = FakeHandle()
    fut.resolve(handle)
    assert len(handle.cancels) == 1
    handle.result_future.resolve(_status("canceled"))
    assert rig.events[-1] == ("result", 3, "canceled", False)


# ---- 취소 응답: 수락, 거절, 유실 ----
@pytest.mark.parametrize("kind", KINDS)
def test_an_accepted_cancel_is_the_only_cancel_the_fsm_hears_of(kind, monkeypatch):
    """FollowPath, NavigateToPose는 취소 수락을 알린다. BackUp은 계약에 그 콜백이 없고
    결과(또는 FSM 시간 초과)만 본다."""
    rig = Rig(kind, monkeypatch)
    handle = rig.accepted(3)
    rig.runner.cancel_all()
    handle.cancels[-1].resolve(_cancel_yes())
    assert rig.events == ([] if kind == "backup" else [("cancel_ok", 3)])
    handle.result_future.resolve(_status("canceled"))
    assert rig.events[-1] == ("result", 3, "canceled", False)


@pytest.mark.parametrize("kind", KINDS)
@pytest.mark.parametrize("answer", ("refused", "error", "empty"))
def test_a_cancel_that_is_not_confirmed_keeps_the_goal_and_warns(kind, answer, monkeypatch):
    """BackUp은 취소 응답을 보지 않아 거절이나 유실이 로그에도 남지 않았다."""
    rig = Rig(kind, monkeypatch)
    handle = rig.accepted(3)
    rig.runner.cancel_all()
    fut = handle.cancels[-1]
    if answer == "refused":
        fut.resolve(_cancel_code("ERROR_REJECTED"))
    elif answer == "error":
        fut.fail(RuntimeError("no answer"))
    else:
        fut.resolve(None)
    assert rig.events == [] and rig.node.logs
    rig.runner.cancel_all()
    assert len(handle.cancels) == 2, "취소되지 않은 goal은 다음 취소 때 다시 취소한다"


@pytest.mark.parametrize("kind", KINDS)
def test_a_cancel_on_a_finished_goal_is_confirmed(kind, monkeypatch):
    rig = Rig(kind, monkeypatch)
    handle = rig.accepted(3)
    rig.runner.cancel_all()
    handle.cancels[-1].resolve(_cancel_code("ERROR_GOAL_TERMINATED"))
    assert rig.events == ([] if kind == "backup" else [("cancel_ok", 3)])
    rig.runner.cancel_all()
    assert len(handle.cancels) == 1


# ---- 결과 future 예외 ----
@pytest.mark.parametrize("kind", ("follow", "nav_goal"))
def test_a_lost_result_is_reported_lost_and_the_goal_is_cancelled(kind, monkeypatch):
    rig = Rig(kind, monkeypatch)
    handle = rig.accepted(3)
    handle.result_future.fail(RuntimeError("lost"))
    assert rig.events == [("result", 3, "aborted", True)] and len(handle.cancels) == 1


@pytest.mark.parametrize("kind", KINDS)
def test_a_result_future_with_no_result_counts_as_lost(kind, monkeypatch):
    """취소된 rclpy future는 result()가 None이다."""
    rig = Rig(kind, monkeypatch)
    handle = rig.accepted(3)
    handle.result_future.resolve(None)
    assert len(handle.cancels) == 1 and rig.node.logs


def test_a_lost_backup_result_is_not_reported_while_the_cancel_is_only_in_progress(monkeypatch):
    """goals_canceling은 취소 중이지 종료가 아니다. 그때 aborted를 알리면
    core가 _backup_unanswered를 지워 후진이 도는 중에 다음 동작이 나갈 수 있다."""
    rig = Rig("backup", monkeypatch)
    handle = rig.accepted(3)
    handle.result_future.fail(RuntimeError("lost"))
    assert rig.events == [] and len(handle.cancels) == 1
    handle.cancels[-1].resolve(_cancel_yes())
    assert rig.events == [], "종료를 확인할 수 없으면 FSM 시간 초과에 맡긴다"
    rig.runner.cancel_all()
    assert len(handle.cancels) == 2, "취소 요청은 계속 보낸다"


@pytest.mark.parametrize("code", ("ERROR_GOAL_TERMINATED", "ERROR_UNKNOWN_GOAL_ID"))
def test_a_lost_backup_result_is_reported_once_the_goal_is_known_to_have_ended(code, monkeypatch):
    rig = Rig("backup", monkeypatch)
    handle = rig.accepted(3)
    handle.result_future.fail(RuntimeError("lost"))
    handle.cancels[-1].resolve(_cancel_yes())
    rig.runner.cancel_all()
    handle.cancels[-1].resolve(_cancel_code(code))
    assert rig.events == [("result", 3, "aborted", False)]
    handle.cancels[0].resolve(_cancel_code(code))
    assert len(rig.events) == 1, "한 번만 알린다"


def test_a_lost_backup_result_with_a_refused_cancel_is_left_to_the_fsm_timeout(monkeypatch):
    """취소가 거절되면 후진은 아직 돈다. 알리지 않고 핸들을 남긴다(FSM 시간 초과가 끝낸다)."""
    rig = Rig("backup", monkeypatch)
    handle = rig.accepted(3)
    handle.result_future.fail(RuntimeError("lost"))
    handle.cancels[-1].resolve(_cancel_code("ERROR_REJECTED"))
    assert rig.events == []
    rig.runner.cancel_all()
    handle.cancels[-1].resolve(_cancel_code("ERROR_GOAL_TERMINATED"))
    assert rig.events == [("result", 3, "aborted", False)], "다음 취소가 끝을 확인하면 알린다"


def test_a_backup_cancel_in_progress_keeps_the_handle(monkeypatch):
    """결과를 잃기 전에 취소 중 응답을 받아도 핸들을 지우지 않는다.
    지우면 결과를 잃었을 때 이미 끝났다고 보고 바로 aborted를 알린다."""
    rig = Rig("backup", monkeypatch)
    handle = rig.accepted(3)
    rig.runner.cancel_all()
    handle.cancels[-1].resolve(_cancel_yes())
    handle.result_future.fail(RuntimeError("lost"))
    assert rig.events == [] and len(handle.cancels) == 2


def test_a_lost_backup_result_after_the_goal_was_known_to_have_ended_is_reported_at_once(
        monkeypatch):
    rig = Rig("backup", monkeypatch)
    handle = rig.accepted(3)
    rig.runner.cancel_all()
    handle.cancels[-1].resolve(_cancel_code("ERROR_GOAL_TERMINATED"))
    handle.result_future.fail(RuntimeError("lost"))
    assert rig.events == [("result", 3, "aborted", False)] and len(handle.cancels) == 1


# ---- future를 돌려주기 전에 나는 예외 ----
def _raise(*_a, **_k):
    raise RuntimeError("client gone")


@pytest.mark.parametrize("kind", KINDS)
def test_a_send_call_that_raises_returns_false(kind, monkeypatch):
    """send()가 False면 노드는 다음 tick에 거절로 넘긴다
    (서버가 없을 때와 같은 길)."""
    rig = Rig(kind, monkeypatch)
    rig.client.send_goal_async = _raise
    args = {"follow": ([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], True, False),
            "nav_goal": ((1.0, 0.0, 0.0),), "backup": (0.1, 0.05, 6.0)}[kind]
    assert rig.runner.send(3, *args) is False
    assert rig.events == [] and rig.node.logs and not rig.runner._pending


@pytest.mark.parametrize("kind", KINDS)
def test_a_result_request_that_raises_counts_as_a_lost_result(kind, monkeypatch):
    rig = Rig(kind, monkeypatch)
    handle = FakeHandle()
    handle.get_result_async = _raise
    rig.send(3).resolve(handle)
    assert len(handle.cancels) == 1 and rig.node.logs
    if kind == "backup":
        assert rig.events == []
        handle.cancels[-1].resolve(_cancel_code("ERROR_GOAL_TERMINATED"))
        assert rig.events == [("result", 3, "aborted", False)]
    else:
        assert rig.events == [("result", 3, "aborted", True)]


@pytest.mark.parametrize("kind", KINDS)
def test_a_cancel_call_that_raises_warns_and_keeps_the_goal(kind, monkeypatch):
    rig = Rig(kind, monkeypatch)
    handle = rig.accepted(3)
    calls = []

    def boom():
        calls.append(1)
        raise RuntimeError("client gone")
    handle.cancel_goal_async = boom
    rig.runner.cancel_all()
    assert rig.events == [] and rig.node.logs
    rig.runner.cancel_all()
    assert len(calls) == 2, "핸들을 남겨 다음 취소 때 다시 취소한다"


@pytest.mark.parametrize("kind", KINDS)
def test_a_lost_result_whose_cancel_call_raises_is_still_reported(kind, monkeypatch):
    """결과를 잃은 뒤 취소 호출이 예외를 내도 FollowPath, NavigateToPose는 lost를 알린다.
    BackUp은 끝을 확인할 수 없으니 알리지 않는다."""
    rig = Rig(kind, monkeypatch)
    handle = rig.accepted(3)
    handle.cancel_goal_async = _raise
    handle.result_future.fail(RuntimeError("lost"))
    assert rig.events == ([] if kind == "backup" else [("result", 3, "aborted", True)])


def test_a_backup_result_carries_the_travelled_distance(monkeypatch):
    rig = Rig("backup", monkeypatch)
    seen = []
    rig.runner._on_result = lambda *a: seen.append(a)
    fut = rig.send(3)
    handle = FakeHandle()
    fut.resolve(handle)
    rig.runner._feedback(3, SimpleNamespace(feedback=SimpleNamespace(distance_traveled=0.04)))
    handle.result_future.resolve(_status("aborted", 714))
    assert seen == [(3, "aborted", 714, pytest.approx(0.04))]


@pytest.mark.parametrize("free_id, final, reverse, free, want", [
    ("free", True, False, False, ("FollowPath", "goal")),
    ("free", True, False, True, ("FollowPath", "free")),
    ("free", False, False, True, ("Through", "free")),
    ("free", True, True, True, ("Reverse", "back")),   # 후진은 자기 checker가 먼저다
    ("", True, False, True, ("FollowPath", "goal")),    # 이름을 비우면 전처럼 끝 방향을 맞춘다
])
def test_the_follow_runner_names_the_goal_checker_of_each_kind_of_path(
        free_id, final, reverse, free, want, monkeypatch):
    _patch_client(monkeypatch, "pode_fsm.follow_ros")
    from pode_fsm.follow_ros import FollowPathRunner
    runner = FollowPathRunner(
        FakeNode(), "follow_path", "map", ("FollowPath", "goal", "progress"), "Through",
        ("Reverse", "back"), free_id, lambda s: None, lambda s, o, c, lost: None, lambda s: None)
    assert runner.send(1, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], final, reverse, free)
    goal = runner._client.goals[-1]
    assert (goal.controller_id, goal.goal_checker_id) == want
