"""노드의 담기 전 결품 확인(W-40 · PI-28)을 그래프 없이 본다: 서버가 없으면 바로 보내고, 모자라면
OUT_OF_STOCK 결과로 끝내며, 3초 안에 답이 없으면 다시 보내지 않고 잡아 둔다(IR-40)."""
import time
from types import MethodType, SimpleNamespace
from unittest.mock import MagicMock

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces")

from fleet_core.common.commands import CancelCmd, FillCmd  # noqa: E402
from fleet_interfaces.msg import OrderItem  # noqa: E402
from fleet_manager.fleet_manager_node import FleetManagerNode  # noqa: E402
from fleet_manager.stock_check import LIMIT_S, StockChecks  # noqa: E402

FILL = FillCmd(goal_id=7, robot="pode1", station_id="WORK", order_id="T-1",
               items={"choco_bar": 2})
METHODS = ("_check_then_send", "_stock_arrived", "_stock_answered", "_stock_unanswered",
           "_step_stock_checks", "_drop_stock_future", "_drop_held_fill", "_fill_or_hold",
           "_release_fills")


class Future:
    def __init__(self):
        self._callbacks, self._done, self._result, self._error = [], False, None, None

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def done(self):
        return self._done

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result

    def finish(self, result=None, error=None):
        self._done, self._result, self._error = True, result, error
        for callback in self._callbacks:
            callback(self)


class Client:
    def __init__(self, ready=True):
        self.ready, self.futures, self.dropped, self.raises = ready, [], [], None

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        if self.raises:
            raise self.raises
        self.futures.append(Future())
        return self.futures[-1]

    def remove_pending_request(self, future):
        self.dropped.append(future)


def node_with(client, waiting=None):
    sent, results, events, given_up = [], [], [], set()
    waiting = [] if waiting is None else waiting
    core = SimpleNamespace(
        on_fill_result=lambda goal_id, success, placed, missing, reason="", **kw:
        results.append((goal_id, success, placed, missing, reason)),
        arm=SimpleNamespace(holds=lambda goal_id: goal_id not in given_up))
    node = SimpleNamespace(
        _stock_client=client, _stock=StockChecks(), _held_fills=[], core=core,
        site=SimpleNamespace(stations={"WORK": SimpleNamespace(server="workcell")}),
        _send_fill=lambda command: sent.append(command.goal_id),
        _dispatch=lambda command, now: events.append(command.code),
        _post=lambda handler, *args: handler(*args), _now=lambda: 0.0,
        get_logger=lambda: MagicMock(), _arms_waiting=lambda command: list(waiting),
        _mode_sync=SimpleNamespace(system_mode="TEST"))
    for name in METHODS:
        setattr(node, name, MethodType(getattr(FleetManagerNode, name), node))
    return node, sent, results, events, given_up


def answer(*items, accepted=True, reason=""):
    return SimpleNamespace(accepted=accepted, reason=reason,
                           available=[OrderItem(kind=k, count=n) for k, n in items])


def test_without_a_stock_server_the_fill_goes_at_once():
    client = Client(ready=False)
    node, sent, results, _, _ = node_with(client)
    node._check_then_send(FILL)
    assert sent == [7] and client.futures == [] and results == []


def test_enough_stock_sends_the_fill():
    client = Client()
    node, sent, results, _, _ = node_with(client)
    node._check_then_send(FILL)
    assert sent == []
    client.futures[0].finish(answer(("choco_bar", 2)))
    assert sent == [7] and results == []


def test_short_stock_ends_the_fill_as_out_of_stock_without_sending():
    client = Client()
    node, sent, results, events, _ = node_with(client)
    node._check_then_send(FILL)
    client.futures[0].finish(answer(("choco_bar", 1)))
    assert sent == []
    assert results == [(7, False, {}, {"choco_bar": 1}, "OUT_OF_STOCK")]
    assert events == ["OUT_OF_STOCK"]


def test_no_answer_in_time_holds_the_fill_and_asks_no_more():
    client = Client()
    node, sent, results, events, _ = node_with(client)
    node._check_then_send(FILL)
    node._step_stock_checks(1e9)
    assert client.dropped == [client.futures[0]]
    assert events == ["STOCK_CHECK_NO_ANSWER"] and sent == [] and results == []
    client.futures[0].finish(answer(("choco_bar", 2)))  # 한도 뒤 답은 버린다
    node._step_stock_checks(2e9)
    assert sent == [] and len(client.futures) == 1 and events == ["STOCK_CHECK_NO_ANSWER"]


def test_a_check_within_the_limit_is_kept():
    client = Client()
    node, _, _, events, _ = node_with(client)
    node._check_then_send(FILL)
    asked_at = node._stock._asking[7].asked_at
    node._step_stock_checks(asked_at + LIMIT_S)
    assert events == [] and client.dropped == []


def test_a_refused_check_holds_the_fill():
    client = Client()
    node, sent, results, events, _ = node_with(client)
    node._check_then_send(FILL)
    client.futures[0].finish(answer(accepted=False, reason="WEB_DOWN"))
    assert sent == [] and results == [] and events == ["STOCK_CHECK_NO_ANSWER"]
    assert 7 in node._stock


def test_a_failed_reply_holds_the_fill():
    client = Client()
    node, sent, _, events, _ = node_with(client)
    node._check_then_send(FILL)
    client.futures[0].finish(error=RuntimeError("context is shutting down"))
    assert sent == [] and events == ["STOCK_CHECK_NO_ANSWER"] and 7 in node._stock


def test_a_call_that_raises_holds_the_fill():
    client = Client()
    client.raises = RuntimeError("context is shutting down")
    node, sent, _, events, _ = node_with(client)
    node._check_then_send(FILL)
    assert sent == [] and events == ["STOCK_CHECK_NO_ANSWER"] and 7 in node._stock


def test_a_held_fill_that_is_canceled_ends_as_canceled():
    client = Client()
    node, sent, results, _, _ = node_with(client)
    node._check_then_send(FILL)
    node._step_stock_checks(1e9)
    assert node._drop_held_fill(CancelCmd("workcell", 7)) is True
    assert results == [(7, False, {}, {}, "CANCELED")] and 7 not in node._stock


def test_a_fill_canceled_while_asking_drops_the_request_and_ignores_the_answer():
    client = Client()
    node, sent, results, _, _ = node_with(client)
    node._check_then_send(FILL)
    assert node._drop_held_fill(CancelCmd("workcell", 7)) is True
    assert client.dropped == [client.futures[0]]
    client.futures[0].finish(answer(("choco_bar", 2)))
    assert sent == [] and results == [(7, False, {}, {}, "CANCELED")]


def test_a_cancel_for_another_server_leaves_the_check():
    client = Client()
    node, _, results, _, _ = node_with(client)
    node._check_then_send(FILL)
    assert node._drop_held_fill(CancelCmd("pode1", 7)) is False
    assert results == [] and 7 in node._stock


def test_an_answer_for_a_job_the_core_gave_up_sends_nothing():
    # 같은 inbox 묶음에서 팔 ERROR로 코어가 먼저 포기한 경우
    client = Client()
    node, sent, results, events, given_up = node_with(client)
    node._check_then_send(FILL)
    given_up.add(7)
    client.futures[0].finish(answer(("choco_bar", 2)))
    assert sent == [] and results == [] and events == [] and 7 not in node._stock


def test_an_answer_that_arrives_after_the_limit_holds_the_fill_before_the_tick_expires_it():
    client = Client()
    node, sent, results, events, _ = node_with(client)
    node._check_then_send(FILL)
    node._stock._asking[7].asked_at -= LIMIT_S + 1.0
    client.futures[0].finish(answer(("choco_bar", 2)))
    assert sent == [] and results == [] and events == ["STOCK_CHECK_NO_ANSWER"]
    assert 7 in node._stock


def test_an_answer_that_arrived_in_time_counts_even_if_handled_after_the_limit():
    client, queued = Client(), []
    node, sent, _, events, _ = node_with(client)
    node._post = lambda handler, *args: queued.append((handler, args))
    node._check_then_send(FILL)
    client.futures[0].finish(answer(("choco_bar", 2)))  # 3초 안에 와서 inbox에서 기다림
    node._step_stock_checks(time.monotonic() + LIMIT_S + 5.0)
    assert events == [] and sent == []
    for handler, args in queued:
        handler(*args)
    assert sent == [7]


def test_a_mode_change_during_the_check_holds_the_fill_until_the_arms_report():
    client, waiting = Client(), []
    node, sent, _, _, _ = node_with(client, waiting)
    node._check_then_send(FILL)
    waiting.append("arm_left")  # 확인 중 시스템 모드가 바뀌고 팔이 아직 새 모드를 보고하지 않음
    client.futures[0].finish(answer(("choco_bar", 2)))
    assert sent == [] and node._held_fills == [FILL] and 7 not in node._stock


def test_a_fill_held_for_the_mode_is_checked_again_and_sent_once_the_arms_report():
    client, waiting = Client(), []
    node, sent, _, _, _ = node_with(client, waiting)
    node._check_then_send(FILL)
    waiting.append("arm_left")
    client.futures[0].finish(answer(("choco_bar", 2)))
    waiting.clear()
    node._release_fills()
    assert len(client.futures) == 2 and node._held_fills == [] and sent == []
    client.futures[1].finish(answer(("choco_bar", 2)))
    assert sent == [7]


def test_a_fill_the_core_gave_up_on_is_dropped_without_a_result():
    client = Client()
    node, sent, results, _, given_up = node_with(client)
    node._check_then_send(FILL)
    given_up.add(7)
    node._step_stock_checks(0.0)
    assert client.dropped == [client.futures[0]] and 7 not in node._stock
    client.futures[0].finish(answer(("choco_bar", 2)))
    assert sent == [] and results == []
