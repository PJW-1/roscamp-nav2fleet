# ---- Nav2 액션 goal의 수락, 취소, 결과를 seq별로 다루는 공통 실행기 ----
from __future__ import annotations

import threading
from functools import partial

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from rclpy.action import ActionClient

# ---- 액션 상태 이름과 이미 끝난 goal의 취소 응답 ----
OUTCOME = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
           GoalStatus.STATUS_CANCELED: "canceled"}
GONE = (CancelGoal.Response.ERROR_GOAL_TERMINATED, CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID)


def cancel_confirmed(resp) -> bool:
    # 그 goal이 취소 중이거나 이미 없을 때만 참
    return bool(resp.goals_canceling) or resp.return_code in GONE


# ---- 수락 전 취소, 취소 응답 확인, 잃은 결과 처리 ----
class GoalRunner:
    """하위 클래스는 _rejected, _finished, _lost, _cancel_accepted로 FSM에 알린다."""

    _label = "action"

    def __init__(self, node, action_type, action_name: str, callback_group=None):
        self._node = node
        self._client = ActionClient(node, action_type, action_name,
                                    callback_group=callback_group)
        self._lock = threading.Lock()
        self._handles: dict[int, object] = {}
        self._pending: set[int] = set()
        self._cancel_on_accept: set[int] = set()

    def cancel_all(self) -> None:
        with self._lock:
            handles = list(self._handles.items())
            self._cancel_on_accept |= self._pending
        for seq, handle in handles:
            self._cancel(seq, handle)

    def destroy(self) -> None:
        self._client.destroy()

    # ---- 하위 클래스가 정하는 알림 ----
    def _rejected(self, seq: int) -> None:
        raise NotImplementedError

    def _finished(self, seq: int, outcome: str, code: int) -> None:
        raise NotImplementedError

    def _lost(self, seq: int) -> None:
        raise NotImplementedError

    def _cancel_accepted(self, seq: int) -> None:
        pass

    # ---- goal 보내기와 수락 응답 ----
    def _send_goal(self, seq: int, goal, feedback_callback=None) -> bool:
        # 거짓이면 노드가 다음 tick에 거절로 넘긴다(서버가 없을 때와 같은 길)
        with self._lock:
            self._pending.add(seq)
        try:
            if feedback_callback is None:
                future = self._client.send_goal_async(goal)
            else:
                future = self._client.send_goal_async(goal, feedback_callback=feedback_callback)
            future.add_done_callback(partial(self._on_response, seq))
        except Exception as exc:  # noqa: BLE001 - future가 없으면 응답도 오지 않는다
            self._node.get_logger().warn(f"{self._label} #{seq} could not be sent: {exc}")
            with self._lock:
                self._pending.discard(seq)
                self._cancel_on_accept.discard(seq)
            return False
        return True

    def _on_response(self, seq: int, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - 콜백이 죽으면 이 goal은 결과가 없다
            self._node.get_logger().warn(f"{self._label} #{seq} was not sent: {exc}")
            handle = None
        with self._lock:
            self._pending.discard(seq)
            accepted = handle is not None and handle.accepted
            cancel = accepted and seq in self._cancel_on_accept
            self._cancel_on_accept.discard(seq)
            if accepted:
                self._handles[seq] = handle
        if not accepted:
            self._rejected(seq)
            return
        if cancel:
            self._cancel(seq, handle)
        try:
            handle.get_result_async().add_done_callback(partial(self._on_done, seq))
        except Exception as exc:  # noqa: BLE001 - 결과를 받을 길이 없다. goal은 돌 수 있다
            self._node.get_logger().warn(f"{self._label} #{seq} result request failed: {exc}")
            self._result_lost(seq)

    # ---- 결과 처리 ----
    def _on_done(self, seq: int, future) -> None:
        try:
            res = future.result()
            outcome, code = OUTCOME.get(res.status, "aborted"), int(res.result.error_code)
            if outcome != "succeeded" and code:
                self._node.get_logger().info(f"{self._label} #{seq} {outcome}: "
                                             f"{code} {res.result.error_msg}")
        except Exception as exc:  # noqa: BLE001 - 결과를 잃었다. goal은 아직 돌 수 있다
            self._node.get_logger().warn(f"{self._label} #{seq} lost its result: {exc}")
            self._result_lost(seq)
            return
        with self._lock:
            self._handles.pop(seq, None)
        self._finished(seq, outcome, code)

    def _result_lost(self, seq: int) -> None:
        # 핸들은 두고 취소한다. 취소가 확인되기 전까지 다음 cancel_all이 다시 취소한다
        with self._lock:
            handle = self._handles.get(seq)
        if handle is not None:
            self._cancel(seq, handle)
        self._lost(seq)

    # ---- 취소 요청과 취소 응답 ----
    def _cancel(self, seq: int, handle) -> None:
        # 호출이 실패하면 핸들을 두어 다음 cancel_all이 다시 취소한다
        try:
            handle.cancel_goal_async().add_done_callback(partial(self._on_cancel, seq))
        except Exception as exc:  # noqa: BLE001 - 취소를 못 보냈다. goal은 돌 수 있다
            self._node.get_logger().warn(f"{self._label} #{seq} cancel could not be sent: {exc}")

    def _on_cancel(self, seq: int, future) -> None:
        try:
            resp = future.result()
            accepted, gone = cancel_confirmed(resp), resp.return_code in GONE
        except Exception as exc:  # noqa: BLE001 - 답이 없으면 계속 돌 수 있다고 본다
            self._node.get_logger().warn(f"{self._label} #{seq} cancel got no answer: {exc}")
            return
        if not accepted:
            self._node.get_logger().warn(f"{self._label} #{seq} cancel was refused")
            return
        self._cancel_confirmed(seq, gone)

    def _cancel_confirmed(self, seq: int, gone: bool) -> None:
        # gone은 goal이 이미 끝났다는 뜻. 아니면 취소 중일 뿐이다
        with self._lock:
            self._handles.pop(seq, None)
        self._cancel_accepted(seq)
