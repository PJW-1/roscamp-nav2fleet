# ---- Nav2 BackUp 액션을 보내고 결과를 돌려주는 실행기 ----
from __future__ import annotations

import threading
from collections.abc import Callable
from functools import partial

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav2_msgs.action import BackUp as BackUpAction
from rclpy.action import ActionClient

# ---- 액션 상태 이름과 결과 콜백 형식 ----
_OUTCOME = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled"}

OnResult = Callable[[int, str, int, float], None]


# ---- BackUp 액션 클라이언트 생성, goal 전송, 전체 취소 ----
class BackUpRunner:
    def __init__(self, node, action_name: str, on_result: OnResult, callback_group=None):
        self._client = ActionClient(node, BackUpAction, action_name,
                                    callback_group=callback_group)
        self._on_result = on_result
        self._lock = threading.Lock()
        self._handles: dict[int, object] = {}
        self._pending: set[int] = set()
        self._cancel_on_accept: set[int] = set()
        self._travelled: dict[int, float] = {}

    def send(self, seq: int, distance_m: float, speed_mps: float,
             time_allowance_s: float) -> bool:
        if not self._client.server_is_ready():
            return False
        sec = int(time_allowance_s)
        goal = BackUpAction.Goal(
            target=Point(x=float(distance_m)), speed=float(speed_mps),
            time_allowance=Duration(sec=sec, nanosec=int((time_allowance_s - sec) * 1e9)))
        with self._lock:
            self._pending.add(seq)
            self._travelled[seq] = 0.0
        future = self._client.send_goal_async(goal, feedback_callback=partial(self._feedback, seq))
        future.add_done_callback(partial(self._on_response, seq))
        return True

    def cancel_all(self) -> None:
        with self._lock:
            handles = list(self._handles.values())
            self._cancel_on_accept |= self._pending
        for handle in handles:
            handle.cancel_goal_async()

    def destroy(self) -> None:
        self._client.destroy()

    # ---- 이동 거리 피드백, 수락 응답, 결과 처리 ----
    def _feedback(self, seq: int, msg) -> None:
        with self._lock:
            self._travelled[seq] = float(msg.feedback.distance_traveled)

    def _on_response(self, seq: int, future) -> None:
        handle = future.result()
        with self._lock:
            self._pending.discard(seq)
            if not handle.accepted:
                travelled = self._travelled.pop(seq, 0.0)
            else:
                self._handles[seq] = handle
                cancel = seq in self._cancel_on_accept
                self._cancel_on_accept.discard(seq)
        if not handle.accepted:
            self._on_result(seq, "rejected", 0, travelled)
            return
        if cancel:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(partial(self._on_done, seq))

    def _on_done(self, seq: int, future) -> None:
        res = future.result()
        with self._lock:
            self._handles.pop(seq, None)
            travelled = self._travelled.pop(seq, 0.0)
        self._on_result(seq, _OUTCOME.get(res.status, "aborted"), int(res.result.error_code),
                        travelled)
