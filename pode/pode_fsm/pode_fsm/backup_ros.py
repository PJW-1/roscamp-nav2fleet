# ---- Nav2 BackUp 액션을 보내고 결과를 돌려주는 실행기 ----
from __future__ import annotations

from collections.abc import Callable
from functools import partial

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav2_msgs.action import BackUp as BackUpAction

from pode_fsm.action_client import GoalRunner

# ---- 결과 콜백 형식 ----
OnResult = Callable[[int, str, int, float], None]


# ---- BackUp 액션 클라이언트 생성, goal 전송 ----
class BackUpRunner(GoalRunner):
    """취소 수락 콜백이 없다. FSM은 결과나 자기 시간 초과로만 후진이 끝났다고 본다.
    핸들은 결과가 오거나 goal이 이미 끝났다는 취소 응답(GONE)이 올 때만 지운다."""

    _label = "backup"

    def __init__(self, node, action_name: str, on_result: OnResult, callback_group=None):
        super().__init__(node, BackUpAction, action_name, callback_group)
        self._on_result = on_result
        self._travelled: dict[int, float] = {}
        self._lost_seqs: set[int] = set()

    def send(self, seq: int, distance_m: float, speed_mps: float,
             time_allowance_s: float) -> bool:
        if not self._client.server_is_ready():
            return False
        sec = int(time_allowance_s)
        goal = BackUpAction.Goal(
            target=Point(x=float(distance_m)), speed=float(speed_mps),
            time_allowance=Duration(sec=sec, nanosec=int((time_allowance_s - sec) * 1e9)))
        with self._lock:
            self._travelled[seq] = 0.0
        if self._send_goal(seq, goal, partial(self._feedback, seq)):
            return True
        with self._lock:
            self._travelled.pop(seq, None)
        return False

    # ---- 이동 거리 피드백과 결과 알림 ----
    def _feedback(self, seq: int, msg) -> None:
        with self._lock:
            self._travelled[seq] = float(msg.feedback.distance_traveled)

    def _report(self, seq: int, outcome: str, code: int) -> None:
        with self._lock:
            travelled = self._travelled.pop(seq, 0.0)
            self._lost_seqs.discard(seq)
        self._on_result(seq, outcome, code, travelled)

    # ---- 공통 실행기 알림을 FSM 결과로 ----
    def _rejected(self, seq: int) -> None:
        self._report(seq, "rejected", 0)

    def _finished(self, seq: int, outcome: str, code: int) -> None:
        self._report(seq, outcome, code)

    def _result_lost(self, seq: int) -> None:
        # 후진이 도는 중에 aborted를 알리면 FSM이 다음 goal을 보낸다. 끝난 것이 확인된 뒤 알린다
        # 핸들이 없으면 이미 GONE 응답을 받은 goal이다
        with self._lock:
            handle = self._handles.get(seq)
            if handle is not None:
                self._lost_seqs.add(seq)
        if handle is None:
            self._report(seq, "aborted", 0)
            return
        self._cancel(seq, handle)

    def _cancel_confirmed(self, seq: int, gone: bool) -> None:
        # 취소 중(goals_canceling)은 끝이 아니다. 핸들을 두고 알리지 않는다(FSM 시간 초과)
        if not gone:
            return
        # 두 취소 응답이 함께 와도 한 번만 알린다
        with self._lock:
            self._handles.pop(seq, None)
            if seq not in self._lost_seqs:
                return
            self._lost_seqs.discard(seq)
            travelled = self._travelled.pop(seq, 0.0)
        self._on_result(seq, "aborted", 0, travelled)
