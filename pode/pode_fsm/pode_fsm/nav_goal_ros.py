# ---- Nav2 NavigateToPose 액션을 보내고 결과를 돌려주는 실행기 ----
from __future__ import annotations

from collections.abc import Callable
from functools import partial

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from pode_fsm.action_client import GoalRunner
from pode_fsm.geometry import yaw_quaternion

# ---- 콜백 형식 ----
OnFeedback = Callable[[int, float], None]
OnResult = Callable[[int, str, bool], None]
OnCancelAccepted = Callable[[int], None]


# ---- NavigateToPose 액션 클라이언트 생성 ----
class NavGoalRunner(GoalRunner):
    _label = "navigate_to_pose"

    def __init__(self, node, action_name: str, frame: str, on_feedback: OnFeedback,
                 on_result: OnResult, on_cancel_accepted: OnCancelAccepted,
                 callback_group=None):
        super().__init__(node, NavigateToPose, action_name, callback_group)
        self._frame = frame
        self._on_feedback, self._on_result = on_feedback, on_result
        self._on_cancel_accepted = on_cancel_accepted

    # ---- 목표 자세 goal 보내기 ----
    def send(self, seq: int, pose) -> bool:
        if not self._client.server_is_ready():
            return False
        target = PoseStamped()
        target.header.frame_id = self._frame
        target.header.stamp = self._node.get_clock().now().to_msg()
        x, y, yaw = (float(v) for v in pose)
        target.pose.position.x, target.pose.position.y = x, y
        _, _, qz, qw = yaw_quaternion(yaw)
        target.pose.orientation.z, target.pose.orientation.w = qz, qw
        return self._send_goal(seq, NavigateToPose.Goal(pose=target),
                               partial(self._feedback, seq))

    # ---- 남은 거리 피드백 ----
    def _feedback(self, seq: int, msg) -> None:
        self._on_feedback(seq, float(msg.feedback.distance_remaining))

    # ---- 공통 실행기 알림을 FSM 콜백으로 ----
    def _rejected(self, seq: int) -> None:
        self._on_result(seq, "rejected", False)

    def _finished(self, seq: int, outcome: str, code: int) -> None:
        self._on_result(seq, outcome, False)

    def _lost(self, seq: int) -> None:
        self._on_result(seq, "aborted", True)

    def _cancel_accepted(self, seq: int) -> None:
        self._on_cancel_accepted(seq)
