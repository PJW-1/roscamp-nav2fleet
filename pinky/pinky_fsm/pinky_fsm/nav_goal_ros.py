"""Nav2 bt_navigator의 navigate_to_pose(nav2_msgs/action/NavigateToPose) 액션 클라이언트.

nav_goal.SendNavGoal을 Nav2 goal로 보내고 feedback의 distance_remaining, 결과, 받아들여진 취소를
순번으로 돌려준다(MoveTo mode nav_goal, 계약 0.11-draft 제안). node.py의 FollowPath 처리와 같은
규칙이다: 수락 전에 취소하면 수락되는 대로 취소하고, 결과를 잃으면(future 오류) goal은 아직 돌 수
있으므로 핸들을 두고 취소를 보낸다.

콜백은 이 클래스의 잠금을 쥐지 않은 채 부른다. 부르는 쪽은 자기 잠금을 쥐고 send나 cancel_all을
불러도 된다.
"""
from __future__ import annotations

import threading
from collections.abc import Callable
from functools import partial

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from pinky_fsm.geometry import yaw_quaternion

_OUTCOME = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled"}
# 취소 답이 이렇다면 그 goal은 돌지 않는다
_GONE = (CancelGoal.Response.ERROR_GOAL_TERMINATED, CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID)

OnFeedback = Callable[[int, float], None]  # (순번, distance_remaining m)
OnResult = Callable[[int, str, bool], None]  # (순번, outcome, 결과를 잃었나)
OnCancelAccepted = Callable[[int], None]


class NavGoalRunner:
    def __init__(self, node, action_name: str, frame: str, on_feedback: OnFeedback,
                 on_result: OnResult, on_cancel_accepted: OnCancelAccepted,
                 callback_group=None):
        self._node = node
        self._frame = frame
        self._client = ActionClient(node, NavigateToPose, action_name,
                                    callback_group=callback_group)
        self._on_feedback, self._on_result = on_feedback, on_result
        self._on_cancel_accepted = on_cancel_accepted
        self._lock = threading.Lock()
        self._handles: dict[int, object] = {}  # 순번 → 수락된 goal
        self._pending: set[int] = set()  # 보냈지만 수락 여부를 아직 모른다
        self._cancel_on_accept: set[int] = set()

    def send(self, seq: int, pose) -> bool:
        """pose = (x, y, yaw), map 틀. 보냈으면 True. 서버가 없으면 False이고 콜백도 없다."""
        if not self._client.server_is_ready():
            return False
        target = PoseStamped()
        target.header.frame_id = self._frame
        target.header.stamp = self._node.get_clock().now().to_msg()
        x, y, yaw = (float(v) for v in pose)
        target.pose.position.x, target.pose.position.y = x, y
        _, _, qz, qw = yaw_quaternion(yaw)
        target.pose.orientation.z, target.pose.orientation.w = qz, qw
        with self._lock:
            self._pending.add(seq)
        future = self._client.send_goal_async(NavigateToPose.Goal(pose=target),
                                              feedback_callback=partial(self._feedback, seq))
        future.add_done_callback(partial(self._on_response, seq))
        return True

    def cancel_all(self) -> None:
        with self._lock:
            handles = list(self._handles.items())
            self._cancel_on_accept |= self._pending
        for seq, handle in handles:
            self._cancel(seq, handle)

    def destroy(self) -> None:
        self._client.destroy()

    def _cancel(self, seq: int, handle) -> None:
        handle.cancel_goal_async().add_done_callback(partial(self._on_cancel, seq))

    def _feedback(self, seq: int, msg) -> None:
        self._on_feedback(seq, float(msg.feedback.distance_remaining))

    def _on_response(self, seq: int, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - 콜백이 죽으면 이 goal은 결과가 없다
            self._node.get_logger().warn(f"navigate_to_pose #{seq} was not sent: {exc}")
            handle = None
        with self._lock:
            self._pending.discard(seq)
            accepted = handle is not None and handle.accepted
            cancel = accepted and seq in self._cancel_on_accept
            self._cancel_on_accept.discard(seq)
            if accepted:
                self._handles[seq] = handle
        if not accepted:
            self._on_result(seq, "rejected", False)
            return
        if cancel:
            self._cancel(seq, handle)  # 멈추는 동안 수락된 goal은 바로 취소한다
        handle.get_result_async().add_done_callback(partial(self._on_done, seq))

    def _on_done(self, seq: int, future) -> None:
        try:
            res = future.result()
            outcome = _OUTCOME.get(res.status, "aborted")
            if outcome != "succeeded" and int(res.result.error_code):
                self._node.get_logger().info(f"navigate_to_pose #{seq} {outcome}: "
                                             f"{res.result.error_code} {res.result.error_msg}")
        except Exception as exc:  # noqa: BLE001 - 결과를 잃었다. goal은 아직 돌 수 있다
            self._node.get_logger().warn(f"navigate_to_pose #{seq} lost its result: {exc}")
            with self._lock:
                handle = self._handles.get(seq)
            if handle is not None:
                self._cancel(seq, handle)
            self._on_result(seq, "aborted", True)
            return
        with self._lock:
            self._handles.pop(seq, None)
        self._on_result(seq, outcome, False)

    def _on_cancel(self, seq: int, future) -> None:
        try:
            resp = future.result()
            accepted = bool(resp.goals_canceling) or resp.return_code in _GONE
        except Exception as exc:  # noqa: BLE001 - 답이 없으면 계속 돌 수 있다고 본다
            self._node.get_logger().warn(f"navigate_to_pose #{seq} cancel got no answer: {exc}")
            return
        if not accepted:
            self._node.get_logger().warn(f"navigate_to_pose #{seq} cancel was refused")
            return
        with self._lock:
            self._handles.pop(seq, None)  # 다시 취소하지 않는다. 결과가 오면 그대로 받는다
        self._on_cancel_accepted(seq)
