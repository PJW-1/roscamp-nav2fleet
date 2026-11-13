# ---- Nav2 FollowPath 액션으로 경로를 보내고 결과를 돌려주는 실행기 ----
from __future__ import annotations

import threading
from collections.abc import Callable
from functools import partial

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient

from pinky_fsm.geometry import yaw_quaternion

# ---- 액션 상태 이름과 콜백 형식 ----
_OUTCOME = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled"}
_GONE = (CancelGoal.Response.ERROR_GOAL_TERMINATED, CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID)

OnRejected = Callable[[int], None]
OnResult = Callable[[int, str, int, bool], None]
OnCancelAccepted = Callable[[int], None]


# ---- FollowPath 액션 클라이언트 생성 ----
class FollowPathRunner:
    def __init__(self, node, action_name: str, frame: str, nav_ids: tuple[str, str, str],
                 through_id: str, reverse_id: tuple[str, str], on_rejected: OnRejected,
                 on_result: OnResult, on_cancel_accepted: OnCancelAccepted,
                 callback_group=None):
        self._node = node
        self._frame = frame
        self._nav_ids = nav_ids
        self._through_id = through_id
        self._reverse_id = reverse_id
        self._client = ActionClient(node, FollowPath, action_name,
                                    callback_group=callback_group)
        self._on_rejected, self._on_result = on_rejected, on_result
        self._on_cancel_accepted = on_cancel_accepted
        self._lock = threading.Lock()
        self._handles: dict[int, object] = {}
        self._pending: set[int] = set()
        self._cancel_on_accept: set[int] = set()

    # ---- Path 메시지를 만들고 컨트롤러를 골라 goal 보내기 ----
    def send(self, seq: int, poses, final_goal: bool, reverse: bool) -> bool:
        if not self._client.server_is_ready():
            return False
        path = Path()
        path.header.frame_id, path.header.stamp = self._frame, self._node.get_clock().now().to_msg()
        for x, y, yaw in poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            qx, qy, qz, qw = yaw_quaternion(yaw)
            ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
            path.poses.append(ps)
        controller, checker, progress = self._nav_ids
        if self._through_id and not final_goal:
            controller = self._through_id
        if reverse:
            controller, checker = self._reverse_id[0], self._reverse_id[1] or checker
        goal = FollowPath.Goal(path=path, controller_id=controller, goal_checker_id=checker,
                               progress_checker_id=progress)
        with self._lock:
            self._pending.add(seq)
        self._client.send_goal_async(goal).add_done_callback(partial(self._on_response, seq))
        return True

    # ---- goal 취소와 취소 응답 처리 ----
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

    def _on_cancel(self, seq: int, future) -> None:
        try:
            resp = future.result()
            accepted = bool(resp.goals_canceling) or resp.return_code in _GONE
        except Exception as exc:  # noqa: BLE001 - 답이 없으면 계속 돌 수 있다고 본다
            self._node.get_logger().warn(f"follow_path #{seq} cancel got no answer: {exc}")
            return
        if not accepted:
            self._node.get_logger().warn(f"follow_path #{seq} cancel was refused")
            return
        with self._lock:
            self._handles.pop(seq, None)
        self._on_cancel_accepted(seq)

    # ---- goal 수락 응답 처리 ----
    def _on_response(self, seq: int, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - 콜백이 죽으면 이 FollowPath는 결과가 없다
            self._node.get_logger().warn(f"follow_path #{seq} was not sent: {exc}")
            handle = None
        with self._lock:
            self._pending.discard(seq)
            accepted = handle is not None and handle.accepted
            cancel = accepted and seq in self._cancel_on_accept
            self._cancel_on_accept.discard(seq)
            if accepted:
                self._handles[seq] = handle
        if not accepted:
            self._on_rejected(seq)
            return
        if cancel:
            self._cancel(seq, handle)
        handle.get_result_async().add_done_callback(partial(self._on_done, seq))

    # ---- goal 결과 처리 ----
    def _on_done(self, seq: int, future) -> None:
        try:
            res = future.result()
            outcome, code = _OUTCOME.get(res.status, "aborted"), int(res.result.error_code)
            if outcome != "succeeded" and code:
                self._node.get_logger().info(f"follow_path #{seq} {outcome}: "
                                             f"{code} {res.result.error_msg}")
        except Exception as exc:  # noqa: BLE001 - 결과를 잃었다. goal은 아직 돌 수 있다
            self._node.get_logger().warn(f"follow_path #{seq} lost its result: {exc}")
            with self._lock:
                handle = self._handles.get(seq)
            if handle is not None:
                self._cancel(seq, handle)
            self._on_result(seq, "aborted", 0, True)
            return
        with self._lock:
            self._handles.pop(seq, None)
        self._on_result(seq, outcome, code, False)
