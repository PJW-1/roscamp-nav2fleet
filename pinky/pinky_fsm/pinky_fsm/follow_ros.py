"""Nav2 controller_server의 follow_path(nav2_msgs/action/FollowPath) 액션 클라이언트.

core.SendPath를 FollowPath goal로 보내고 결과, 거절, 받아들여진 취소를 순번으로 돌려준다.
수락 전에 취소하면 수락되는 대로 취소하고, 결과를 잃으면(future 오류) goal은 아직 돌 수 있으므로
핸들을 두고 취소를 보낸다(nav_goal_ros.NavGoalRunner와 같은 규칙).

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
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient

from pinky_fsm.geometry import yaw_quaternion

_OUTCOME = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled"}
# 취소 답이 이렇다면 그 goal은 돌지 않는다
_GONE = (CancelGoal.Response.ERROR_GOAL_TERMINATED, CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID)

OnRejected = Callable[[int], None]  # (순번). 서버가 거절했거나 보내지 못했다
OnResult = Callable[[int, str, int, bool], None]  # (순번, outcome, error_code, 결과를 잃었나)
OnCancelAccepted = Callable[[int], None]


class FollowPathRunner:
    def __init__(self, node, action_name: str, frame: str, nav_ids: tuple[str, str, str],
                 through_id: str, reverse_id: tuple[str, str], on_rejected: OnRejected,
                 on_result: OnResult, on_cancel_accepted: OnCancelAccepted,
                 callback_group=None):
        self._node = node
        self._frame = frame
        self._nav_ids = nav_ids  # (controller, goal checker, progress checker)
        self._through_id = through_id
        self._reverse_id = reverse_id  # (후진 controller, 후진 goal checker. 비면 보통 checker)
        self._client = ActionClient(node, FollowPath, action_name,
                                    callback_group=callback_group)
        self._on_rejected, self._on_result = on_rejected, on_result
        self._on_cancel_accepted = on_cancel_accepted
        self._lock = threading.Lock()
        self._handles: dict[int, object] = {}  # 순번 → 수락된 FollowPath 목표
        self._pending: set[int] = set()  # 보냈지만 수락 여부를 아직 모른다
        self._cancel_on_accept: set[int] = set()

    def send(self, seq: int, poses, final_goal: bool, reverse: bool) -> bool:
        """poses = [(x, y, yaw)], map 틀. 보냈으면 True. 서버가 없으면 False이고 콜백도 없다."""
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
            # Nav2 switches controllers on a preempting goal (controller_server
            # updateGlobalPath), and the speed limit reaches every loaded controller
            controller = self._through_id
        if reverse:
            controller, checker = self._reverse_id[0], self._reverse_id[1] or checker
        goal = FollowPath.Goal(path=path, controller_id=controller, goal_checker_id=checker,
                               progress_checker_id=progress)
        with self._lock:
            self._pending.add(seq)
        self._client.send_goal_async(goal).add_done_callback(partial(self._on_response, seq))
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

    def _on_cancel(self, seq: int, future) -> None:
        """취소가 받아들여진 FollowPath만 core가 결과 없이도 잊을 수 있다(Codex 재검토 1).
        이미 끝났거나 서버가 모르는 goal도 돌지 않으므로 같다(재검토 2: 결과를 잃은 goal)."""
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
            self._handles.pop(seq, None)  # 다시 취소하지 않는다. 결과가 오면 그대로 받는다
        self._on_cancel_accepted(seq)

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
            self._cancel(seq, handle)  # 멈추는 동안 수락된 목표는 바로 취소한다
        handle.get_result_async().add_done_callback(partial(self._on_done, seq))

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
            if handle is not None:  # 취소가 받아들여질 때까지 core가 이 goal을 기다린다
                self._cancel(seq, handle)
            self._on_result(seq, "aborted", 0, True)
            return
        with self._lock:
            self._handles.pop(seq, None)
        self._on_result(seq, outcome, code, False)
