# ---- Nav2 FollowPath 액션으로 경로를 보내고 결과를 돌려주는 실행기 ----
from __future__ import annotations

from collections.abc import Callable

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path

from pode_fsm.action_client import GoalRunner
from pode_fsm.geometry import yaw_quaternion

# ---- 콜백 형식 ----
OnRejected = Callable[[int], None]
OnResult = Callable[[int, str, int, bool], None]
OnCancelAccepted = Callable[[int], None]


# ---- FollowPath 액션 클라이언트 생성 ----
class FollowPathRunner(GoalRunner):
    _label = "follow_path"

    def __init__(self, node, action_name: str, frame: str, nav_ids: tuple[str, str, str],
                 through_id: str, reverse_id: tuple[str, str], free_id: str,
                 on_rejected: OnRejected,
                 on_result: OnResult, on_cancel_accepted: OnCancelAccepted,
                 callback_group=None):
        super().__init__(node, FollowPath, action_name, callback_group)
        self._frame = frame
        self._nav_ids = nav_ids
        self._through_id = through_id
        self._free_id = free_id
        self._reverse_id = reverse_id
        self._on_rejected, self._on_result = on_rejected, on_result
        self._on_cancel_accepted = on_cancel_accepted

    # ---- Path 메시지를 만들고 컨트롤러를 골라 goal 보내기 ----
    def send(self, seq: int, poses, final_goal: bool, reverse: bool,
             free_heading: bool = False) -> bool:
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
        if free_heading and self._free_id:
            checker = self._free_id
        if reverse:
            controller, checker = self._reverse_id[0], self._reverse_id[1] or checker
        goal = FollowPath.Goal(path=path, controller_id=controller, goal_checker_id=checker,
                               progress_checker_id=progress)
        return self._send_goal(seq, goal)

    # ---- 공통 실행기 알림을 FSM 콜백으로 ----
    def _rejected(self, seq: int) -> None:
        self._on_rejected(seq)

    def _finished(self, seq: int, outcome: str, code: int) -> None:
        self._on_result(seq, outcome, code, False)

    def _lost(self, seq: int) -> None:
        self._on_result(seq, "aborted", 0, True)

    def _cancel_accepted(self, seq: int) -> None:
        self._on_cancel_accepted(seq)
