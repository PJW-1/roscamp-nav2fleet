"""가짜 Pode(ROS)의 위치 다시 찾기 서버: 선 로봇이면 약 2 s 뒤 성공한다.

가짜 Pode는 위치 추정이 늘 맞으므로 다시 찾아도 자세가 바뀌지 않는다. 움직이는 중이면 받자마자
BUSY로 끝낸다(계약). 취소하면 CANCELED. LOCALIZING 보고와 실패 주입은 없다(세계 흉내에 있다).
"""
from __future__ import annotations

import time

from fleet_interfaces.action import Relocalize
from rclpy.action import ActionServer, CancelResponse, GoalResponse

RELOCALIZE_S = 2.0
STEP_S = 0.05


def add_relocalize_server(node, contract, robot_id: str, group) -> ActionServer:
    """node(FakePodeNode)에 /{pode}/relocalize 서버를 단다."""

    def execute(handle):
        m = node._motion
        start = (m.x, m.y, m.yaw)
        if node._active is not None:
            handle.succeed()
            return Relocalize.Result(success=False, reason=Relocalize.Result.REASON_BUSY)
        until = time.monotonic() + RELOCALIZE_S
        while time.monotonic() < until:
            if handle.is_cancel_requested:
                handle.canceled()
                return _result(False, Relocalize.Result.REASON_CANCELED, start, m)
            if not node._running():
                handle.abort()
                return Relocalize.Result()
            time.sleep(STEP_S)
        handle.succeed()
        return _result(True, "", start, m, stage="NOMOTION")

    return ActionServer(
        node, Relocalize, contract.name("relocalize", pode=robot_id), execute_callback=execute,
        goal_callback=lambda _req: GoalResponse.ACCEPT,
        cancel_callback=lambda _gh: CancelResponse.ACCEPT, callback_group=group)


def _result(success: bool, reason: str, start, m, stage: str = "") -> Relocalize.Result:
    out = Relocalize.Result(success=success, reason=reason, stage=stage, pose_valid=True,
                            scan_match=1.0)
    out.start_pose.x, out.start_pose.y, out.start_pose.theta = start
    out.pose.x, out.pose.y, out.pose.theta = m.x, m.y, m.yaw
    return out
