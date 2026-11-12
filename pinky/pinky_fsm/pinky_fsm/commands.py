"""핑키 FSM이 노드에 내는 명령(core.PinkyFsm이 돌려준다). ROS를 모른다. core에서도 내보낸다."""
from __future__ import annotations

from dataclasses import dataclass, field

from pinky_fsm.geometry import Pose


@dataclass(frozen=True)
class SendPath:
    """Nav2 FollowPath에 이 경로를 보낸다. 달리는 목표는 취소하지 않는다(안내 2.1)."""
    seq: int
    poses: list[Pose] = field(compare=False)
    # False when the path ends at a lock boundary, not at the mission end (MoveTo.final_goal).
    # The node may then use a controller without the approach slow-down
    final_goal: bool = True
    # 참이면 뒤로 따른다(Retreat.path). 노드는 후진 controller(reverse_controller_id)를 쓴다
    reverse: bool = False


@dataclass(frozen=True)
class CancelNav:
    """Nav2 FollowPath 목표를 취소한다. controller가 속도 0을 내고 선다."""


@dataclass(frozen=True)
class LimitSpeed:
    """Nav2 controller의 속도 제한(m/s). 0이면 제한이 없다(nav2_msgs/SpeedLimit)."""
    mps: float


@dataclass(frozen=True)
class ClearCostmap:
    """local costmap을 비운다. 다음 라이다 스캔이 지금 있는 물체를 다시 그린다."""
