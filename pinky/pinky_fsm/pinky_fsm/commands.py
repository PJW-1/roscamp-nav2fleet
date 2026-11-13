# ---- 상태 기계가 노드에 내리는 주행 명령 자료형 ----
from __future__ import annotations

from dataclasses import dataclass, field

from pinky_fsm.geometry import Pose


# ---- 경로 전송, 주행 취소, 속도 제한, 코스트맵 정리 명령 ----
@dataclass(frozen=True)
class SendPath:
    seq: int
    poses: list[Pose] = field(compare=False)
    final_goal: bool = True
    reverse: bool = False


@dataclass(frozen=True)
class CancelNav:
    pass


@dataclass(frozen=True)
class LimitSpeed:
    mps: float


@dataclass(frozen=True)
class ClearCostmap:
    pass
