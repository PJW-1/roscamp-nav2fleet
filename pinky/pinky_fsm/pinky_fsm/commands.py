from __future__ import annotations

from dataclasses import dataclass, field

from pinky_fsm.geometry import Pose


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
