"""핑키 FSM이 보고하는 상태와 결과 이름, 그리고 결과 명령 Finish. core와 nav_moves가 같이 쓴다
(core는 그대로 다시 내보낸다)."""
from __future__ import annotations

from collections.abc import Hashable
from dataclasses import dataclass

# RobotState.STATE_* 와 같은 값
IDLE = "IDLE"
MOVING = "MOVING"
DOCKING = "DOCKING"
DOCKED = "DOCKED"
BLOCKED = "BLOCKED"
LOCALIZING = "LOCALIZING"
RECOVERING = "RECOVERING"  # 2026-11-09 제안: 자가 복구 중

# MoveTo.Result.REASON_* 와 같은 값
NAV_ABORTED = "NAV_ABORTED"
CANCELED = "CANCELED"
TIMEOUT = "TIMEOUT"
HEARTBEAT_LOST = "HEARTBEAT_LOST"
PATH_NOT_CONNECTED = "PATH_NOT_CONNECTED"
DOCK_UNAVAILABLE = "DOCK_UNAVAILABLE"  # Dock 결과에는 reason이 없다. 로그에만 남는다


@dataclass(frozen=True)
class Finish:
    goal_id: Hashable
    success: bool
    reason: str
