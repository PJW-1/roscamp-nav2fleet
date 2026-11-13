# ---- 상태 이름, 결과 사유, goal 종료(Finish) 자료형 ----
from __future__ import annotations

from collections.abc import Hashable
from dataclasses import dataclass

# ---- 로봇 상태 이름 ----
IDLE = "IDLE"
MOVING = "MOVING"
DOCKING = "DOCKING"
DOCKED = "DOCKED"
BLOCKED = "BLOCKED"
LOCALIZING = "LOCALIZING"
RECOVERING = "RECOVERING"

# ---- goal 실패 사유 이름 ----
NAV_ABORTED = "NAV_ABORTED"
CANCELED = "CANCELED"
TIMEOUT = "TIMEOUT"
HEARTBEAT_LOST = "HEARTBEAT_LOST"
PATH_NOT_CONNECTED = "PATH_NOT_CONNECTED"
SUPERSEDED = "SUPERSEDED"
ORDER_UPDATE_REJECTED = "ORDER_UPDATE_REJECTED"
HORIZON_INVALID = "HORIZON_INVALID"
DOCK_UNAVAILABLE = "DOCK_UNAVAILABLE"


# ---- goal 종료 알림 ----
@dataclass(frozen=True)
class Finish:
    goal_id: Hashable
    success: bool
    reason: str
    last_done_seq_id: int = 0
