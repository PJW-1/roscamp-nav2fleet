from __future__ import annotations

from collections.abc import Hashable
from dataclasses import dataclass

IDLE = "IDLE"
MOVING = "MOVING"
DOCKING = "DOCKING"
DOCKED = "DOCKED"
BLOCKED = "BLOCKED"
LOCALIZING = "LOCALIZING"
RECOVERING = "RECOVERING"

NAV_ABORTED = "NAV_ABORTED"
CANCELED = "CANCELED"
TIMEOUT = "TIMEOUT"
HEARTBEAT_LOST = "HEARTBEAT_LOST"
PATH_NOT_CONNECTED = "PATH_NOT_CONNECTED"
SUPERSEDED = "SUPERSEDED"
ORDER_UPDATE_REJECTED = "ORDER_UPDATE_REJECTED"
HORIZON_INVALID = "HORIZON_INVALID"
DOCK_UNAVAILABLE = "DOCK_UNAVAILABLE"


@dataclass(frozen=True)
class Finish:
    goal_id: Hashable
    success: bool
    reason: str
    last_done_seq_id: int = 0
