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
# order 모델(계약 0.11-draft 제안). SUPERSEDED는 실패가 아니다(update가 goal을 넘겨받았다)
SUPERSEDED = "SUPERSEDED"
ORDER_UPDATE_REJECTED = "ORDER_UPDATE_REJECTED"  # reason은 "ORDER_UPDATE_REJECTED: <이유>"
HORIZON_INVALID = "HORIZON_INVALID"  # feedback horizon_warning의 앞말. 결과가 아니다
DOCK_UNAVAILABLE = "DOCK_UNAVAILABLE"  # Dock 결과에는 reason이 없다. 로그에만 남는다


@dataclass(frozen=True)
class Finish:
    goal_id: Hashable
    success: bool
    reason: str
    last_done_seq_id: int = 0  # order 모델(계약 0.11-draft 제안). order가 아니면 0
