from __future__ import annotations

from collections.abc import Hashable
from dataclasses import dataclass

from pinky_fsm.geometry import Polyline, Pose


@dataclass
class _Move:
    goal_id: Hashable
    target_id: str
    target: Pose
    route: Polyline
    s: float = 0.0
    s_start: float = 0.0
    final_goal: bool = True
    seq: int = 0
    sent: Polyline | None = None
    sent_from: float = 0.0
    sent_lead: float = 0.0
    nav_running: bool = False
    last_send: float = 0.0
    far_successes: int = 0
    waiting_since: float | None = None
    blocked_since: float | None = None
    handover_since: float | None = None


@dataclass
class _Recovery:
    stuck_s: float
    backup_seq: int | None = None
    backup_since: float = 0.0
    follow_since: float = 0.0
    local: bool = False
    wait_until: float | None = None
    stood_since: float = 0.0
    backup_m: float = 0.0


@dataclass
class _Dock:
    goal_id: Hashable
    dock_id: str
