"""PinkyFsm이 지금 하는 일의 기록(_active, _recovery). core에서도 내보낸다."""
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
    s: float = 0.0  # 로봇이 선 위에서 간 거리
    s_start: float = 0.0  # 목표를 받은 자리(선 위 거리). 지나온 길은 s_start부터 s까지다
    final_goal: bool = True  # 끝이 임무의 마지막 지점인가(계약 0.8-draft 제안)
    seq: int = 0  # 마지막으로 보낸 FollowPath 순번
    nav_running: bool = False
    last_send: float = 0.0
    far_successes: int = 0
    waiting_since: float | None = None  # when Nav2 stopped for an obstacle; resend when clear
    blocked_since: float | None = None  # when this goal became blocked (block_timeout_s)
    # 앞 NavigateToPose가 끝나기를 기다린 때부터(아직 FollowPath를 보내지 않았다). None이면 간다
    handover_since: float | None = None


@dataclass
class _Recovery:
    """자가 복구 한 번. 시도마다 막힌 곳(stuck_s, 선 위 거리)을 새로 잡는다."""
    stuck_s: float
    backup_seq: int | None = None  # 물러나는 중이면 그 BackUp 순번
    backup_since: float = 0.0
    follow_since: float = 0.0  # 남은 선을 다시 보낸 시각
    # 로봇단 교착 처리(추가 지시서 3의 7-4)면 참. 자가 복구와 같은 기록에 기다림 단계를 더한다
    local: bool = False
    wait_until: float | None = None  # 기다리는 중이면 기다림이 끝나는 시각
    stood_since: float = 0.0  # 선 시각. 이때부터 blocked_after_s 안에 BLOCKED가 된다


@dataclass
class _Dock:
    goal_id: Hashable
    dock_id: str
