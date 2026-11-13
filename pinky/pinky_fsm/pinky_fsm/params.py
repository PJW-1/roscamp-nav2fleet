# ---- 상태 기계 설정값 정의 ----
from __future__ import annotations

from dataclasses import dataclass

COMM_LOSS_POLICIES = ("stop", "base_end")


# ---- 주행, 막힘, 후진, 복구, 양보 설정 기본값 ----
@dataclass(frozen=True)
class FsmParams:
    arrival_tolerance_m: float = 0.15
    connect_tolerance_m: float = 0.15
    refresh_within_m: float = 0.15
    refresh_every_s: float = 0.5
    heartbeat_timeout_s: float = 1.5
    comm_loss_policy: str = "stop"
    stopped_speed_mps: float = 0.02
    stopped_turn_rps: float = 0.1
    block_report_s: float = 1.0
    block_clear_s: float = 1.0
    block_lookahead_m: float = 0.5
    block_timeout_s: float = 90.0
    block_sample_m: float = 0.02
    path_step_m: float = 0.05
    turn_radius_m: float = 0.10
    join_min_off_m: float = 0.02
    join_slope: float = 3.0
    far_success_limit: int = 3
    track_back_m: float = 0.2
    track_ahead_m: float = 1.0
    recovery_attempts: int = 2
    backup_max_m: float = 0.15
    release_distance_m: float = 0.2
    backup_margin_m: float = 0.03
    backup_min_m: float = 0.02
    backup_lateral_limit_m: float = 0.03
    backup_speed_mps: float = 0.08
    backup_time_allowance_s: float = 5.0
    retreat_stop_wait_s: float = 3.0
    reverse_speed_mps: float = 0.05
    backup_result_grace_s: float = 2.0
    nav_result_grace_s: float = 2.0
    recovery_pass_m: float = 0.05
    recovery_timeout_s: float = 10.0
    recovery_reset_m: float = 0.3
    horizon_decel_mps2: float = 0.25
    horizon_min_mps: float = 0.05
    horizon_cap_mps: float = 0.25
    horizon_latency_s: float = 0.3
    blocked_after_s: float = 10.0
    nav_progress_m: float = 0.05
    local_yield: bool = True
    local_wait_s: float = 2.0
    local_backup_m: float = 0.10
    local_peer_near_m: float = 0.20
    peers_timeout_s: float = 3.0

    # ---- 통신 끊김 정책 값 검사 ----
    def __post_init__(self):
        if self.comm_loss_policy not in COMM_LOSS_POLICIES:
            raise ValueError(f"comm_loss_policy must be one of {COMM_LOSS_POLICIES}, "
                             f"got {self.comm_loss_policy!r}")
