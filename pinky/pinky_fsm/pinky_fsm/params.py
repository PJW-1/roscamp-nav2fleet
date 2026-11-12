"""핑키 FSM의 매개변수. 기본값과 출처를 한곳에 둔다. ROS를 모른다."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FsmParams:
    arrival_tolerance_m: float = 0.15  # R4, 관제 arrival_tolerance_m
    connect_tolerance_m: float = 0.15  # 안내 2.2, 관제 lane_error_warn_m
    refresh_within_m: float = 0.15  # R13
    refresh_every_s: float = 0.5  # R13
    heartbeat_timeout_s: float = 1.5  # R10. 0이면 멈추지 않는다
    stopped_speed_mps: float = 0.02  # 계약 4절
    stopped_turn_rps: float = 0.1  # 제자리 회전은 서 있는 것이 아니다
    block_report_s: float = 1.0  # 계약 4절: 1초 서 있으면
    block_clear_s: float = 1.0  # 계약 4절: 1초 안 보이면 끈다
    block_lookahead_m: float = 0.5  # 계약 4절: 경로 위 0.5 m 안
    # Proposed. A goal blocked this long ends with TIMEOUT. The fleet manager cancels a blocked
    # goal after blocked_mark_s (10 s); this is the last safeguard for when it does not
    block_timeout_s: float = 90.0
    block_sample_m: float = 0.02  # costmap 해상도(0.025)보다 촘촘하게
    path_step_m: float = 0.05  # 전달사항 6.1 poses_along
    far_success_limit: int = 3  # 목표에서 먼 Nav2 성공을 몇 번까지 다시 보낼까
    track_back_m: float = 0.2  # 진행 거리를 찾을 때 뒤로 허용하는 거리
    track_ahead_m: float = 1.0  # 진행 거리를 찾을 때 앞으로 보는 거리
    # 자가 복구(핑키_계약 3절 7번, 2026-11-09 제안). 숫자는 추정이다
    recovery_attempts: int = 2  # 두 번 실패하면 BLOCKED(물체 앞)나 NAV_ABORTED로 넘긴다
    backup_max_m: float = 0.15
    # 관제 release_distance_m. 꼭짓점에서 이만큼 가면 뒤 레인이 풀려 뒤차가 꼭짓점에 설 수 있다.
    # 물러난 중심은 마지막으로 지난 경로 꼭짓점에서 이것 + backup_margin_m 앞에 남는다
    release_distance_m: float = 0.2
    backup_margin_m: float = 0.03  # 관제가 보는 위치 오차(eps)
    backup_min_m: float = 0.02  # 이보다 짧으면 물러나지 않고 지도만 비운다
    # 후진은 로봇 방향으로 곧게 가므로 옆으로 밀린다. 관제가 위치 오차로 보는 값(eps)까지만
    backup_lateral_limit_m: float = 0.03
    backup_speed_mps: float = 0.08
    backup_time_allowance_s: float = 5.0
    retreat_stop_wait_s: float = 3.0  # 물러나기 전 달리던 로봇이 서기를 기다리는 한계(1-A, 추정)
    # 후진 controller(pinky_nav_config FollowPathReverse)의 desired_linear_vel. 경로 물러나기를
    # 이보다 느리게 시키면 speed_limit으로 줄인다(RPP는 제한을 속도로 쓰므로 빠르게는 안 준다)
    reverse_speed_mps: float = 0.05
    backup_result_grace_s: float = 2.0  # time allowance 뒤에도 결과가 없으면 멈추고 이어 간다
    # 물러나기가 취소한 FollowPath의 결과를 이만큼만 기다린다. 결과를 잃어도 서 있으면 물러난다
    # (Codex 리뷰 2, 추정)
    nav_result_grace_s: float = 2.0
    recovery_pass_m: float = 0.05  # 막힌 곳을 이만큼 지나면 복구가 끝난다
    recovery_timeout_s: float = 10.0  # 다시 보낸 뒤 이만큼 못 지나면 다음 시도로 친다
    recovery_reset_m: float = 0.3  # 첫 시도 자리에서 이만큼 벗어나면 시도 수를 지운다
    # 잠금 경계에서 끝나는 목표(final_goal 거짓, 버전 2 4절 제안): 끝까지 남은 거리 d에서
    # √(2·a·d)로 제한한다. 숫자는 추정이다(실측 a 전). 한계 이상이면 제한하지 않는다
    horizon_decel_mps2: float = 0.25
    horizon_min_mps: float = 0.05
    # RPP takes an absolute limit as desired_linear_vel (no min), so never above nav2's 0.25
    horizon_cap_mps: float = 0.25
    # Time from a position update to the limit acting in the controller (control tick,
    # speed_limit topic, RPP cycle). Assumed, not measured: at horizon_cap_mps this keeps
    # horizon_cap_mps * horizon_latency_s (0.075 m) of the distance left out of sqrt(2 a d)
    horizon_latency_s: float = 0.3
    # mode nav_goal(계약 0.11-draft 제안): 이만큼 진전이 없으면 BLOCKED(goal은 그대로). 진전은
    # nav_progress_m 이상 움직인 것이다(nav_goal.py). 숫자는 제안이다
    blocked_after_s: float = 5.0
    nav_progress_m: float = 0.05
