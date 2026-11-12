"""핑키 FSM의 매개변수. 기본값과 출처를 한곳에 둔다. ROS를 모른다."""
from __future__ import annotations

from dataclasses import dataclass

COMM_LOSS_POLICIES = ("stop", "base_end")


@dataclass(frozen=True)
class FsmParams:
    arrival_tolerance_m: float = 0.15  # R4, 관제 arrival_tolerance_m
    connect_tolerance_m: float = 0.15  # 안내 2.2, 관제 lane_error_warn_m
    refresh_within_m: float = 0.15  # R13
    refresh_every_s: float = 0.5  # R13
    heartbeat_timeout_s: float = 1.5  # R10. 0이면 멈추지 않는다
    # order 모델의 통신 끊김(추가지시서 3절 3번). stop: 목표를 바로 끝내고 선다(order가 아닌
    # 목표와 같다). base_end: 받은 base 끝까지 달리고 선다. 끊김은 heartbeat_timeout_s로 안다
    comm_loss_policy: str = "stop"
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
    # 추가 지시서 3의 2-1(기준(사용자)): 경로의 꺾이는 곳을 이 반지름의 호로 지난다. 폭 0.20 m 띠
    # 안에서 제자리 회전하면 몸 모서리가 벽 앞의 라이다 잡음에 닿는다. 0이면 꺾은 그대로 보낸다
    turn_radius_m: float = 0.10
    # 추가 지시서 3의 2-4(이음 정밀도): Nav2 goal은 지점에서 0.05~0.10 m 떨어져 끝난다. 로봇이
    # 받은 선에서 join_min_off_m 넘게 벗어나 있으면, Nav2에 보내는 경로를 로봇의 실제 위치에서
    # 시작해 벗어난 거리의 join_slope배 앞에서 선에 붙인다(들어가는 각 18° 이하). 선 위의 점에서
    # 시작하는 경로를 받은 컨트롤러는 옆으로 벗어난 만큼을 제자리 회전으로 메운다. 0이면 예전대로다
    join_min_off_m: float = 0.02
    join_slope: float = 3.0
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
    # 선 때부터 BLOCKED를 보고하기까지의 상한(추가 지시서 3의 7-4, 기준(사용자), 계약 0.13-draft).
    # 로봇단 교착 처리를 다 하고도 남는다: block_report_s 1 + local_wait_s 2 + 후진(멈추고 0.10 m를
    # 0.08 m/s, 가감속 포함 약 2) + 다시 가서 또 서기(0.10 m 되돌아가기 약 1 + block_report_s 1)
    # = 7 s, 여유 3 s. mode nav_goal에서는 진전 없이 이만큼 지나면 BLOCKED다(goal은 그대로). 진전은
    # nav_progress_m 이상 움직인 것이다(nav_goal.py). 0.12까지는 5 s(nav_goal만)였다
    blocked_after_s: float = 10.0
    nav_progress_m: float = 0.05
    # 로봇단 교착 처리(추가 지시서 3의 7-4, 기준(사용자)). 끄면 0.12까지의 자가 복구
    # (바로 후진, 두 번)다
    local_yield: bool = True
    local_wait_s: float = 2.0  # 물체 앞에 서서 먼저 기다리는 시간
    local_backup_m: float = 0.10  # 낮은 쪽이 지나온 길 안에서 물러나는 최대 거리
    # 관제 /fleet/robots의 다른 로봇 중심이 가던 길 앞 점에서 이만큼 안이면 그 로봇이 막았다
    # (발자국 반 대각선 0.085 m + 위치 오차와 여유, 추정)
    local_peer_near_m: float = 0.20
    peers_timeout_s: float = 3.0  # /fleet/robots(1 Hz)가 이보다 오래됐으면 쓰지 않는다

    def __post_init__(self):
        if self.comm_loss_policy not in COMM_LOSS_POLICIES:
            raise ValueError(f"comm_loss_policy must be one of {COMM_LOSS_POLICIES}, "
                             f"got {self.comm_loss_policy!r}")
