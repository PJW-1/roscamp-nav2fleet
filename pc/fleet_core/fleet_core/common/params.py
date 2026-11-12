"""플릿 매니저 정책 값. data/fleet_params.yaml에서 읽는다."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_PARAMS = Path(__file__).parents[1] / "data" / "fleet_params.yaml"


@dataclass(frozen=True)
class FleetParams:
    basket_capacity: int
    ship_partial: bool
    robot_radius_m: float
    release_margin_extra_m: float
    release_distance_m: float
    release_confirm_samples: int
    lane_error_warn_m: float
    move_retry_limit: int
    move_retry_wait_s: float
    arrival_tolerance_m: float
    arrival_settle_s: float
    startup_wait_s: float
    comm_timeout_s: float
    heartbeat_period_s: float
    heartbeat_timeout_s: float
    recover_check_s: float
    arm_cancel_wait_s: float
    arm_retry_wait_s: float
    fill_timeout_s: float
    swap_timeout_s: float
    cancel_retry_s: float
    cancel_warn_after: int
    report_retry_s: float
    zone_wait_warn_s: float
    deadlock_grace_s: float
    score_same_call: float
    score_distance_weight: float
    priority_at_station: int
    priority_has_cargo: int
    # 기본값: 이 값이 없던 기록도 재생된다.
    input_pose_margin_m: float = 1.0
    stall_s: float = 6.0
    stall_move_m: float = 0.05
    dock_stall_s: float = 15.0
    blocked_mark_s: float = 10.0
    blocked_timeout_s: float = 60.0
    # 조용히 서 있는 로봇 감시(1.1 지시서 H2). 0이면 끈다. 이 값이 없던 기록은 감시 없이 재생한다
    idle_watch_s: float = 0.0
    # 제안(2026-11-10): 보고한 배터리(0-100)가 battery_low_pct 이하면 새 일을 받지 않고 대기
    # 지점(충전 자리)으로 가고, battery_charged_pct 이상이 되면 다시 받는다. 0이면 끈다(기본).
    # 0 이하로 보고한 배터리는 모르는 값으로 본다
    battery_low_pct: float = 0.0
    battery_charged_pct: float = 80.0
    # Proposal (work order 1.1, H3): a failed outbound dispatch is sent again after the wait,
    # and failing more than the limit is an ERROR. Records without these values replay with them
    dispatch_retry_wait_s: float = 2.0
    dispatch_retry_limit: int = 3
    # Proposal (pinky self-recovery, 2026-11-09): a robot that keeps reporting RECOVERING longer
    # than this is taken as BLOCKED. The pinky FSM's two attempts take about 35 s at most.
    # Records from before RECOVERING never report it, so the value does not change their replay
    self_recovery_limit_s: float = 45.0
    # Which traffic layer runs (traffic/modes.py). zones is the only one: zone permissions on a
    # graph of points and zones
    traffic_mode: str = "zones"
    # ETA(제안, 실측 전). eta_*는 버전 2 공식 ETA와 같은 값
    eta_fill_s_per_item: float = 4.0
    eta_fill_base_s: float = 4.0  # 첫 품목 전 준비 시간(가짜 팔은 품목 수 + 1번)
    eta_swap_s: float = 8.0
    # 구역 교통 층(traffic/zone). 모두 제안값이고 실측 전이다
    nav_speed_mps: float = 0.12  # 넓은 구역(Nav2 goal)의 ETA와 길 비용에 쓰는 속도
    follow_speed_mps: float = 0.12  # 고정 경로(FollowPath)의 같은 값
    retreat_speed_mps: float = 0.05  # 후진(Retreat) 속도
    # 뒤 구역은 보고 중심이 구역에서 footprint_radius_m + release_margin_m 넘게 벗어나면 놓는다
    # (몸 전체가 빠져나간 뒤, 추가 지시서 2의 5절)
    footprint_radius_m: float = 0.085  # Nav2 발자국 0.12 x 0.12 m의 반 대각선
    release_margin_m: float = 0.05  # 보고 위치 오차의 여유
    zone_clear_m: float = 0.0  # 옛 기록 전용: 0이 아니면 이 거리로 놓는다(그때의 규칙으로 재생)
    node_clear_m: float = 0.20  # 뒤 지점에서 이만큼 멀어지면 그 지점을 놓는다
    lane_follow_gap_s: float = 4.0  # 앞 로봇이 한 줄 구역에 들어간 뒤 이만큼 지나야 뒤따른다
    lane_hysteresis_s: float = 5.0  # 반대쪽이 이만큼 더 빨리 닿을 때만 방향을 바꾼다
    lane_handover_s: float = 30.0  # 이만큼 기다린 쪽에 방향을 넘긴다
    dock_wait_estimate_s: float = 30.0  # 찬 도크를 기다리는 예상 시간(길 비용)
    blocked_retry_s: float = 60.0  # 막힘으로 표시한 연결을 이만큼 뒤 다시 써 본다
    near_m: float = 0.30  # 넓은 구역에서 두 로봇이 이만큼 가까이
    near_stuck_s: float = 5.0  # 이만큼 못 나아가면 우선순위 낮은 쪽이 비킨다
    outside_zone_m: float = 0.05  # 쥔 구역 밖으로 이만큼 벗어난 보고가
    outside_zone_samples: int = 5  # 이만큼 이어지면 OUTSIDE_ZONE
    # 마당 물러나기(제안, 2026-11-12). 넓은 구역에서 비켜 서는 로봇이 BLOCKED를 보고했으면 선
    # 방향 그대로 뒤로 물린 뒤 세운다. 끄면(기본) 세우기만 한다
    yard_backoff: bool = False
    yard_backoff_m: float = 0.12  # 물러나는 거리
    # 비킬 로봇의 BLOCKED 보고를 이만큼 더 기다린다. near_stuck_s와 더해 stall_s보다 짧아야 한다
    yard_backoff_wait_s: float = 0.5
    yard_backoff_edge_m: float = 0.10  # 설 자리가 벽, 진열대, 고정 경로 구역에서 떨어질 거리
    # 관제가 시키는 물러나기(마당 물러나기, 고정 구간 후진)의 길이 다른 로봇에 이만큼 안으로
    # 다가가면 내지 않는다(추가 지시서 2의 4절: 뒤에 다른 로봇이 없을 때만 물러난다)
    retreat_clear_m: float = 0.20
    # order 모델(VDA 5050의 base/horizon, 추가 지시서). 끄면 조각 하나씩 보낸다
    use_order_model: bool = False
    lookahead_s: float = 2.0  # base 끝까지 이만큼 남으면 다음 구역을 잡아 본다
    comm_base_hold_extra_s: float = 3.0  # 끊긴 로봇의 base 구역을 base 끝 ETA + 이만큼 쥔다
    # 시험 판정(추가 지시서 2, 2026-11-12). 0이면 끈다. 이 값이 없던 기록은 감시 없이 재생한다
    silent_limit_s: float = 0.0  # 갈 곳이 있는 로봇이 나아가지 않고 관제도 말이 없으면 SILENT_STALL
    start_limit_s: float = 0.0  # goal을 보낸 뒤 이만큼 안에 2 cm 넘게 움직이지 않으면 NO_START
    # 막힘 처리 보완(추가 지시서 3의 4절, 기준(사용자)). 이 값이 없던 기록은 장애물 위치가 없으므로
    # 예전 규칙 7 그대로 재생한다
    blocked_by_position: bool = True  # 끄면 장애물 위치를 보지 않는다(예전 규칙 7, 비교용)
    obstacle_look_m: float = 0.30  # 로봇이 가던 길 앞 이만큼 안의 가장 가까운 장애물 칸을 보고한다
    check_range_m: float = 0.6  # 막힘 표시의 점에서 이만큼 안에 있는 로봇에게 CheckClear를 묻는다
    check_radius_m: float = 0.10  # CheckClear의 r: 그 점에서 이만큼 안에 장애물 칸이 있으면 blocked
    check_interval_s: float = 5.0  # 한 표시에 이만큼마다 한 번을 넘게 묻지 않는다
    clear_confirm: int = 2  # clear 답이 이만큼 이어지면 표시를 지운다(BLOCK_CLEARED)
    blocked_retry_max_s: float = 300.0  # 다시 막히면 두 배로 늘리는 지우기 시간의 상한


def load_params(path=None) -> FleetParams:
    source = Path(path) if path else DEFAULT_PARAMS
    return FleetParams(**yaml.safe_load(source.read_text(encoding="utf-8")))
