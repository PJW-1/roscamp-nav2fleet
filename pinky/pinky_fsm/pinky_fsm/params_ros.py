"""pinky_fsm 노드의 FSM 파라미터: 선언하고 읽어 FsmParams와 OrderParams로 만든다."""
from __future__ import annotations

from pinky_fsm.order_fsm import OrderParams
from pinky_fsm.params import FsmParams

_FLOAT_PARAMS = (
    "arrival_tolerance_m", "connect_tolerance_m", "refresh_within_m", "refresh_every_s",
    "heartbeat_timeout_s", "stopped_speed_mps", "stopped_turn_rps", "block_report_s",
    "block_clear_s", "block_timeout_s", "block_lookahead_m", "path_step_m", "turn_radius_m",
    "join_min_off_m", "join_slope",
    # 자가 복구
    "backup_max_m", "release_distance_m", "backup_margin_m", "backup_min_m",
    "backup_lateral_limit_m", "backup_speed_mps", "backup_time_allowance_s",
    "backup_result_grace_s", "recovery_pass_m", "recovery_timeout_s", "recovery_reset_m",
    # 관제가 시키는 물러나기(버전 2 1-A, 경로 물러나기)
    "retreat_stop_wait_s", "reverse_speed_mps", "nav_result_grace_s",
    # 잠금 경계 목표의 속도 제한(버전 2 4절 horizon 최소판)
    "horizon_decel_mps2", "horizon_min_mps", "horizon_cap_mps", "horizon_latency_s",
    # BLOCKED까지의 시간(두 mode 모두, 계약 0.13-draft)과 nav_goal의 진전 기준
    "blocked_after_s", "nav_progress_m",
    # 로봇단 교착 처리(추가 지시서 3의 7-4)
    "local_wait_s", "local_backup_m", "local_peer_near_m", "peers_timeout_s")
_INT_PARAMS = ("recovery_attempts",)
_ORDER_PARAMS = ("join_m", "pass_m")  # order 모델(계약 0.11-draft 제안). 이름 앞에 order_


def declare_fsm_params(node) -> tuple[FsmParams, OrderParams]:
    d = FsmParams()
    declare = node.declare_parameter
    o = OrderParams()
    for name in _ORDER_PARAMS:
        declare(f"order_{name}", float(getattr(o, name)))
    for name in _FLOAT_PARAMS:
        declare(name, float(getattr(d, name)))
    for name in _INT_PARAMS:
        declare(name, int(getattr(d, name)))
    declare("comm_loss_policy", d.comm_loss_policy)  # stop / base_end. 다른 값은 시작에서 거절
    declare("local_yield", d.local_yield)  # 끄면 0.12까지의 자가 복구
    get = node.get_parameter
    return (
        FsmParams(**{name: float(get(name).value) for name in _FLOAT_PARAMS},
                  **{name: int(get(name).value) for name in _INT_PARAMS},
                  comm_loss_policy=str(get("comm_loss_policy").value),
                  local_yield=bool(get("local_yield").value)),
        OrderParams(**{name: float(get(f"order_{name}").value) for name in _ORDER_PARAMS}),
    )
