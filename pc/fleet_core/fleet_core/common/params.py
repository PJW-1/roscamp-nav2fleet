# ---- 운영 파라미터 정의와 파일에서 읽기 ----
from __future__ import annotations

import math
from dataclasses import dataclass, fields
from pathlib import Path

import yaml

DEFAULT_PARAMS = Path(__file__).parents[1] / "data" / "fleet_params.yaml"
COMM_LOSS_POLICIES = ("stop", "base_end")
# yaml에서 일부러 빼는 키. zone_clear_m은 0이면 footprint_radius_m + release_margin_m로 놓는다
OPTIONAL = frozenset({"zone_clear_m"})
# 0이 '끔'(zone_clear_m은 '식을 씀')인 키. 다른 시간·거리 키는 0보다 커야 한다
ZERO_MEANS_OFF = frozenset({"zone_clear_m", "idle_watch_s", "silent_limit_s", "start_limit_s",
                            "battery_low_pct"})
# 이 끝말의 키가 시간·거리·속도·백분율이다
MEASURE_SUFFIXES = ("_s", "_m", "_mps", "_pct")
# Pode 기본 blocked_after_s(pode_fsm/params.py). 계약에 없어 기본값을 적는다
ROBOT_BLOCKED_AFTER_S = 10.0
# 상태 보고 주기 0.5 s, 두 쪽 진전 재기 시작 차이, 여유
BLOCKED_REPORT_MARGIN_S = 2.0


# ---- FleetParams: 운영 파라미터 목록 ----
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
    input_pose_margin_m: float = 1.0
    stall_s: float = 12.0
    stall_move_m: float = 0.05
    dock_stall_s: float = 15.0
    blocked_mark_s: float = 10.0
    blocked_timeout_s: float = 60.0
    blocked_reset_m: float = 0.30
    blocked_reset_s: float = 20.0
    idle_watch_s: float = 0.0  # 기록 호환: 이 키가 없는 옛 기록은 감시 없이 돈다(파일은 10)
    battery_low_pct: float = 0.0
    battery_charged_pct: float = 80.0
    dispatch_retry_wait_s: float = 2.0
    dispatch_retry_limit: int = 3
    self_recovery_limit_s: float = 45.0
    traffic_mode: str = "zones"
    nav_speed_mps: float = 0.12
    follow_speed_mps: float = 0.12
    retreat_speed_mps: float = 0.05
    footprint_radius_m: float = 0.085
    release_margin_m: float = 0.05
    zone_clear_m: float = 0.0
    node_clear_m: float = 0.20
    lane_follow_gap_s: float = 4.0
    lane_hysteresis_s: float = 5.0
    lane_handover_s: float = 30.0
    dock_wait_estimate_s: float = 30.0
    blocked_retry_s: float = 60.0
    near_m: float = 0.30
    near_stuck_s: float = 5.0
    outside_zone_m: float = 0.05
    outside_zone_samples: int = 5
    pose_jump_window_s: float = 2.0
    pose_jump_m: float = 0.15
    pose_jump_quiet_s: float = 5.0
    redock_wait_s: float = 10.0
    retreat_clear_m: float = 0.20
    use_order_model: bool = True
    lookahead_s: float = 2.0
    standby_lookahead_s: float = 0.5
    comm_base_hold_extra_s: float = 3.0
    comm_loss_policy: str = "stop"
    silent_limit_s: float = 0.0  # 기록 호환(파일은 60, test_zone_silent)
    start_limit_s: float = 0.0  # 기록 호환(파일은 5, test_zone_no_start)
    blocked_by_position: bool = True
    obstacle_look_m: float = 0.30
    check_range_m: float = 0.6
    check_radius_m: float = 0.10
    check_interval_s: float = 5.0
    clear_confirm: int = 2
    blocked_retry_max_s: float = 300.0

    # ---- 파라미터 값 검사 ----
    def __post_init__(self):
        if self.comm_loss_policy not in COMM_LOSS_POLICIES:
            raise ValueError(f"comm_loss_policy must be one of {COMM_LOSS_POLICIES}, "
                             f"got {self.comm_loss_policy!r}")


# ---- 파라미터 파일 읽기 ----
def load_params(path=None) -> FleetParams:
    source = Path(path) if path else DEFAULT_PARAMS
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{source}: expected a mapping of parameters")
    names = {f.name for f in fields(FleetParams)}
    missing = sorted(names - OPTIONAL - set(data))
    unknown = sorted(set(data) - names)
    if missing or unknown:
        raise ValueError(f"{source}: missing keys {missing}, unknown keys {unknown}")
    check_types(data, source)
    params = FleetParams(**data)
    check_measures(params, source)
    return params


def _finite(value) -> bool:
    # 실수로 못 바꾸는 큰 정수는 유한하지 않은 값으로 본다
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


# ---- 타입 검사: 불리언·문자열은 숫자가 아니고, 실수는 유한 ----
def check_types(data: dict, source="params") -> None:
    for f in fields(FleetParams):
        if f.name not in data:
            continue
        value = data[f.name]
        if f.type == "bool":
            ok = isinstance(value, bool)
        elif f.type == "str":
            ok = isinstance(value, str)
        elif f.type == "int":
            ok = isinstance(value, int) and not isinstance(value, bool)
        else:
            ok = (isinstance(value, (int, float)) and not isinstance(value, bool)
                  and _finite(value))
        if not ok:
            raise ValueError(f"{source}: {f.name} must be a finite {f.type}, got {value!r}")
    if data.get("release_confirm_samples", 1) < 1:
        raise ValueError(f"{source}: release_confirm_samples must be >= 1, "
                         f"got {data['release_confirm_samples']!r}")


# ---- 시간·거리 값 검사: 유한, 음수 아님, 끔이 아니면 0보다 큼 ----
def check_measures(params: FleetParams, source="params") -> None:
    for name in (f.name for f in fields(FleetParams)):
        if not name.endswith(MEASURE_SUFFIXES):
            continue
        value = getattr(params, name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{source}: {name} must be a number, got {value!r}")
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{source}: {name} must be finite and >= 0, got {value!r}")
        if value == 0 and name not in ZERO_MEANS_OFF:
            raise ValueError(f"{source}: {name} must be > 0 (0 does not mean off), got {value!r}")
        if name.endswith("_pct") and value > 100:
            raise ValueError(f"{source}: {name} must be <= 100, got {value!r}")


# ---- PC·로봇 시간 관계 경고(오류 아님) ----
def timing_warnings(params: FleetParams) -> list[str]:
    # move goal은 BLOCKED 보고가 멈춤 시계를 세운다. DOCK은 아니라 dock_stall_s는 보지 않는다
    need = ROBOT_BLOCKED_AFTER_S + BLOCKED_REPORT_MARGIN_S
    if params.stall_s < need:
        return [f"stall_s {params.stall_s:g} s is shorter than the robot default blocked_after_s "
                f"{ROBOT_BLOCKED_AFTER_S:g} s + {BLOCKED_REPORT_MARGIN_S:g} s = {need:g} s: "
                f"the fleet may take back a goal before the robot reports BLOCKED"]
    return []
