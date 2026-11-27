# ---- 코어 사이에 오가는 명령, 보고, 조회 데이터 클래스 ----
from __future__ import annotations

from dataclasses import dataclass, field

# ---- 주문 상태, 보류 사유, 강제 출고 거절 사유 상수와 주문 기록 ----
SHIPPED = "SHIPPED"
PARTIAL = "PARTIAL"
HELD = "HELD"
CANCELED = "CANCELED"
HOLD_ROBOT_FAULT = "ROBOT_FAULT"
HOLD_ARM_FAULT = "ARM_FAULT"
# 강제 출고 거절은 계약 request_rejected 코드로 낸다
FORCE_NOT_HELD = "ORDER_STATE"
FORCE_ROBOT_FAULT = "NOT_READY"
FORCE_UNKNOWN_ORDER = "NOT_FOUND"
FORCE_NOT_IMPLEMENTED = "NOT_IMPLEMENTED"  # 주문 쪽이 아직 받지 않는 운반 건 명령
# 치우기 · 다시 투입 거절도 계약 request_rejected 코드로 낸다
REJECT_UNKNOWN_TARGET = "UNKNOWN_TARGET"
REJECT_NOT_FOUND = "NOT_FOUND"
REJECT_NOT_STOPPED = "NOT_STOPPED"
REJECT_NOT_READY = "NOT_READY"
REJECT_ORDER_STATE = "ORDER_STATE"
REJECT_LOCK_RELEASED = "LOCK_RELEASED"


@dataclass(frozen=True)
class OrderRecord:
    order_id: str
    items: dict[str, int]


# ---- 이동, 물러나기, 막힘 지점 확인 명령 ----
@dataclass(frozen=True)
class MoveCmd:
    robot: str
    goal_id: int
    target_id: str
    x: float
    y: float
    yaw: float
    path: tuple = ()
    final_goal: bool = True
    mode: str = "follow_path"
    priority: int = -1


@dataclass(frozen=True)
class RetreatCmd:
    robot: str
    goal_id: int
    distance_m: float
    speed_mps: float
    path: tuple = ()


@dataclass(frozen=True)
class CheckClearCmd:
    robot: str
    check_id: int
    x: float
    y: float
    radius_m: float


# ---- 막힘 지점 확인 답, 장애물 종류, 로봇단 교착 처리 단계 상수 ----
CLEAR = "clear"
BLOCKED_AT = "blocked"
UNKNOWN = "unknown"
NO_ANSWER = ""
OBSTACLE_KINDS = ("unknown", "person", "object", "robot")
LOCAL_YIELD_STEPS = ("wait", "backup", "retry")


# ---- 구간 출력과 이동 주문 명령 ----
@dataclass(frozen=True)
class SegmentOut:
    seq_id: int
    mode: str
    end_node: str
    zones: tuple = ()
    path: tuple = ()
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    controller_id: str = ""


@dataclass(frozen=True)
class MoveOrderCmd:
    robot: str
    goal_id: int
    order_id: str
    update_id: int
    stitch_node: str
    base: tuple = ()
    horizon: tuple = ()
    priority: int = -1


# ---- 도킹, 취소, 채우기, 교환, 주문 보고 명령 ----
@dataclass(frozen=True)
class DockCmd:
    robot: str
    goal_id: int
    dock_id: str


@dataclass(frozen=True)
class CancelCmd:
    robot: str
    goal_id: int


@dataclass(frozen=True)
class FillCmd:
    goal_id: int
    robot: str
    station_id: str
    order_id: str
    items: dict[str, int]


@dataclass(frozen=True)
class SwapCmd:
    arm: str
    goal_id: int
    robot: str
    station_id: str
    order_id: str
    take_full: bool
    put_empty: bool


@dataclass(frozen=True)
class ReportOrderCmd:
    order_id: str
    robot: str
    status: str
    placed: dict[str, int]
    missing: dict[str, int]
    detail: str
    hold_reason: str = ""  # HELD일 때만 채움


# ---- 운반 건 진행 보고: 끝 상태는 ReportOrderCmd가 맡는다 ----
TRIP_ASSIGNED = "ASSIGNED"
TRIP_FILLING = "FILLING"
TRIP_TRANSPORTING = "TRANSPORTING"
TRIP_FILLING_PAUSE = "FILLING_PAUSE"  # 관리자 [담기 재개] · [보충 불가] 대기
TRIP_STATUSES = (TRIP_ASSIGNED, TRIP_FILLING, TRIP_TRANSPORTING, TRIP_FILLING_PAUSE)


@dataclass(frozen=True)
class ReportTripCmd:
    trip_id: str  # 주문판 Order.order_id 값
    robot: str
    status: str  # TRIP_STATUSES 중 하나
    placed: dict[str, int]  # 누적
    missing: dict[str, int]
    reason: str = ""  # 사유 코드, 이번 주에는 비움


# ---- 이벤트 출력과 화면 조회 구조 ----
@dataclass(frozen=True)
class EventOut:
    level: str
    code: str
    detail: str
    robot: str = ""


@dataclass(frozen=True)
class ZoneView:
    zone_id: str
    holder: str
    queue: tuple[str, ...]
    acquired_at: float | None


@dataclass(frozen=True)
class AssignmentView:
    robot_id: str
    order_id: str
    station_id: str
    task_type: str


@dataclass(frozen=True)
class RobotStatusView:
    robot_id: str
    fsm_state: str
    task_type: str
    order_id: str
    fault: str
    fault_reason: str
    wait_zone: str
    wait_robots: tuple[str, ...]
    battery: float
    x: float
    y: float
    yaw: float
    base: tuple[str, ...] = ()
    horizon: tuple[str, ...] = ()
    priority: int = -1


# ---- 배차와 로봇 해제, 고장 명령 ----
@dataclass(frozen=True)
class DispatchCmd:
    goal_id: int
    station_id: str
    robot_id: str = ""


@dataclass(frozen=True)
class CancelDispatchCmd:
    goal_id: int


@dataclass(frozen=True)
class ReleaseCmd:
    robot: str


@dataclass(frozen=True)
class RobotFaultCmd:
    robot: str
    reason: str


# ---- 팔 작업 종류와 팔 명령 ----
FILL = "fill"
SWAP = "swap"


@dataclass(frozen=True)
class ArmFillCmd:
    robot: str
    station_id: str
    order_id: str
    items: dict[str, int]


@dataclass(frozen=True)
class ArmSwapCmd:
    robot: str
    station_id: str
    order_id: str
    take_full: bool
    put_empty: bool


@dataclass(frozen=True)
class ArmStopCmd:
    robot: str


@dataclass(frozen=True)
class ArmDropCmd:
    robot: str
    isolated: bool = False
    order_id: str = ""
    bound: int = 0


@dataclass(frozen=True)
class ArmFailedCmd:
    robot: str


@dataclass(frozen=True)
class ArmResetCmd:
    robot: str
    forget: bool = False


ARM_COMMANDS = (ArmFillCmd, ArmSwapCmd, ArmStopCmd, ArmDropCmd, ArmFailedCmd, ArmResetCmd)


# ---- 팔 보고 ----
@dataclass(frozen=True)
class ArmDeclined:
    robot: str


@dataclass(frozen=True)
class ArmFillResult:
    robot: str
    success: bool
    placed: dict[str, int]
    # 주문 층이 쓰는 칸: 실패 사유(W-41), 못 담은 것, 확인된 담은 수, 쥐고 있던 상품(W-43)
    reason: str = ""
    missing: dict[str, int] = field(default_factory=dict)
    placed_confirmed: dict[str, int] = field(default_factory=dict)
    uncertain: dict[str, int] = field(default_factory=dict)
    held: bool = False  # 쥔 채 락이 풀렸던 목표: 상품 행방을 몰라 격리여도 보류


@dataclass(frozen=True)
class ArmSwapResult:
    robot: str
    took_full: bool
    put_empty_done: bool


@dataclass(frozen=True)
class ArmLost:
    robot: str
    goal_id: int
    server: str
    kind: str
    cleared: bool = False
    isolated: bool = False
    order_id: str | None = None
    bound: int = 0


@dataclass(frozen=True)
class ArmLate:
    goal_id: int
    kind: str
    placed: dict[str, int] | None = None
    took_full: bool = False
    put_empty_done: bool = False
    uncertain: dict[str, int] | None = None


# 팔 고장: 재시도 한도 초과나 무응답
@dataclass(frozen=True)
class ArmFault:
    robot: str
    kind: str
    server: str
    reason: str


# 결과로 끝난 담기 실패: 관제는 다시 보내지 않고 관리자를 기다린다 (팔 관제 합의 ①)
# ---- 관리자 [복구] 답: 두 팔에 set_pause(false) · 남은 상품으로 새 담기 목표 ----
RESUME_SET_PAUSE_OFF = "SET_PAUSE_OFF"
RESUME_REFILL = "REFILL"


@dataclass(frozen=True)
class ArmFillPaused:
    robot: str
    server: str
    reason: str
    detail: str


# ---- 담기 진행 · 재개 · 작업대 막힘 보고 (팔 관제 합의 ④ · ⑥ · ⑪ · ⑫) ----
@dataclass(frozen=True)
class ArmFillProgress:
    robot: str
    placed: dict[str, int]  # 이번 목표에서 담은 것 (goal items 순서대로 placed_total만큼)


@dataclass(frozen=True)
class ArmFillPlaced:
    robot: str
    placed: dict[str, int]  # 끝난 목표에서 담은 것: 주문에 더하고 남은 상품을 다시 요청
    held: bool = False  # 쥐고 있던 상품의 행방을 모름: 격리여도 보류


@dataclass(frozen=True)
class ArmFillResumed:
    robot: str  # 관리자 [복구]: 남은 상품을 다시 요청해도 됨


@dataclass(frozen=True)
class ArmStationBlocked:
    station_id: str
    reason: str  # 비면 다시 열림


ARM_REPORTS = (ArmDeclined, ArmFillResult, ArmSwapResult, ArmLost, ArmLate, ArmFault,
               ArmFillPaused, ArmFillProgress, ArmFillPlaced, ArmFillResumed,
               ArmStationBlocked)


# ---- 배차 보고와 로봇 격리 ----
NO_ROBOT = "NO_ROBOT"


@dataclass(frozen=True)
class RobotIsolated:
    robot: str
    reason: str


@dataclass(frozen=True)
class DispatchFeedback:
    goal_id: int
    robot_id: str
    phase: str


@dataclass(frozen=True)
class DispatchResult:
    goal_id: int
    success: bool
    robot_id: str
    reason: str = ""
    error_x_cm: float = 0.0
    error_y_cm: float = 0.0
    error_yaw_deg: float = 0.0
