# ---- 코어 사이에 오가는 명령, 보고, 조회 데이터 클래스 ----
from __future__ import annotations

from dataclasses import dataclass

# ---- 주문 상태 상수와 주문 기록 ----
SHIPPED = "SHIPPED"
PARTIAL = "PARTIAL"
HELD = "HELD"


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


ARM_REPORTS = (ArmDeclined, ArmFillResult, ArmSwapResult, ArmLost, ArmLate)


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
