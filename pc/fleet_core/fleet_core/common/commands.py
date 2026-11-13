from __future__ import annotations

from dataclasses import dataclass

SHIPPED = "SHIPPED"
PARTIAL = "PARTIAL"
HELD = "HELD"


@dataclass(frozen=True)
class OrderRecord:
    order_id: str
    items: dict[str, int]


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


CLEAR = "clear"
BLOCKED_AT = "blocked"
UNKNOWN = "unknown"
NO_ANSWER = ""
OBSTACLE_KINDS = ("unknown", "person", "object", "robot")
LOCAL_YIELD_STEPS = ("wait", "backup", "retry")


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
