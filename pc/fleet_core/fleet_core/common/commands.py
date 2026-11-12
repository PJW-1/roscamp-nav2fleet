"""FMS 코어의 명령, 결과, 뷰. ROS는 모른다.

앞부분은 ROS 래퍼가 goal과 메시지로 바꾸고, 뒷부분은 FleetManager 안에서만 오간다.
"""
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
    # 레인 폴리라인 (x, y, yaw)들. 첫 점은 로봇 위치, 끝 점은 목표. 비면 점 목표(point 모드)
    path: tuple = ()
    # 끝이 임무의 마지막 지점인가(버전 2 4절 horizon 최소판, 계약 0.8-draft 제안)
    final_goal: bool = True
    # 가는 방식(계약 0.11-draft 제안). follow_path: path를 FollowPath로 따른다.
    # nav_goal: (x, y, yaw)로 Nav2 goal을 보내고 로봇이 길을 짠다. path는 비운다
    mode: str = "follow_path"


@dataclass(frozen=True)
class RetreatCmd:
    """관제가 시키는 물러나기(제안). 로봇의 retreat 액션으로 간다. path가 비면 보는 방향 그대로
    곧게 distance_m 후진, 있으면 그 꺾은선 (x, y, yaw)들을 후진으로 따라간다(첫 점은 로봇 위치,
    끝 점은 설 곳. distance_m은 그 길이)."""
    robot: str
    goal_id: int
    distance_m: float
    speed_mps: float
    path: tuple = ()


@dataclass(frozen=True)
class CheckClearCmd:
    """관제가 막힘 표시의 점 둘레가 비었는지 로봇에게 묻는다(추가 지시서 3의 4-3, 기준(사용자)).
    로봇 노드의 CheckClear 서비스로 간다. 답은 CLEAR, BLOCKED_AT, UNKNOWN 가운데 하나다."""
    robot: str
    check_id: int
    x: float
    y: float
    radius_m: float


CLEAR = "clear"  # 그 점에서 radius_m 안에 장애물 칸이 없다
BLOCKED_AT = "blocked"  # 있다
UNKNOWN = "unknown"  # 그 점이 센서나 local costmap 밖이다(묻지 못한 경우도 이것으로 본다)
OBSTACLE_KINDS = ("unknown", "person", "object", "robot")  # RobotState.obstacle_kind


@dataclass(frozen=True)
class SegmentOut:
    """order 모델의 조각 하나(계약 0.11-draft 제안의 Segment). 좌표는 map 프레임이다."""
    seq_id: int  # 이동 안에서 1부터. update를 거쳐도 이어진다
    mode: str  # follow_path / nav_goal / retreat
    end_node: str  # 이 조각이 끝나는 지점
    zones: tuple = ()  # 지나는 구역(표시와 검사용)
    path: tuple = ()  # follow_path와 retreat: (x, y, yaw)들. 첫 점은 앞 조각의 끝(또는 로봇 위치)
    x: float = 0.0  # nav_goal의 목표
    y: float = 0.0
    yaw: float = 0.0
    controller_id: str = ""  # FollowPath / FollowPathReverse. nav_goal이면 빈 값


@dataclass(frozen=True)
class MoveOrderCmd:
    """order 모델의 이동(목적지 하나까지) 또는 그 update. order_id가 있는 MoveTo goal 하나로 간다.

    이름의 order는 VDA 5050의 order다. 주문(OrderRecord)과는 다른 것이다. update도 새 goal로
    보낸다: 로봇은 이음 조건이 맞으면 이전 goal을 SUPERSEDED로 끝내고 새 goal 아래에서 이어 간다.
    """
    robot: str
    goal_id: int
    order_id: str
    update_id: int  # 처음 0, update마다 1씩
    stitch_node: str  # update일 때 이전 base의 마지막 지점. 처음이면 빈 값
    base: tuple = ()  # 새로 허락하는 SegmentOut들. 비면 horizon만 바꾸는 update
    horizon: tuple = ()  # 아직 허락하지 않은 SegmentOut들. 매번 통째로 바꾼다


@dataclass(frozen=True)
class DockCmd:
    robot: str
    goal_id: int
    dock_id: str


@dataclass(frozen=True)
class CancelCmd:
    robot: str  # 로봇 이름 또는 팔 작업 서버
    goal_id: int


@dataclass(frozen=True)
class FillCmd:
    goal_id: int
    robot: str
    station_id: str
    order_id: str
    items: dict[str, int]  # 바구니에 아직 없는 것만


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
    status: str  # SHIPPED | PARTIAL | HELD
    placed: dict[str, int]
    missing: dict[str, int]
    detail: str


@dataclass(frozen=True)
class EventOut:
    level: str  # INFO | WARN | ERROR
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
    task_type: str  # TO_WORK | FILL | TO_OUTBOUND | SWAP | WAIT


@dataclass(frozen=True)
class RobotStatusView:
    robot_id: str
    fsm_state: str  # 핑키가 보낸 값 그대로
    task_type: str  # AssignmentView와 같음, 코스 밖(투입 전, OUT_OF_SERVICE)이면 빈 값
    order_id: str
    fault: str  # NONE | ERROR | COMM_LOST | OUT_OF_SERVICE | RECOVERING
    fault_reason: str
    wait_zone: str
    wait_robots: tuple[str, ...]
    battery: float
    x: float
    y: float
    yaw: float
    base: tuple[str, ...] = ()  # 앞에 남은 길 가운데 구역을 허락받은 지점들
    horizon: tuple[str, ...] = ()  # 아직 허락받지 않은 지점들


# ---- inside the FMS: orders -> traffic --------------------------------------------------------


@dataclass(frozen=True)
class DispatchCmd:
    goal_id: int
    station_id: str
    robot_id: str = ""  # 비면 교통이 로봇을 고른다


@dataclass(frozen=True)
class CancelDispatchCmd:
    goal_id: int


@dataclass(frozen=True)
class ReleaseCmd:
    robot: str


@dataclass(frozen=True)
class RobotFaultCmd:
    """주문 쪽이 이 로봇을 포기했으니 교통도 실패로 다룬다."""

    robot: str
    reason: str


# ---- inside the FMS: orders -> arm -------------------------------------------------------------

# The station action written in the map, and the kind of an arm job
FILL = "fill"
SWAP = "swap"


@dataclass(frozen=True)
class ArmFillCmd:
    """One attempt to fill this robot's basket. The arm side sends a goal or answers ArmDeclined."""

    robot: str
    station_id: str
    order_id: str
    items: dict[str, int]  # only what is not in the basket yet


@dataclass(frozen=True)
class ArmSwapCmd:
    robot: str
    station_id: str
    order_id: str
    take_full: bool
    put_empty: bool


@dataclass(frozen=True)
class ArmStopCmd:
    """Cancel the running goal and wait for its result."""

    robot: str


@dataclass(frozen=True)
class ArmDropCmd:
    """Cancel the running goal and forget it. order_id and bound come back in ArmLost."""

    robot: str
    isolated: bool = False
    order_id: str = ""
    bound: int = 0


@dataclass(frozen=True)
class ArmFailedCmd:
    """The result just given was a failed attempt: count it; past the limit, fault the robot."""

    robot: str


@dataclass(frozen=True)
class ArmResetCmd:
    """This robot's arm work is done: attempts back to 0. With forget, its job record goes too."""

    robot: str
    forget: bool = False


ARM_COMMANDS = (ArmFillCmd, ArmSwapCmd, ArmStopCmd, ArmDropCmd, ArmFailedCmd, ArmResetCmd)


# ---- inside the FMS: arm -> orders -------------------------------------------------------------


@dataclass(frozen=True)
class ArmDeclined:
    """No goal was sent now (wait after a failure, server still stopping). Ask again."""

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
    """A goal with no result: never answered (cleared false) or dropped. A late one may come."""

    robot: str
    goal_id: int
    server: str
    kind: str  # FILL or SWAP
    cleared: bool = False
    isolated: bool = False
    order_id: str | None = None  # as ArmDropCmd gave it; None: the orders take what is bound now
    bound: int = 0


@dataclass(frozen=True)
class ArmLate:
    """The result of a goal no robot waits for."""

    goal_id: int
    kind: str  # FILL or SWAP
    placed: dict[str, int] | None = None
    took_full: bool = False
    put_empty_done: bool = False


ARM_REPORTS = (ArmDeclined, ArmFillResult, ArmSwapResult, ArmLost, ArmLate)


# ---- inside the FMS: traffic -> orders --------------------------------------------------------

NO_ROBOT = "NO_ROBOT"  # 지금 읽는 피드백 단계는 이것뿐


@dataclass(frozen=True)
class RobotIsolated:
    """교통이 이 로봇을 ERROR로 세웠다. 구역은 쥐고, 주문 쪽이 주문을 되돌린다."""

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
