"""시나리오 설정값: 상수, 기본값, 설정 클래스와 ScenarioError."""
from __future__ import annotations

from dataclasses import dataclass, field

ORDER_RESULTS = ("SHIPPED", "PARTIAL", "HELD")
ROBOT_FAULTS = ("comm_loss", "nav_abort", "stuck", "operator_remove", "blocked", "drift",
                "operator_reinstate", "obstacle_behind", "low_battery")
ARM_FAULTS = ("arm_fail", "swap_partial")
# 로봇에 묶이지 않는 고장. object: 지도에 없는 물체가 (x, y)에 duration_s 동안 있다(짧으면 지나가는
# 사람, 길면 놓고 간 상자). 갈 길 앞에서 그것을 본 로봇은 서고 자기 복구 뒤 BLOCKED다. 관제는 모른다
WORLD_FAULTS = ("object",)
FAULT_TYPES = ROBOT_FAULTS + ARM_FAULTS + WORLD_FAULTS
# 이 고장이 난 로봇은 돌아오지 않으므로 주차를 기다리지 않는다.
GONE_FAULTS = ("stuck", "operator_remove")
OBJECT_RADIUS_M = 0.10  # object 고장의 기본 반지름: 사람 다리나 작은 상자
OBJECT_KEYS = ("x", "y", "radius_m")
OBJECT_MAP_MARGIN_M = 1.0  # 물체는 지도 범위에서 이만큼 안에 있어야 한다(오타 막기)
LAYOUTS = ("d2", "d", "c")  # map: 으로 고르는 배치(fleet_core/data/fleet_map_<배치>.yaml)
REINSTATE_BASKETS = ("EMPTY", "NONE")  # operator_reinstate가 관리자 대신 알리는 바구니
EXPECT_DEFAULTS = {
    "all_orders_finished": True,
    "orders_shipped": None,  # None은 검사 안 함, 숫자는 정확히 일치해야 함
    "orders_partial": None,
    "orders_held": None,
    "orders_shipped_min": 0,
    "fifo_violations": 0,
    "dual_arm_overlaps": 0,
    "zone_holder_max": 1,
    "released_without_position_check": 0,
    "unheld_zone_pose": 0,
    "direct_returns_min": 0,
    "parked_while_backlog": 0,  # null이면 안 봄(충전하러 가는 로봇은 주문이 밀려도 WAIT로 간다)
    "timed_out": False,  # 시간 초과가 목적인 시나리오에서만 true
    "unexpected_errors": 0,  # 코드가 events_include나 event_counts에 없는 ERROR 이벤트
    "bound_after_reinstate_min": 0,  # 다시 투입한 로봇이 그 뒤에 묶은 주문 수
    "shipped_after_reinstate_min": 0,  # 그중 되돌려지지 않고 출고까지 간 주문 수
    "order_results": {},  # 주문 id: SHIPPED, PARTIAL, HELD
    "events_include": [],
    "event_counts": {},  # 코드: 정확히 몇 번 나와야 하는지
    "shipped_before_operator_min": 0,  # 격리 뒤 confirm_clear 전 출고(지시서 5.5)
    "detours_around_isolated_min": 0,  # 격리된 로봇이 서 있는 동안의 REROUTE(치우기 전)
    "released_while_off_lane": 0,  # drift로 레인 밖이라 보고하는 동안 그 로봇이 놓은 구역 수
    "path_not_connected": 0,  # 관제가 처리한 PATH_NOT_CONNECTED 실패 수(지시서 7.5). null이면 안 봄
    "robot_idle": 0,  # ROBOT_IDLE 이벤트 수(1.1 지시서 H2). 정상이면 0, null이면 안 봄
}
TOP_LEVEL_KEYS = {"seed", "speed_factor", "timeout_s", "stock", "inventory", "orders", "robots",
                  "arm", "dock", "faults", "expect", "map", "random"}
RANDOM_KEYS = {"order_gap_s", "low_battery"}
LOW_BATTERY_KEYS = {"from_s", "to_s", "charge_s"}

class ScenarioError(ValueError):
    """시나리오 파일이 잘못됐다."""


@dataclass(frozen=True)
class RobotCfg:
    start: str  # 지도의 지점
    speed_mps: float = 0.2
    pose_noise_m: float = 0.0
    report_delay_s: float = 0.0  # 측정하고 이만큼 지난 뒤에 보고가 나간다
    stop_error_m: float = 0.0  # 축마다 표준편차: 이동이 목표 주변 어디서 끝나는지
    localizing_s: float = 0.0  # 시작하고 이만큼 LOCALIZING으로 보고한다(계약 3절 6번)


@dataclass(frozen=True)
class ArmCfg:
    seconds_per_item: float = 4.0
    swap_seconds: float = 8.0
    fail_rate: float = 0.0
    cancel_seconds: float = 0.0  # 가짜 작업셀: 취소부터 결과까지 걸리는 시간(서버는 바쁨)


@dataclass(frozen=True)
class DockCfg:
    duration_s: float = 3.0
    error_cm: float = 1.0


@dataclass(frozen=True)
class OrderCfg:
    at_s: float
    id: str
    items: dict


@dataclass(frozen=True)
class Fault:
    at_s: float
    robot: str = ""  # 로봇 고장의 대상
    type: str = ""
    duration_s: float = 0.0
    count: int = 1
    arm: str = ""  # arm_fail의 대상
    after_items: int = 0  # arm_fail: 한 목표에서 몇 개를 놓은 뒤 실패하는지
    start: str | None = None  # operator_reinstate: 로봇을 다시 놓는 지점
    basket: str = "EMPTY"  # operator_reinstate: 관리자가 알리는 바구니
    target: str = ""  # nav_abort: 이 지점으로 가는 목표만 중단. 비면 모든 목표
    offset_m: float = 0.0  # drift: 보고 위치를 진행 방향 왼쪽으로 미는 거리(음수면 오른쪽)
    distance_m: float = 0.0  # obstacle_behind: 로봇 뒤 물체까지(path를 따라). 후진을 막는다
    x: float = 0.0  # object: 물체 중심(맵 좌표, m)
    y: float = 0.0
    radius_m: float = OBJECT_RADIUS_M  # object: 물체 반지름


@dataclass(frozen=True)
class Expect:
    all_orders_finished: bool = True
    orders_shipped: int | None = None
    orders_partial: int | None = None
    orders_held: int | None = None
    orders_shipped_min: int = 0
    fifo_violations: int = 0
    dual_arm_overlaps: int = 0
    zone_holder_max: int = 1
    released_without_position_check: int = 0
    unheld_zone_pose: int = 0
    direct_returns_min: int = 0
    parked_while_backlog: int | None = 0
    timed_out: bool = False
    unexpected_errors: int = 0
    bound_after_reinstate_min: int = 0
    shipped_after_reinstate_min: int = 0
    order_results: dict = field(default_factory=dict)
    events_include: list = field(default_factory=list)
    event_counts: dict = field(default_factory=dict)
    shipped_before_operator_min: int = 0
    detours_around_isolated_min: int = 0
    released_while_off_lane: int = 0
    path_not_connected: int = 0
    robot_idle: int | None = 0


@dataclass(frozen=True)
class Scenario:
    seed: int
    speed_factor: float
    timeout_s: float
    stock: dict
    inventory: dict
    orders: list  # 등록 순서
    robots: dict
    arm: ArmCfg
    dock: DockCfg
    faults: list
    expect: Expect

    @property
    def order_ids(self) -> list[str]:
        return [o.id for o in self.orders]

    @property
    def gone_robots(self) -> set[str]:
        """끝까지 돌아오지 않는 로봇. 마지막 치우기 뒤에 다시 투입하면 빠진다."""
        gone = set()
        for f in sorted(self.faults, key=lambda f: f.at_s):  # 안정 정렬: 같은 시각은 파일 순서
            if f.type in GONE_FAULTS:
                gone.add(f.robot)
            elif f.type == "operator_reinstate":
                gone.discard(f.robot)
        return gone
