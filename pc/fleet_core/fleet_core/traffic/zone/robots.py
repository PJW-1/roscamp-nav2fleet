# ---- 로봇별 관제 기록 자료형 ----
from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.traffic.mission import IDLE

from .episodes import BlockedEpisode, Episode, WaitEpisode


# ---- 로봇에게 낸 goal 기록 ----
@dataclass
class Goal:
    kind: str
    id: int
    edges: int = 0
    end: str = ""
    mode: str = ""


# ---- order 모델 주문 기록 ----
ORDER = "order"


@dataclass
class LeftReport:
    seq: int
    m: float
    at: float
    jumped_at: float | None = None


@dataclass
class MoveOrder:
    id: str
    cuts: list
    given: int = 0
    done_seq: int = 0
    update: int = 0
    waiting: bool = False
    waited: bool = False
    before: tuple | None = None
    before_lost: int | None = None  # 결과를 잃은 이전 goal id. 되살리면 결과 모름(IR-62)
    asked_at: float = 0.0
    broken: bool = False
    horizon_at: float = 0.0
    warned: bool = False
    overrun: bool = False
    left: LeftReport | None = None


# ---- 배차 기록 ----
@dataclass
class Dispatch:
    goal_id: int
    station_id: str


# ---- 로봇이 보고한 상태 ----
@dataclass
class Report:
    robot_state: str = ""
    battery: float = 0.0
    charge: bool = False
    last_seen: float = 0.0
    local_yield: str = ""
    obstacle: tuple | None = None


# ---- 고장, 통신, 자세 의심 상태 ----
@dataclass
class Health:
    fault: str = ""
    fault_reason: str = ""
    fault_xy: tuple | None = None
    pose_doubt: bool = False
    comm: str = ""
    recover_at: float = 0.0
    pose_at: float | None = None
    suspect_quiet_until: float = 0.0
    localizing_since: float | None = None
    localizing_at: float | None = None
    aborted_at: float | None = None


# ---- 양보, 물러나기, 우회 기록 ----
@dataclass
class Avoid:
    yield_until: float = 0.0
    yielded: tuple | None = None
    retreat_to: str = ""
    retreat_via: str = ""
    retreat_since: float | None = None
    retreat_behind: str = ""
    detour: tuple | None = None
    robot_yield_at: float | None = None
    behind_since: float | None = None


# ---- 감시마다의 일화 기록 ----
@dataclass
class Watches:
    start: Episode = field(default_factory=Episode)
    silent: Episode = field(default_factory=Episode)
    idle: Episode = field(default_factory=Episode)
    stall: Episode = field(default_factory=Episode)
    outside: Episode = field(default_factory=Episode)
    wait: WaitEpisode = field(default_factory=WaitEpisode)
    local_episode: Episode = field(default_factory=Episode)
    redock: Episode = field(default_factory=Episode)
    blocked: BlockedEpisode = field(default_factory=BlockedEpisode)


# ---- 로봇 한 대의 관제 상태: 위치와 보고 ----
@dataclass
class Robot:
    name: str
    placed: bool = False
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    report: Report = field(default_factory=Report)
    # ---- 단계, 고장, 통신 상태 ----
    phase: str = IDLE
    health: Health = field(default_factory=Health)
    # ---- route, goal, 재시도 기록 ----
    node: str = ""
    off_node: tuple | None = None
    dest: str | None = None
    route: object = None
    goal: Goal | None = None
    move_order: MoveOrder | None = None
    base_until: float = 0.0
    attempts: int = 0
    dock_tries: int = 0
    retry_at: float = 0.0
    settle_until: float = 0.0
    arrival_by: float | None = None
    result_lost_at: float | None = None  # goal 결과를 잃은 시각(IR-62 결과 모름). 정하면 None
    heard_since_success: bool = False
    not_sent_warned: bool = False
    dock_error_m: float = 0.0
    sent_priority: int = -1
    # ---- 호출, 배차 기록 ----
    called: bool = False
    dispatch: Dispatch | None = None
    hold: bool = False
    mode_blocked: bool = False  # 운영 모드가 달라 새 호출에서 뺌 (IR-73)
    lock_released: bool = False  # 락이 풀려 새 호출에서 뺌 (W-34)
    asked_at: float | None = None
    asked_gap: float = 0.0
    # ---- 구역 놓기, 양보와 물러나기, 감시 기록 ----
    clear_n: dict = field(default_factory=dict)
    avoid: Avoid = field(default_factory=Avoid)
    watch: Watches = field(default_factory=Watches)

    # ---- 가용 여부와 표시 상태 속성 ----
    @property
    def live(self) -> bool:
        return self.placed and not self.health.fault

    @property
    def available(self) -> bool:
        return self.live and not self.health.comm

    @property
    def state(self) -> str:
        return self.health.fault or self.health.comm or self.phase
