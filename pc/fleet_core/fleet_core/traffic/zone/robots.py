# ---- 로봇별 관제 기록 자료형 ----
from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.traffic.mission import IDLE

# ---- 로봇에게 낸 goal 기록 ----
BACKOFF = "backoff"


@dataclass
class Goal:
    kind: str
    id: int
    links: int = 0
    end: str = ""
    via: str = ""


# ---- order 모델 주문 기록 ----
ORDER = "order"


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
    asked_at: float = 0.0
    broken: bool = False
    horizon_at: float = 0.0
    warned: bool = False
    overrun: bool = False


# ---- 위치 다시 찾기 기록 ----
@dataclass
class Reloc:
    why: str  # 시킨 까닭: NOT_AT_TARGET 또는 DOCK_FAILED
    late: bool  # 재시도를 다 쓴 뒤(격리 직전)인가
    before: tuple  # 실패 전 (attempts, dock_tries). 성공하면 되돌린다
    attempt: int = 0
    goal_id: int = 0  # 0이면 아직 보내지 않았다
    sent_at: float = 0.0
    due: float = 0.0  # 이 시도의 기한(서기를 기다림, BUSY로 다시 보냄, 답 기다림 모두)
    busy: bool = False  # 지난 답이 BUSY(또는 STOP으로 거둠): 시도 수를 늘리지 않고 다시 보낸다
    xy: tuple = (0.0, 0.0)  # 보낼 때 보고 위치


# ---- 배차 기록 ----
@dataclass
class Dispatch:
    goal_id: int
    station_id: str


# ---- 로봇 한 대의 관제 상태: 위치와 보고 ----
@dataclass
class Robot:
    name: str
    placed: bool = False
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    robot_state: str = ""
    battery: float = 0.0
    charge: bool = False
    last_seen: float = 0.0
    pose_at: float | None = None
    suspect_quiet_until: float = 0.0
    # ---- 단계, 고장, 통신 상태 ----
    phase: str = IDLE
    fault: str = ""
    fault_reason: str = ""
    fault_xy: tuple | None = None
    pose_doubt: bool = False
    doubt: str = ""  # 위치 의심 까닭(suspect_hold). 있으면 구역을 놓지 않는다
    doubt_by: tuple | None = None  # (goal id, 확인 기한)
    reloc: Reloc | None = None  # 위치 다시 찾는 중. 있으면 구역을 쥐고 아무것도 보내지 않는다
    relocs: int = 0  # 이 목적지로 가며 다시 찾기를 시작한 수. 도킹 성공이나 새 목적지에서 0
    comm: str = ""
    recover_at: float = 0.0
    # ---- 트립, goal, 재시도 기록 ----
    node: str = ""
    off_node: tuple | None = None
    dest: str | None = None
    trip: object = None
    goal: Goal | None = None
    move_order: MoveOrder | None = None
    base_until: float = 0.0
    attempts: int = 0
    dock_tries: int = 0
    redock_since: float | None = None
    redock: bool = False
    retry_at: float = 0.0
    settle_until: float = 0.0
    arrival_by: float | None = None
    heard_since_success: bool = False
    not_sent_warned: bool = False
    # ---- 호출, 배차, 대기 기록 ----
    called: bool = False
    dispatch: Dispatch | None = None
    hold: bool = False
    asked_at: float | None = None
    asked_gap: float = 0.0
    wait_since: float | None = None
    wait_for: tuple = ()
    wait_warned: bool = False
    queued: bool = False
    # ---- 멈춤, 막힘, 무진척, 출발 감시 기록 ----
    stall_from: tuple | None = None
    stall_since: float = 0.0
    blocked_since: float | None = None
    blocked_noted: bool = False
    blocked_goal: int = 0
    rear_noted: bool = False
    idle_since: float | None = None
    dock_error_m: float = 0.0
    silent_from: tuple | None = None
    silent_since: float = 0.0
    silent_noted: bool = False
    start_goal: int = 0
    start_xy: tuple = (0.0, 0.0)
    start_since: float = 0.0
    start_done: bool = False
    # ---- 자가 복구, 구역 놓기, 양보, 물러나기 기록 ----
    self_recovery_since: float | None = None
    local_yield: str = ""
    local_yield_since: float | None = None
    local_yield_noted: str = ""
    sent_priority: int = -1
    clear_n: dict = field(default_factory=dict)
    outside_n: int = 0
    outside_noted: bool = False
    yield_until: float = 0.0
    yielded: tuple | None = None
    retreat_to: str = ""
    retreat_via: str = ""
    retreat_since: float | None = None
    retreat_behind: str = ""
    backoff_from: tuple | None = None
    obstacle: tuple | None = None
    detour: tuple | None = None
    robot_yield_at: float | None = None
    behind_since: float | None = None

    # ---- 가용 여부와 표시 상태 속성 ----
    @property
    def live(self) -> bool:
        return self.placed and not self.fault

    @property
    def available(self) -> bool:
        return self.live and not self.comm

    @property
    def state(self) -> str:
        return self.fault or self.comm or self.phase
