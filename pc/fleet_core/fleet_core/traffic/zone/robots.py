from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.traffic.mission import IDLE

BACKOFF = "backoff"


@dataclass
class Goal:
    kind: str
    id: int
    links: int = 0
    end: str = ""
    via: str = ""


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


@dataclass
class Dispatch:
    goal_id: int
    station_id: str


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
    phase: str = IDLE
    fault: str = ""
    fault_reason: str = ""
    fault_xy: tuple | None = None
    pose_doubt: bool = False
    comm: str = ""
    recover_at: float = 0.0
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
    called: bool = False
    dispatch: Dispatch | None = None
    hold: bool = False
    asked_at: float | None = None
    asked_gap: float = 0.0
    wait_since: float | None = None
    wait_for: tuple = ()
    wait_warned: bool = False
    queued: bool = False
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

    @property
    def live(self) -> bool:
        return self.placed and not self.fault

    @property
    def available(self) -> bool:
        return self.live and not self.comm

    @property
    def state(self) -> str:
        return self.fault or self.comm or self.phase
