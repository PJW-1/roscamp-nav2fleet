"""구역 교통이 보는 로봇 하나: 단계, 가는 길, goal, 배차. 로직 없는 기록이다."""
from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.traffic.mission import IDLE


@dataclass
class Goal:
    kind: str  # mission.MOVE, DOCK, RETREAT
    id: int
    links: int = 0  # 이 명령이 끝나면 다 지난 것이 되는 연결 수(trip.links의 앞에서부터)
    end: str = ""  # 명령이 끝나는 지점
    via: str = ""  # nav_goal, follow_path, retreat


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
    charge: bool = False  # 배터리가 낮다: 새 일을 받지 않고 주차 자리에 있는다
    last_seen: float = 0.0
    phase: str = IDLE
    fault: str = ""  # ERROR 또는 OUT_OF_SERVICE
    fault_reason: str = ""
    fault_xy: tuple | None = None  # ERROR가 될 때의 보고 위치
    comm: str = ""  # COMM_LOST 또는 RECOVERING
    recover_at: float = 0.0
    node: str = ""  # 서 있거나 마지막으로 떠난 지점
    dest: str | None = None  # 가는 중이거나 도킹한 스테이션
    trip: object = None  # trips.Trip
    goal: Goal | None = None
    attempts: int = 0
    retry_at: float = 0.0  # 이 시각 전에는 새 명령을 내지 않는다
    settle_until: float = 0.0  # goal을 거둔 뒤 이 시각까지는 잡은 것을 놓지 않는다
    arrival_by: float | None = None  # 성공이 자세보다 먼저 왔다. 이 시각까지 보고를 기다린다
    heard_since_success: bool = False
    not_sent_warned: bool = False
    called: bool = False
    dispatch: Dispatch | None = None
    hold: bool = False  # 놓아 줄 때까지 스테이션에 둔다
    asked_at: float | None = None  # 지금 스테이션을 바라기 시작한 시각(먼저 온 순서)
    asked_gap: float = 0.0  # 그때 도크까지의 길 길이(m). 같은 시각에 바란 로봇의 순서다
    wait_since: float | None = None  # 구역을 못 잡아 서 있기 시작한 시각
    wait_for: tuple = ()  # 그때 길을 막은 구역과 지점
    wait_warned: bool = False
    queued: bool = False  # 이번 기다림에 DOCK_QUEUED를 냈다
    stall_from: tuple | None = None
    stall_since: float = 0.0
    blocked_since: float | None = None  # BLOCKED 보고가 이어지기 시작한 시각
    blocked_noted: bool = False  # BLOCKED_WAIT를 냈다
    blocked_goal: int = 0  # blocked_since를 잰 goal
    idle_since: float | None = None  # 할 일 없이 서 있기 시작한 시각
    self_recovery_since: float | None = None
    clear_n: dict = field(default_factory=dict)  # 뒤의 구역이나 지점 -> 벗어난 연속 보고 수
    outside_n: int = 0  # 쥔 구역 밖으로 벗어난 연속 보고 수
    outside_noted: bool = False
    yield_until: float = 0.0  # 넓은 구역에서 비켜 서 있는 시각
    retreat_to: str = ""  # 막혀서 물러나는 중이면 물러날 지점

    @property
    def live(self) -> bool:
        return self.placed and not self.fault

    @property
    def available(self) -> bool:
        return self.live and not self.comm

    @property
    def state(self) -> str:
        return self.fault or self.comm or self.phase
