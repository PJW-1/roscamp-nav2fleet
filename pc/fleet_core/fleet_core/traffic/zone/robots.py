"""구역 교통이 보는 로봇 하나: 단계, 가는 길, goal, 배차. 로직 없는 기록이다."""
from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.traffic.mission import IDLE

BACKOFF = "backoff"  # 마당 물러나기(backoff.py). 구역 교통 층 안에서만 쓰는 goal 종류다


@dataclass
class Goal:
    kind: str  # mission.MOVE, DOCK, RETREAT 또는 BACKOFF
    id: int
    links: int = 0  # 이 명령이 끝나면 다 지난 것이 되는 연결 수(trip.links의 앞에서부터)
    end: str = ""  # 명령이 끝나는 지점
    via: str = ""  # nav_goal, follow_path, retreat


ORDER = "order"  # Goal.via: order 모델의 goal(조각 여러 개를 한 goal로 보냈다)


@dataclass
class MoveOrder:
    """order 모델로 보낸 이동 하나(VDA 5050의 order). 주문과는 다른 것이다."""
    id: str
    cuts: list  # 이동 전체의 조각: (번호, 첫 연결, 끝 연결의 다음)
    given: int = 0  # base로 준 조각 수(앞에서부터)
    done_seq: int = 0  # 로봇이 끝냈다고 알린 마지막 조각
    update: int = 0
    waiting: bool = False  # 로봇이 base 끝에 서서 update를 기다린다
    waited: bool = False  # 이번 기다림에 BASE_END_WAIT를 냈다
    # 로봇의 답을 기다리는 update가 있으면: (그 전 goal id, 그 전 base의 연결 수, 그때의 waiting)
    before: tuple | None = None
    asked_at: float = 0.0  # 그 update를 보낸 시각
    broken: bool = False  # update가 거절됐다. 지금 base가 끝나면 새 order로 잇는다
    horizon_at: float = 0.0  # horizon을 마지막으로 정한 시각
    warned: bool = False  # HORIZON_INVALID를 냈다
    overrun: bool = False  # base 밖의 조각을 끝냈다는 보고를 이미 알렸다


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
    pose_at: float | None = None  # 지금 자세를 들은 시각. 자세를 못 믿으면 None
    suspect_quiet_until: float = 0.0  # 이 시각까지는 LOCALIZATION_SUSPECT를 또 내지 않는다
    phase: str = IDLE
    fault: str = ""  # ERROR 또는 OUT_OF_SERVICE
    fault_reason: str = ""
    fault_xy: tuple | None = None  # ERROR가 될 때의 보고 위치
    pose_doubt: bool = False  # 위치를 못 믿어 세운 적이 있다(NOT_AT_TARGET). 다시 투입될 때까지
    comm: str = ""  # COMM_LOST 또는 RECOVERING
    recover_at: float = 0.0
    node: str = ""  # 서 있거나 마지막으로 떠난 지점
    # 그 지점 위라고 보았지만 로봇이 고정 경로에 붙지 못한 자리. goal 하나가 지점에서 끝날 때까지
    # 남는다: 그동안은 지점 바로 위로 보고해야 지점 위다(base.py의 _on_node)
    off_node: tuple | None = None
    dest: str | None = None  # 가는 중이거나 도킹한 스테이션
    trip: object = None  # trips.Trip
    goal: Goal | None = None
    move_order: MoveOrder | None = None  # order 모델로 달리는 중이면 그 order
    base_until: float = 0.0  # 통신이 끊겼다: 이 시각까지는 base 끝으로 달리는 중일 수 있다
    attempts: int = 0
    dock_tries: int = 0  # 이 스테이션에서 실패한 도킹 수. 도킹이 되거나 목적이 바뀌면 0
    redock_since: float | None = None  # 물러날 길을 처음 잡아 본 시각(redock_wait_s의 기준)
    redock: bool = False  # 도킹이 실패했다: 다시 하기 전에 물러났다 들어온다(redock.py)
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
    rear_noted: bool = False  # 뒤의 로봇 때문에 후진을 미룬다고 알렸다(RETREAT_WAIT)
    idle_since: float | None = None  # 할 일 없이 서 있기 시작한 시각
    dock_error_m: float = 0.0  # 마지막 도킹이 보고한 자리 오차(구역 놓기에 쓴다)
    silent_from: tuple | None = None  # 조용히 멈춤을 재기 시작한 자리(silent.py)
    silent_since: float = 0.0
    silent_noted: bool = False  # 이번에 SILENT_STALL을 냈다
    start_goal: int = 0  # 움직이기 시작했는지 재는 goal(nostart.py)
    start_xy: tuple = (0.0, 0.0)
    start_since: float = 0.0
    start_done: bool = False  # 그 goal은 다 보았다
    self_recovery_since: float | None = None
    # 로봇단 교착 처리(추가 지시서 3의 7-4): 마지막 보고의 단계(wait/backup/retry), 이번 처리의
    # 시작, 마지막으로 LOCAL_YIELD를 낸 단계
    local_yield: str = ""
    local_yield_since: float | None = None
    local_yield_noted: str = ""
    sent_priority: int = -1  # 마지막으로 보낸 MoveTo의 priority(/fleet/robots에 보인다)
    clear_n: dict = field(default_factory=dict)  # 뒤의 구역이나 지점 -> 벗어난 연속 보고 수
    outside_n: int = 0  # 쥔 구역 밖으로 벗어난 연속 보고 수
    outside_noted: bool = False
    yield_until: float = 0.0  # 넓은 구역에서 비켜 서 있는 시각
    yielded: tuple | None = None  # 마지막으로 비켜 선 (x, y, 시각)
    retreat_to: str = ""  # 막혀서 물러나는 중이면 물러날 지점
    retreat_via: str = ""  # 그 후진이 꺾는 갈림목 지점. 곧게 물러나면 빈 값
    retreat_since: float | None = None  # 물러나기로 정한 시각(물러날 길을 못 잡고 기다리는 한도)
    retreat_behind: str = ""  # 꺾이는 물러날 길 위에 있어 기다리는 로봇(RETREAT_WAIT를 한 번만)
    backoff_from: tuple | None = None  # 마당 물러나기를 시작한 자리
    # 마지막 보고의 장애물 (x, y, 종류). 로봇이 모르면 None(추가 지시서 3의 4-1)
    obstacle: tuple | None = None
    # 막혀 길을 버렸다: (그때의 목적지, 그때 지나던 구역들). 다음에 잡은 길로 우회 종류를 센다
    detour: tuple | None = None
    # 다른 로봇에 막혀 표시 없이 비킨 시각. blocked_retry_s 안에 또 막히면 예전처럼 표시한다
    robot_yield_at: float | None = None
    # 후진(Retreat)이 OBSTACLE_BEHIND로 처음 끝난 시각. 비키기를 blocked_timeout_s까지 기다린다
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
