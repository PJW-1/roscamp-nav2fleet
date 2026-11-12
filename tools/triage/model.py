"""분류가 읽는 한 번의 실행(Run), 장애물, 결과 한 줄(Row). 어떤 기록이든 이 꼴로 바꿔 읽는다."""
from __future__ import annotations

from dataclasses import dataclass, field

CAUSES = ("관제", "주행", "회피", "진짜 막힘", "시험 틀")

# 목표를 보냈다는 사건(질문 1). ORDER_SENT는 지시서에 있으나 코어에는 아직 없다.
# RETREAT_START는 관제가 낸 후진 명령이다(도크에서 나올 때 첫 명령)
GOAL_CODES = ("NAV_GOAL_SENT", "FOLLOW_SENT", "ORDER_SENT", "RETREAT_START")
# 막혔다는 사건(질문 3). 로봇 상태 BLOCKED 보고도 같다
BLOCK_CODES = ("NAV_ABORTED",)
# 시험 틀: 이 사건이 있거나 상태 보고가 없으면 관제와 로봇을 탓하지 않는다
FRAME_CODES = ("SIM_DIED", "SIM_ERROR", "SETUP_FAILED", "BRIDGE_ERROR", "INPUT_REJECTED",
               "ROBOT_NOT_REPORTING")
# 막힌 뒤 관제가 보인 첫 반응(3절: 1 s 안)
FIRST_REACTION = ("BLOCKED_WAIT", "ZONE_MARKED_BLOCKED", "REROUTE", "RETREAT_START",
                  "GOAL_FAILED", "GOAL_RETRY", "ROBOT_ISOLATED", "ORDER_HELD", "ORDER_FAILED",
                  "ORDER_REQUEUED", "HALL_YIELD", "NAV_GOAL_SENT", "FOLLOW_SENT")
# 다음 단계로 넘어갔다는 사건(막힘 표시 또는 포기)
NEXT_STAGE = ("ZONE_MARKED_BLOCKED", "REROUTE", "GOAL_FAILED", "ROBOT_ISOLATED", "ORDER_HELD",
              "ORDER_FAILED", "ORDER_REQUEUED")


@dataclass(frozen=True)
class Event:
    t: float
    code: str
    robot: str = ""
    detail: str = ""
    level: str = "INFO"


@dataclass(frozen=True)
class Obstacle:
    """지도에 없는 원 장애물. t0부터 t1 전까지 있다(headless의 Obstacle과 같은 꼴)."""
    x: float
    y: float
    radius: float = 0.05
    t0: float = 0.0
    t1: float = float("inf")


@dataclass(frozen=True)
class Row:
    order_id: str
    cause: str
    event: str
    t: float


@dataclass
class Run:
    events: list[Event] = field(default_factory=list)
    states: dict = field(default_factory=dict)  # 로봇 -> [(t, x, y, 상태)] 로봇이 보고한 것
    poses: dict = field(default_factory=dict)  # 로봇 -> [(t, x, y)] 참 위치(있을 때만)
    orders: list[str] = field(default_factory=list)  # 주문판에 있던 id, 처음 본 순서
    reports: dict = field(default_factory=dict)  # 주문 id -> 끝 보고 (t, SHIPPED|PARTIAL|HELD)
    end_t: float = 0.0
    cut: bool = False  # 기록이 줄 중간에서 끝남
    params: dict = field(default_factory=dict)  # 기록 머리의 관제 파라미터

    def add_order(self, order_id: str) -> None:
        if order_id and order_id not in self.orders:
            self.orders.append(order_id)

    def moves_of(self, robot):
        """움직임을 재는 위치: 참 위치가 있으면 그것, 없으면 보고."""
        return self.poses.get(robot) or [(t, x, y) for t, x, y, _ in self.states.get(robot, ())]
