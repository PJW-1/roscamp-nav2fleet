"""지시서 6-2의 열 가지 상황: 평소 흐름에서는 잘 안 생기는 일을 일부러 만든다.

상황마다 세계를 만드는 값(출발 자리, 주문, 주입, 시간 한도)과 기대(expect)가 있다. 기대는 세계를
다 돌린 뒤 보고, 못 지킨 것을 글로 돌려준다. 자리는 배치(d2, d, c)마다 지도에서 읽는다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.fleet_map import LANE, load_fleet_map

from .injections import CommLoss, NoisyPose, Obstacle, Reinstate, Removal, SlowStation, Stuck
from .stations import six_orders

LAYOUTS = ("d2", "d", "c")


@dataclass
class Situation:
    name: str
    title: str  # 지시서 표의 상황 이름
    robots: dict | None = None  # 이름 -> 출발 지점. None이면 주차 자리
    orders: list = field(default_factory=six_orders)
    injections: list = field(default_factory=list)
    timeout_s: float = 900.0
    finish_all: bool = True  # 모든 주문이 끝나야 한다
    allowed: tuple[str, ...] = ()  # 생겨도 되는 불변식 위반 코드
    max_retreats: int | None = None
    need_events: dict = field(default_factory=dict)  # 사건 코드 -> 적어도 이만큼
    no_events: tuple[str, ...] = ("DEADLOCK_SEEN",)


def _hall_mid(site):
    (ax, ay), (bx, by) = site.node_pose("N_W"), site.node_pose("N_E")
    return ((ax + bx) / 2, (ay + by) / 2)


def _lane_mid(site):
    lane = next(z for z in site.zones.values() if z.kind == LANE)
    x0, y0, x1, y1 = lane.rects[0]
    return ((x0 + x1) / 2, (y0 + y1) / 2)


def _near_work(layout):
    if layout == "c":
        return {"pinky1": "N_W", "pinky2": "S_W", "pinky3": "P-2"}
    return {"pinky1": "N_W", "pinky2": "W_Q", "pinky3": "P-2"}


def build(name: str, layout: str = "d2") -> Situation:
    site = load_fleet_map(layout)
    hall, lane = _hall_mid(site), _lane_mid(site)
    if name == "express_both_ends":
        return Situation(name, "급행 양 끝에서 동시에",
                         robots={"pinky1": "W", "pinky2": "O", "pinky3": "P-2"})
    if name == "express_obstacle":
        return Situation(name, "급행 안 장애물(오래)",
                         injections=[Obstacle(*lane, 0.04, 20.0, 140.0)])
    if name == "hall_obstacle":
        return Situation(name, "마당 장애물", injections=[Obstacle(*hall, 0.05, 10.0, 130.0)])
    if name == "person_passing":
        return Situation(name, "사람이 잠깐 지나감", injections=[Obstacle(*hall, 0.05, 15.0, 23.0)],
                         max_retreats=0, no_events=("DEADLOCK_SEEN", "REROUTE"))
    if name == "outbound_slow":
        return Situation(name, "출고 도크 오래 씀", injections=[SlowStation("OUTBOUND", 6.0)],
                         timeout_s=1500.0)
    if name == "front_robot_stuck":
        # 멈춘 로봇은 서 있지 않은 구역을 돌려주고(ZONES_RETURNED) 그 주문은 다른 로봇에 간다.
        # c는 작업 도크로 가는 갈림목이 하나라 거기서 멈추면 아무것도 못 끝낸다.
        return Situation(name, "앞 로봇 고장", injections=[Stuck("pinky1", 20.0)],
                         finish_all=layout != "c", need_events={"ROBOT_ISOLATED": 1})
    if name == "order_burst":
        return Situation(name, "주문 몰림", orders=six_orders(gap_s=2.0, count=10),
                         timeout_s=1500.0)
    if name == "comm_loss_reinstate":
        return Situation(name, "통신 끊김, 재투입",
                         injections=[CommLoss("pinky2", 30.0, 60.0), Removal("pinky2", 70.0),
                                     Reinstate("pinky2", 90.0, site.parking[1])],
                         need_events={"COMM_LOST": 1, "ROBOT_REINSTATED": 1})
    if name == "three_at_junction":
        return Situation(name, "세 대가 갈림목에 동시에", robots=_near_work(layout),
                         orders=six_orders(count=3))
    if name == "pose_jumps":
        # 20초 동안 보고가 0.30 m 옆으로 밀린다(위치 잡기가 틀어짐). OUTSIDE_ZONE은 보고가 쥔
        # 구역들 밖으로 outside_zone_samples번 이어져야 나온다. 넓은 구역이나 쥔 줄 안에서 밀리면
        # 안 나오므로 세기만 한다. 밀린 보고로 도착을 못 믿어 NOT_AT_TARGET 재시도 끝에 격리되면
        # 그 로봇은 재시도 사이에 갈림목에 서 있을 수 있고(5번), 남은 주문은 못 끝날 수 있다(c).
        return Situation(name, "위치가 크게 튐",
                         injections=[NoisyPose("pinky1", 20.0, 40.0, sigma_m=0.03, jump_m=0.30)],
                         finish_all=layout != "c", allowed=("I5_STANDING",))
    raise KeyError(f"no situation {name!r}")


NAMES = ("express_both_ends", "express_obstacle", "hall_obstacle", "person_passing",
         "outbound_slow", "front_robot_stuck", "order_burst", "comm_loss_reinstate",
         "three_at_junction", "pose_jumps")
