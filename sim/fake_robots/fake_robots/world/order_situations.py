"""order 추가지시서 6-2의 여섯 상황을 order 모델을 끄고 켜서 돌린다(D2).

상황마다 세계를 만드는 값과, 모델이 켜졌을 때만 보는 기대(on_events: 사건 코드 -> 적어도 이만큼)가
있다. 끝나면 표 한 줄(order_row)을 만든다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.fleet_map import load_fleet_map

from .injections import CommLoss, Obstacle, OrderFault, SlowStation
from .stations import six_orders
from .table import judge
from .world import ZoneWorld

ORDER_SITUATIONS = ("normal8", "comm_horizon", "extend_fail", "express_horizon", "burst10",
                    "stitch_fault")
WHERE = "구역 세계(헤드리스)"
ORDER_HEADER = ("상황", "배치", "order 모델", "어디서", "몇 번(씨드)", "끝남", "끝 시각",
                "이음매 공백 평균/최대(s)", "이동 한 건 시간(s)", "정지 횟수",
                "update 수", "거절", "base 밖 진입(세계 / SEGMENT_BEYOND_BASE)", "교착", "판정")


@dataclass
class OrderSituation:
    name: str
    title: str
    robots: dict | None = None
    orders: list = field(default_factory=six_orders)
    injections: list = field(default_factory=list)
    timeout_s: float = 1500.0
    on_events: dict = field(default_factory=dict)
    # 아래는 table.judge가 읽는다
    finish_all: bool = True
    allowed: tuple = ()
    max_retreats: int | None = None
    need_events: dict = field(default_factory=dict)
    no_events: tuple = ("DEADLOCK_SEEN",)


def build_order(name: str) -> OrderSituation:
    if name == "normal8":
        return OrderSituation(name, "평소 흐름 8건", orders=six_orders(count=8))
    if name == "comm_horizon":
        # pinky1이 첫 주문으로 작업 도크에 가는 길(horizon이 남은 때)에 15초 끊긴다
        return OrderSituation(name, "통신 끊김(horizon 있음)",
                              injections=[CommLoss("pinky1", 2.0, 17.0)],
                              on_events={"COMM_LOST": 1})
    if name == "extend_fail":
        return OrderSituation(name, "미리 늘리기 실패(출고 도크 오래 씀)",
                              injections=[SlowStation("OUTBOUND", 6.0)],
                              on_events={"BASE_END_WAIT": 1, "ORDER_UPDATED": 1})
    if name == "express_horizon":
        # 급행 양 끝에서 시작하고, 20~140초에 급행 안에 상자가 있다(돌아가는 길로 바뀜).
        # D2에서는 이 상황에서도 HORIZON_CHANGED가 나오지 않았다(수만 표에 적는다, 보고서 참조)
        x0, y0, x1, y1 = load_fleet_map("d2").zones["EXPRESS"].rects[0]
        box = Obstacle((x0 + x1) / 2, (y0 + y1) / 2, 0.04, 20.0, 140.0)
        return OrderSituation(name, "급행 방향 넘기기 중 horizon 바뀜",
                              robots={"pinky1": "W", "pinky2": "O", "pinky3": "P-2"},
                              injections=[box])
    if name == "burst10":
        return OrderSituation(name, "주문 몰림 10건", orders=six_orders(gap_s=2.0, count=10))
    if name == "stitch_fault":
        return OrderSituation(name, "이음 지점 어긋남", injections=[OrderFault(1.0, "stitch")],
                              on_events={"ORDER_UPDATE_REJECTED": 1})
    raise KeyError(f"no order situation {name!r}")


@dataclass
class OrderOutcome:
    world: ZoneWorld
    row: tuple
    failures: list[str]


def run_order_situation(name: str, order_model: bool, seed: int = 1,
                        layout: str = "d2") -> OrderOutcome:
    s = build_order(name)
    world = ZoneWorld(layout, seed=seed, overrides={"use_order_model": order_model},
                      robots=s.robots, orders=s.orders, injections=s.injections)
    world.run_until(world.orders_done, timeout_s=s.timeout_s)
    failures = judge(s, world)
    if order_model:
        for code, least in s.on_events.items():
            if world.metrics.count(code) < least:
                failures.append(f"{code} x{world.metrics.count(code)}, want >= {least}")
    return OrderOutcome(world, order_row(s, layout, order_model, seed, world, failures), failures)


def order_row(s, layout, order_model, seed, world, failures) -> tuple:
    m, om = world.metrics, world.order_metrics
    end = f"{m.end_t:.1f}" if m.end_t is not None else f"> {world.t:.0f}"
    on = "켬" if order_model else "끔"
    updates = m.count("ORDER_UPDATED") + m.count("HORIZON_CHANGED")
    refused = m.count("ORDER_UPDATE_REJECTED")
    return (s.title, layout, on, WHERE, seed, f"{world.central.finished}/{world.central.total}",
            end, om.seam_text(), om.trip_text(), om.stops,
            updates if order_model else "-", refused if order_model else "-",
            f"{om.outside_base} / {m.count('SEGMENT_BEYOND_BASE')}" if order_model else "-",
            m.count("DEADLOCK_SEEN"),
            "통과" if not failures else "실패")


def order_markdown(rows) -> str:
    lines = ["| " + " | ".join(ORDER_HEADER) + " |",
             "|" + "---|" * len(ORDER_HEADER)]
    lines += ["| " + " | ".join(str(c) for c in row) + " |" for row in rows]
    return "\n".join(lines)
