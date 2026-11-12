"""매장 상황(추가 지시서 3의 5절, 머리 없는 부분): 사람과 상자로 막힘 처리 보완(4절)을 본다.

| 이름 | 지시서의 상황 | 만드는 법 |
|---|---|---|
| person_in_express | 급행 안 사람 | 급행 가운데 물체 8 s(로봇이 들어오는 때) |
| person_at_dock | 도크 앞 사람 | 서쪽 갈림목(X_W) 물체 8 s |
| corridor_box | 통로 안 상자 (D) | 지점 없는 통로 가운데 상자 120 s. 통로가 있는 배치만(d, c) |
| box_removed | 상자 치움 뒤 표시 지우기 | 급행 가운데 상자 20~140 s, 주문을 나눠 넣는다 |
| box_stays | 상자 그대로 | 급행 가운데 상자 20~620 s, 주문을 나눠 넣는다 |

급행 안 상자(오래, 120 s)와 마당 가운데 상자(120 s)는 지시서 6-2의 express_obstacle과
hall_obstacle을 4절 규칙으로 다시 돌린다. 라이다보다 낮은 상자는 Gazebo에서만 본다(여기 없음).
"""
from __future__ import annotations

from fleet_core.fleet_map import SHARED

from .injections import Obstacle
from .stations import six_orders

STORE_NAMES = ("person_in_express", "person_at_dock", "corridor_box", "box_removed",
               "box_stays")
STORE_ROWS = ("person_in_express", "person_at_dock", "express_obstacle", "corridor_box",
              "hall_obstacle", "box_removed", "box_stays")  # 지시서 5절 표의 순서
PERSON_S = 8.0  # 사람이 서 있는 시간(blocked_mark_s 10 s보다 짧다)
QUIET_AFTER_S = 30.0  # 사람이 떠난 뒤 이만큼까지의 우회와 후진만 사람 탓으로 센다
BOX_R = 0.04  # 상자 반지름(급행 띠 0.20 m 안에서 지나갈 틈이 없다)


def applies(name: str, layout: str, site) -> bool:
    """corridor_box는 지점 없는 통로가 있는 배치에서만 돌린다."""
    return name != "corridor_box" or bool(_corridor(site))


def _corridor(site):
    corridors = [z for z in site.zones.values() if z.kind == SHARED and not site.nodes_in(z.id)]
    return min(corridors, key=lambda z: z.id, default=None)


def _middle(zone):
    x0, y0, x1, y1 = zone.rects[0]
    return ((x0 + x1) / 2, (y0 + y1) / 2)


def build_store(name, layout, site, lane, situation):
    """lane은 급행 가운데 점이다. situation은 situations.py의 Situation이다."""
    quiet = ("DEADLOCK_SEEN", "REROUTE", "ZONE_MARKED_BLOCKED")
    # 사람 때문의 우회만 본다: 떠난 뒤 QUIET_AFTER_S까지만 센다(person_passing과 같다)
    if name == "person_in_express":
        return situation(name, "급행 안 사람",
                         injections=[Obstacle(*lane, 0.05, 21.0, 21.0 + PERSON_S)],
                         max_retreats=0, no_events=quiet,
                         quiet_until_s=21.0 + PERSON_S + QUIET_AFTER_S)
    if name == "person_at_dock":
        return situation(name, "도크 앞 사람",
                         injections=[Obstacle(*site.node_pose("X_W"), 0.05, 15.0,
                                              15.0 + PERSON_S)],
                         no_events=quiet,  # 도크에서 나오는 후진은 사람이 비킬 때까지 다시 낸다
                         quiet_until_s=15.0 + PERSON_S + QUIET_AFTER_S)
    if name == "corridor_box":
        zone = _corridor(site)
        if zone is None:
            raise KeyError(f"no corridor in layout {layout}")
        return situation(name, f"통로 안 상자 ({zone.id})",
                         injections=[Obstacle(*_middle(zone), BOX_R, 20.0, 140.0)])
    if name == "box_removed":
        return situation(name, "상자 치움 뒤 표시 지우기",
                         orders=six_orders(gap_s=20.0, count=12),
                         injections=[Obstacle(*lane, BOX_R, 20.0, 140.0)], timeout_s=1500.0,
                         at_least_s=300.0)
    if name == "box_stays":
        return situation(name, "상자 그대로",
                         orders=six_orders(gap_s=40.0, count=15),
                         injections=[Obstacle(*lane, BOX_R, 20.0, 620.0)], timeout_s=1800.0,
                         at_least_s=640.0)
    raise KeyError(f"no situation {name!r}")
