"""매장 상황(머리 없는 부분): 사람과 상자로 막힘 처리 보완을 본다.

person_in_express, person_at_dock, corridor_box, box_removed, box_stays 다섯 가지다.
급행 안 상자와 마당 가운데 상자는 열 가지 상황의 express_obstacle, hall_obstacle을 보완 규칙으로 다시 돌린다.
"""
from __future__ import annotations

import math
from dataclasses import replace

from fleet_core.fleet_map import PASS, SHARED

from .injections import Obstacle
from .stations import six_orders

STORE_NAMES = ("person_in_express", "person_at_dock", "corridor_box", "box_removed",
               "box_stays")
STORE_ROWS = ("person_in_express", "person_at_dock", "express_obstacle", "corridor_box",
              "hall_obstacle", "box_removed", "box_stays")  # 매장 상황 표의 순서
PERSON_S = 8.0  # 사람이 서 있는 시간(blocked_mark_s 10 s보다 짧다)
QUIET_AFTER_S = 30.0  # 사람이 떠난 뒤 이만큼까지의 우회와 후진만 사람 탓으로 센다
BOX_R = 0.04  # 상자 반지름(급행 띠 0.20 m 안에서 지나갈 틈이 없다)
NODE_CORRIDORS = {"v7": ("남쪽 통로", "E-3", "Q2")}


def applies(name: str, layout: str, site) -> bool:
    """corridor_box는 지점 없는 통로가 있는 배치에서만 돌린다."""
    return name != "corridor_box" or _corridor(layout, site) is not None


def _corridor(layout, site):
    corridors = [z for z in site.zones.values() if z.kind == SHARED and not site.nodes_in(z.id)]
    zone = min(corridors, key=lambda z: z.id, default=None)
    if zone is not None:
        return zone.id, _middle(zone)
    if layout not in NODE_CORRIDORS:
        return None
    label, a, b = NODE_CORRIDORS[layout]
    (ax, ay), (bx, by) = site.node_pose(a), site.node_pose(b)
    return label, ((ax + bx) / 2, (ay + by) / 2)


def _dock_door(site):
    dock = site.node_pose(site.stations["WORK"].dock)
    doors = [n for n in site.nodes.values() if n.kind == PASS]
    door = min(doors, key=lambda n: math.dist((n.x, n.y), dock))
    return (door.x, door.y)


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
                         injections=[Obstacle(*_dock_door(site), 0.05, 15.0,
                                              15.0 + PERSON_S)],
                         no_events=quiet,  # 도크에서 나오는 후진은 사람이 비킬 때까지 다시 낸다
                         quiet_until_s=15.0 + PERSON_S + QUIET_AFTER_S)
    if name == "corridor_box":
        corridor = _corridor(layout, site)
        if corridor is None:
            raise KeyError(f"no corridor in layout {layout}")
        label, middle = corridor
        return situation(name, f"통로 안 상자 ({label})",
                         injections=[Obstacle(*middle, BOX_R, 20.0, 140.0)])
    if name == "box_removed":
        return situation(name, "상자 치움 뒤 표시 지우기",
                         orders=six_orders(gap_s=20.0, count=12),
                         injections=[Obstacle(*lane, BOX_R, 20.0, 140.0)], timeout_s=1500.0,
                         at_least_s=300.0)
    if name == "box_stays":
        # 막혀 표시한 로봇은 급행 안에서 도크의 로봇이 나가기를 기다린다(RETREAT_WAIT). 이 기다림은
        # 불변식 5번에 걸리므로 상자가 있는 동안과 치운 뒤 QUIET_AFTER_S까지만 허용한다
        waits = ("I5_STANDING",)
        s = situation(name, "상자 그대로",
                      orders=six_orders(gap_s=40.0, count=15),
                      injections=[Obstacle(*lane, BOX_R, 20.0, 620.0)], timeout_s=1800.0,
                      at_least_s=640.0, allowed=waits, allowed_until_s=620.0 + QUIET_AFTER_S)
        s.rough = replace(s.rough, allowed=waits)
        return s
    raise KeyError(f"no situation {name!r}")
