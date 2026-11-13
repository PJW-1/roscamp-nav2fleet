# ---- 지도 검사: 여유 거리, 선반 끝, 경로 도달성 ----
from __future__ import annotations

import math
from itertools import pairwise

from .geometry import EPS, point_rect_distance, segment_rect_distance, wall_gap
from .graph import route
from .model import NAV_GOAL, RETREAT, SHARED, FleetMapError


# ---- 벽과 선반에서 떨어진 거리 검사 ----
def check_clearance(site, clearance: float) -> None:
    for node in site.nodes.values():
        gap = wall_gap(site.bounds(), node.x, node.y)
        if gap < clearance:
            raise FleetMapError(f"node {node.id} is {gap:.3f} m from a wall, needs {clearance}")
        for shelf in site.shelves:
            gap = point_rect_distance(shelf, node.x, node.y)
            if gap < clearance:
                raise FleetMapError(
                    f"node {node.id} is {gap:.3f} m from the shelf {shelf}, needs {clearance}")
    for link in site.links():
        if link.via == NAV_GOAL:
            continue
        for shelf in site.shelves:
            gap = min(segment_rect_distance(a, b, shelf) for a, b in pairwise(link.points))
            if gap < clearance:
                raise FleetMapError(f"path {link.path} from {link.a} to {link.b} passes "
                                    f"{gap:.3f} m from the shelf {shelf}, needs {clearance}")


# ---- 경로가 선반 끝을 도는지 판단 ----
def _rounds_the_end(a, b, shelf) -> bool:
    x0, y0, x1, y1 = shelf
    axis, low, high = _axis(shelf)
    middle = (y0 + y1) / 2 if axis == 0 else (x0 + x1) / 2
    da, db = a[1 - axis] - middle, b[1 - axis] - middle
    if (da > EPS and db > EPS) or (da < -EPS and db < -EPS):
        return False
    t = 0.0 if abs(da - db) <= EPS else da / (da - db)
    where = a[axis] + t * (b[axis] - a[axis])
    return where < low or where > high


def _axis(shelf):
    x0, y0, x1, y1 = shelf
    return (0, x0, x1) if x1 - x0 >= y1 - y0 else (1, y0, y1)


# ---- 모서리 호 점 계산 ----
def corner_arc(a, b, c, radius: float, count: int = 20, shares=(0.5, 0.5)) -> list:
    u = (b[0] - a[0], b[1] - a[1])
    v = (c[0] - b[0], c[1] - b[1])
    lu, lv = math.hypot(*u), math.hypot(*v)
    if radius <= 0.0 or lu <= EPS or lv <= EPS:
        return []
    u, v = (u[0] / lu, u[1] / lu), (v[0] / lv, v[1] / lv)
    turn = math.acos(max(-1.0, min(1.0, u[0] * v[0] + u[1] * v[1])))
    if turn <= 1e-6 or math.pi - turn <= 1e-6:
        return []
    reach = min(radius * math.tan(turn / 2), lu * shares[0], lv * shares[1])
    r = reach / math.tan(turn / 2)
    side = 1.0 if u[0] * v[1] - u[1] * v[0] > 0 else -1.0
    start = (b[0] - u[0] * reach, b[1] - u[1] * reach)
    centre = (start[0] - side * u[1] * r, start[1] + side * u[0] * r)
    a0 = math.atan2(start[1] - centre[1], start[0] - centre[0])
    return [(centre[0] + r * math.cos(a0 + side * turn * k / count),
             centre[1] + r * math.sin(a0 + side * turn * k / count)) for k in range(1, count)]


def _corners(site):
    for link in site.links():
        if link.via == NAV_GOAL:
            continue
        points = link.points
        for a, b, c in zip(points, points[1:], points[2:], strict=False):
            yield f"path {link.path} from {link.a} to {link.b} at {b}", a, b, c
        for nxt in site.links_from(link.b):
            if nxt.via == link.via and nxt.b != link.a:
                yield (f"path {link.path} from {link.a} to {link.b}, then {nxt.path} at {link.b}",
                       points[-2], points[-1], nxt.points[1])


# ---- 선반 끝과 모서리 근접 경고 ----
def shelf_end_warnings(site, clear: float, turn_radius: float = 0.0) -> list[str]:
    out = []
    for link in site.links():
        if link.via == NAV_GOAL:
            continue
        for shelf in site.shelves:
            gaps = [segment_rect_distance(a, b, shelf) for a, b in pairwise(link.points)
                    if _rounds_the_end(a, b, shelf)]
            if gaps and min(gaps) < clear - EPS:
                out.append(f"path {link.path} from {link.a} to {link.b} passes {min(gaps):.3f} m "
                           f"from the end of the shelf {shelf}, wants {clear}")
    for name, a, b, c in _corners(site):
        arc = [p for shares in ((0.5, 0.5), (1.0, 0.5), (0.5, 1.0), (1.0, 1.0))
               for p in corner_arc(a, b, c, turn_radius, shares=shares)]
        for shelf in site.shelves:
            axis, low, high = _axis(shelf)
            gaps = [point_rect_distance(shelf, *p) for p in arc
                    if p[axis] < low - EPS or p[axis] > high + EPS]
            if gaps and min(gaps) < clear - EPS:
                out.append(f"the corner of {name} passes {min(gaps):.3f} m from the end of the "
                           f"shelf {shelf}, wants {clear}")
    return out


# ---- 물러나기 경로 끝 경고 ----
def retreat_end_warnings(site) -> list[str]:
    out = []
    for link in site.links():
        if link.via != RETREAT:
            continue
        end = site.nodes[link.b]
        onward = any(nxt.via == RETREAT and nxt.path == link.path
                     for nxt in site.links_from(end.id))
        if not onward and site.zones[end.zone].kind != SHARED:
            out.append(f"path {link.path} ends at {end.id} inside {end.zone}: the robot would "
                       "turn in place in a narrow strip")
    return out


# ---- 도크와 주차 노드 사이, 도크끼리 길이 있는지 검사 ----
def check_reach(site) -> None:
    docks = sorted(site.docks)
    pairs = [(spot, dock) for spot in site.parking for dock in docks]
    pairs += [(dock, spot) for dock in docks for spot in site.parking]
    pairs += [(a, b) for a in docks for b in docks if a != b]
    for start, goal in pairs:
        if route(site, start, goal) is None:
            raise FleetMapError(f"there is no way from {start} to {goal}")
