"""읽은 지도 전체를 보는 검사: 몸이 들어갈 여유와 길이 이어지는지."""
from __future__ import annotations

import math
from itertools import pairwise

from .geometry import EPS, point_rect_distance, segment_rect_distance, wall_gap
from .graph import route
from .model import NAV_GOAL, RETREAT, SHARED, FleetMapError


def check_clearance(site, clearance: float) -> None:
    """지점과 고정 경로가 진열대와 벽에서 몸 반 폭만큼 떨어져 있는지 본다.

    Nav2 goal 구간은 보지 않는다: 그 길은 Nav2가 짠다.
    """
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


def _rounds_the_end(a, b, shelf) -> bool:
    """선분 ab가 진열대 긴 축의 연장선을 진열대 밖에서 건너가는가, 곧 진열대 끝을 돌아가는가."""
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
    """진열대 긴 축: (축 번호 0=x 1=y, 끝 low, 끝 high)."""
    x0, y0, x1, y1 = shelf
    return (0, x0, x1) if x1 - x0 >= y1 - y0 else (1, y0, y1)


def corner_arc(a, b, c, radius: float, count: int = 20, shares=(0.5, 0.5)) -> list:
    """a -> b -> c의 꺾이는 곳 b를 핑키가 도는 호의 안쪽 점들(두 끝 접점은 뺀다).

    pinky_fsm geometry.rounded와 같은 호다. 접점까지의 길이는 radius * tan(꺾인 각 / 2)이고 들어오는
    조각과 나가는 조각 길이에 shares(기본 반씩)를 곱한 것을 넘지 않는다. 핑키는 선의 첫 선분과
    마지막 선분에서는 호가 그 선분을 다 쓰고(1.0) 가운데 선분에서는 반(0.5)까지 쓴다.
    곧거나 180° 되돌아가는 곳은 호가 없다.
    """
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
    """핑키가 호로 도는 꺾이는 곳: (이름, a, b, c). 연결 안의 꺾는 점과, 한 명령으로 이어 달리는
    두 고정 연결(같은 방식, trips.segment_end) 사이의 지점."""
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


def shelf_end_warnings(site, clear: float, turn_radius: float = 0.0) -> list[str]:
    """고정 경로가 진열대 끝을 돌아가는 곳은 끝에서 clear 이상 떨어져야 한다(B-G4: 0.10 m에서 끼임).

    진열대 옆을 나란히 지나는 선(급행)은 보지 않는다. 그 여유는 띠의 폭이 정한다.
    """
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
    # 핑키가 실제로 달리는 호도 잰다(Codex 검토 4): 진열대 끝 밖(긴 축 방향으로 진열대를 벗어난
    # 곳)의 점. 이어 달릴 때는 어느 선분이 명령의 처음과 끝인지 명령마다 다르니 두 조각을 반씩, 다,
    # 한쪽만 다 쓰는 네 호를 모두 잰다
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


def retreat_end_warnings(site) -> list[str]:
    """후진(Retreat) 경로는 넓은 구역의 지점에서 끝나야 한다. 좁은 띠(도크, 갈림목, 급행) 안에서
    끝나면 로봇은 다음 선의 방향으로 그 띠 안에서 제자리 회전을 해야 한다(추가 지시서 3의 2-1)."""
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


def check_reach(site) -> None:
    """주차 자리마다 두 도크에 갈 수 있고, 두 도크에서 서로와 모든 주차 자리로 갈 수 있다."""
    docks = sorted(site.docks)
    pairs = [(spot, dock) for spot in site.parking for dock in docks]
    pairs += [(dock, spot) for dock in docks for spot in site.parking]
    pairs += [(a, b) for a in docks for b in docks if a != b]
    for start, goal in pairs:
        if route(site, start, goal) is None:
            raise FleetMapError(f"there is no way from {start} to {goal}")
