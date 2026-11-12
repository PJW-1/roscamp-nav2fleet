"""읽은 지도 전체를 보는 검사: 몸이 들어갈 여유와 길이 이어지는지."""
from __future__ import annotations

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
    if x1 - x0 >= y1 - y0:  # 긴 축이 x
        middle, low, high, axis = (y0 + y1) / 2, x0, x1, 0
    else:
        middle, low, high, axis = (x0 + x1) / 2, y0, y1, 1
    da, db = a[1 - axis] - middle, b[1 - axis] - middle
    if (da > EPS and db > EPS) or (da < -EPS and db < -EPS):
        return False
    t = 0.0 if abs(da - db) <= EPS else da / (da - db)
    where = a[axis] + t * (b[axis] - a[axis])
    return where < low or where > high


def shelf_end_warnings(site, clear: float) -> list[str]:
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
