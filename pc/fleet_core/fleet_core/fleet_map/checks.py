"""읽은 지도 전체를 보는 검사: 몸이 들어갈 여유와 길이 이어지는지."""
from __future__ import annotations

from .geometry import point_rect_distance, segment_rect_distance, wall_gap
from .graph import route
from .model import NAV_GOAL, FleetMapError


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
            gap = segment_rect_distance(link.points[0], link.points[-1], shelf)
            if gap < clearance:
                raise FleetMapError(f"path {link.path} from {link.a} to {link.b} passes "
                                    f"{gap:.3f} m from the shelf {shelf}, needs {clearance}")


def check_reach(site) -> None:
    """주차 자리마다 두 도크에 갈 수 있고, 두 도크에서 서로와 모든 주차 자리로 갈 수 있다."""
    docks = sorted(site.docks)
    pairs = [(spot, dock) for spot in site.parking for dock in docks]
    pairs += [(dock, spot) for dock in docks for spot in site.parking]
    pairs += [(a, b) for a in docks for b in docks if a != b]
    for start, goal in pairs:
        if route(site, start, goal) is None:
            raise FleetMapError(f"there is no way from {start} to {goal}")
