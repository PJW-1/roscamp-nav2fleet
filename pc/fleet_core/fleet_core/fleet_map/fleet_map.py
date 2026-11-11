"""구역 지도: 지점, 구역, 고정 경로, 연결, 스테이션을 한 파일에서 읽는다.

로봇은 넓은 구역(shared)에서 Nav2 goal로 스스로 길을 짜고, 한 줄 구역(lane)과 도크 구역(dock)은
지도에 적힌 고정 경로를 FollowPath로 달린다. 관제는 이 지도로 "어느 구역에 들어가도 되나"만 정한다.
좌표는 지도 파일에만 둔다. 새 배치는 새 파일일 뿐이다.
"""
from __future__ import annotations

import math
from pathlib import Path

import yaml

from .checks import check_clearance, check_reach
from .geometry import inside, point_rect_distance
from .model import WAITING_KINDS, FleetMapError, Link
from .parse import read_links, read_nodes, read_parking, read_rect, read_stations, read_zones

DATA = Path(__file__).parents[1] / "data"
DEFAULT_LAYOUT = "d2"
BODY_CLEARANCE_M = 0.06  # Nav2 발자국의 반 폭. 지도에 clearance_m을 적으면 그 값을 쓴다


class FleetMap:
    def __init__(self, data: dict, source_text: str = ""):
        self.source_text = source_text  # 기록 머리에 적어 재생이 같은 지도를 쓰게 한다
        self.name = str(data.get("name", ""))
        self._bounds = read_rect(data.get("bounds"), "bounds")
        self.shelves = tuple(read_rect(rect, "shelf") for rect in data.get("shelves") or ())
        self.shelf_height_m = float(data.get("shelf_height_m", 0.15))
        self.zones = read_zones(data.get("zones"), self._bounds)
        self.nodes = read_nodes(data.get("nodes"), self.zones)
        self._links = read_links(data.get("paths"), data.get("nav"), self.nodes, self.zones)
        self._from: dict[str, tuple[Link, ...]] = {
            node: tuple(link for (a, _), link in self._links.items() if a == node)
            for node in self.nodes}
        self.stations = read_stations(data.get("stations"), self.nodes)
        self.parking = read_parking(data.get("parking"), self.nodes)
        self.docks = {station.dock for station in self.stations.values()}
        check_clearance(self, float(data.get("clearance_m", BODY_CLEARANCE_M)))
        check_reach(self)

    # ---- 연결 ----

    def link(self, a: str, b: str) -> Link | None:
        return self._links.get((a, b))

    def links(self) -> tuple[Link, ...]:
        return tuple(self._links.values())

    def links_from(self, node: str) -> tuple[Link, ...]:
        return self._from[node]

    def can_wait(self, node: str) -> bool:
        """로봇이 서서 기다려도 되는 자리(도크, 대기 자리, 주차)인가."""
        return self.nodes[node].kind in WAITING_KINDS

    # ---- 자리 ----

    def bounds(self) -> tuple[float, float, float, float]:
        return self._bounds

    def node_pose(self, node: str) -> tuple[float, float]:
        return (self.nodes[node].x, self.nodes[node].y)

    def zone_at(self, x: float, y: float) -> str | None:
        for zone in self.zones.values():
            if any(inside(rect, x, y) for rect in zone.rects):
                return zone.id
        return None

    def distance_to_zone(self, x: float, y: float, zone: str) -> float:
        """점에서 구역까지. 안이면 0. 모르는 구역 이름이면 KeyError."""
        return min(point_rect_distance(rect, x, y) for rect in self.zones[zone].rects)

    def nearest_node(self, x: float, y: float, nodes=None, radius: float | None = None):
        """가장 가까운 지점. nodes로 후보를, radius로 거리를 제한한다. 없으면 None."""
        best, best_gap = None, math.inf
        for name in (self.nodes if nodes is None else nodes):
            gap = math.hypot(self.nodes[name].x - x, self.nodes[name].y - y)
            if gap < best_gap:
                best, best_gap = name, gap
        if radius is not None and best_gap > radius:
            return None
        return best


def load_fleet_map(layout=None) -> FleetMap:
    """배치 이름(d2, c, d) 또는 파일 경로로 읽는다. 주지 않으면 기본 배치다."""
    name = DEFAULT_LAYOUT if layout is None else layout
    path = Path(name)
    if not path.suffix:
        path = DATA / f"fleet_map_{name}.yaml"
    if not path.is_file():
        raise FleetMapError(f"no fleet map at {path}")
    return fleet_map_from_text(path.read_text(encoding="utf-8"))


def fleet_map_from_text(text: str) -> FleetMap:
    """지도 파일의 본문으로 만든다. 기록 첫 줄의 지도(source_text)로 재생할 때 쓴다."""
    return FleetMap(yaml.safe_load(text), source_text=text)
