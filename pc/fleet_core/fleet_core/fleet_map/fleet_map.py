from __future__ import annotations

import logging
import math
from pathlib import Path

import yaml

from .checks import check_clearance, check_reach, retreat_end_warnings, shelf_end_warnings
from .geometry import inside, point_rect_distance
from .model import WAITING_KINDS, FleetMapError, Link
from .parse import read_links, read_nodes, read_parking, read_rect, read_stations, read_zones

DATA = Path(__file__).parents[1] / "data"
DEFAULT_LAYOUT = "d2"
LAYOUTS = ("d2", "d", "c")
TEST_LAYOUTS = ("d2_shelf",)
KNOWN_LAYOUTS = (*LAYOUTS, *TEST_LAYOUTS)
BODY_CLEARANCE_M = 0.06
SHELF_END_CLEAR_M = 0.15
TURN_RADIUS_M = 0.10
_log = logging.getLogger(__name__)


class FleetMap:
    def __init__(self, data: dict, source_text: str = ""):
        self.source_text = source_text
        self.name = str(data.get("name", ""))
        self._bounds = read_rect(data.get("bounds"), "bounds")
        self.shelves = tuple(read_rect(rect, "shelf") for rect in data.get("shelves") or ())
        self.shelf_height_m = float(data.get("shelf_height_m", 0.15))
        self.zones = read_zones(data.get("zones"), self._bounds)
        self.nodes = read_nodes(data.get("nodes"), self.zones)
        self._links = read_links(data.get("paths"), data.get("nav"), self.nodes, self.zones)
        self._all_links = tuple(self._links.values())
        self._from: dict[str, tuple[Link, ...]] = {
            node: tuple(link for (a, _), link in self._links.items() if a == node)
            for node in self.nodes}
        self.stations = read_stations(data.get("stations"), self.nodes)
        self.parking = read_parking(data.get("parking"), self.nodes)
        self.docks = {station.dock for station in self.stations.values()}
        check_clearance(self, float(data.get("clearance_m", BODY_CLEARANCE_M)))
        check_reach(self)
        clear = float(data.get("shelf_end_clear_m", SHELF_END_CLEAR_M))
        turn = float(data.get("turn_radius_m", TURN_RADIUS_M))
        self.warnings = tuple(shelf_end_warnings(self, clear, turn) + retreat_end_warnings(self))
        for warning in self.warnings:
            _log.warning("fleet map %s: %s", self.name, warning)

    def link(self, a: str, b: str) -> Link | None:
        return self._links.get((a, b))

    def links(self) -> tuple[Link, ...]:
        return self._all_links

    def links_from(self, node: str) -> tuple[Link, ...]:
        return self._from[node]

    def can_wait(self, node: str) -> bool:
        return self.nodes[node].kind in WAITING_KINDS

    def bounds(self) -> tuple[float, float, float, float]:
        return self._bounds

    def node_pose(self, node: str) -> tuple[float, float]:
        return (self.nodes[node].x, self.nodes[node].y)

    def zone_at(self, x: float, y: float) -> str | None:
        for zone in self.zones.values():
            if any(inside(rect, x, y) for rect in zone.rects):
                return zone.id
        return None

    def nodes_in(self, zone: str) -> list[str]:
        return [name for name, spec in self.nodes.items() if spec.zone == zone]

    def distance_to_zone(self, x: float, y: float, zone: str) -> float:
        return min(point_rect_distance(rect, x, y) for rect in self.zones[zone].rects)

    def nearest_node(self, x: float, y: float, nodes=None, radius: float | None = None):
        best, best_gap = None, math.inf
        for name in (self.nodes if nodes is None else nodes):
            gap = math.hypot(self.nodes[name].x - x, self.nodes[name].y - y)
            if gap < best_gap:
                best, best_gap = name, gap
        if radius is not None and best_gap > radius:
            return None
        return best


def load_fleet_map(layout=None) -> FleetMap:
    name = DEFAULT_LAYOUT if layout is None else layout
    path = Path(name)
    if not path.suffix:
        path = DATA / f"fleet_map_{name}.yaml"
    if not path.is_file():
        raise FleetMapError(f"no fleet map at {path}")
    return fleet_map_from_text(path.read_text(encoding="utf-8"))


def fleet_map_from_text(text: str) -> FleetMap:
    return FleetMap(yaml.safe_load(text), source_text=text)
