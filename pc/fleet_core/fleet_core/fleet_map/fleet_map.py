# ---- FleetMap: 지도 데이터를 읽어 검사하고 조회 제공 ----
from __future__ import annotations

import logging
import math
from pathlib import Path

import yaml

from .checks import (
    check_clearance,
    check_lane_waits,
    check_reach,
    retreat_end_warnings,
    shelf_end_warnings,
)
from .geometry import inside, overlap_area, point_rect_distance
from .model import WAITING_KINDS, Edge, FleetMapError
from .parse import (
    read_edges,
    read_keepout_extra,
    read_nodes,
    read_parking,
    read_rect,
    read_splits,
    read_stations,
    read_zones,
)

# ---- 레이아웃 이름과 검사 기준 상수 ----
DATA = Path(__file__).parents[1] / "data"
DEFAULT_LAYOUT = "v7"  # 실제 경기장
LAYOUTS = ("d2", "d", "c", "v7")
TEST_LAYOUTS = ("d2_shelf",)
KNOWN_LAYOUTS = (*LAYOUTS, *TEST_LAYOUTS)
BODY_CLEARANCE_M = 0.06
SHELF_END_CLEAR_M = 0.15
TURN_RADIUS_M = 0.10
_log = logging.getLogger(__name__)


# ---- FleetMap 생성: 구역, 노드, edge, 스테이션 읽기와 검사 ----
class FleetMap:
    def __init__(self, data: dict, source_text: str = ""):
        self.source_text = source_text
        self.name = str(data.get("name", ""))
        self._bounds = read_rect(data.get("bounds"), "bounds")
        self.shelves = tuple(read_rect(rect, "shelf") for rect in data.get("shelves") or ())
        # 시뮬 전용 진열대. 없으면 shelves와 같다
        self.sim_shelves = self._read_sim_shelves(data)
        self.shelf_height_m = float(data.get("shelf_height_m", 0.15))
        self.keepout_extra = read_keepout_extra(data.get("keepout_extra"), self._bounds)
        self.zones = read_zones(data.get("zones"), self._bounds)
        self.nodes = read_nodes(data.get("nodes"), self.zones)
        self._edges = read_edges(data.get("paths"), data.get("nav"), self.nodes, self.zones)
        self._all_edges = tuple(self._edges.values())
        self._from: dict[str, tuple[Edge, ...]] = {
            node: tuple(edge for (a, _), edge in self._edges.items() if a == node)
            for node in self.nodes}
        self.stations = read_stations(data.get("stations"), self.nodes)
        self.parking = read_parking(data.get("parking"), self.nodes)
        self.docks = {station.dock for station in self.stations.values()}
        self.splits = read_splits(data.get("splits"), self.nodes, self._edges, self._bounds)
        check_lane_waits(self)
        check_clearance(self, float(data.get("clearance_m", BODY_CLEARANCE_M)))
        check_reach(self)
        clear = float(data.get("shelf_end_clear_m", SHELF_END_CLEAR_M))
        turn = float(data.get("turn_radius_m", TURN_RADIUS_M))
        self.warnings = tuple(shelf_end_warnings(self, clear, turn) + retreat_end_warnings(self))
        for warning in self.warnings:
            _log.warning("fleet map %s: %s", self.name, warning)

    def _read_sim_shelves(self, data: dict) -> tuple:
        """sim_shelves를 읽는다. 사각형마다 같은 순서의 shelves와 가장 많이 겹쳐야 한다."""
        if "sim_shelves" not in data:
            return self.shelves
        raw = data["sim_shelves"]
        if not isinstance(raw, list):
            raise FleetMapError(f"sim shelves: a list of [x0, y0, x1, y1], got {raw!r}")
        rects = tuple(read_rect(rect, "sim shelf") for rect in raw)
        if len(rects) != len(self.shelves):
            raise FleetMapError(f"sim shelves: {len(rects)} rectangles for "
                                f"{len(self.shelves)} shelves (one per shelf, same order)")
        for number, (sim, real) in enumerate(zip(rects, self.shelves, strict=True), start=1):
            own = overlap_area(sim, real)
            others = (shelf for index, shelf in enumerate(self.shelves, start=1) if index != number)
            if own <= 0.0 or any(overlap_area(sim, other) >= own for other in others):
                raise FleetMapError(f"sim shelves: rectangle {number} {list(sim)} does not overlap "
                                    f"shelf {number} {list(real)} more than any other shelf "
                                    f"(one per shelf, same order)")
        return rects

    # ---- edge와 노드 조회 ----
    def edge(self, a: str, b: str) -> Edge | None:
        return self._edges.get((a, b))

    def edges(self) -> tuple[Edge, ...]:
        return self._all_edges

    def edges_from(self, node: str) -> tuple[Edge, ...]:
        return self._from[node]

    def split_via(self, goal: str, x: float, y: float) -> str:
        """(x, y)에서 goal로 가는 Nav2 goal을 나눌 중간 지점. 없으면 빈 문자열."""
        for split in self.splits:
            if split.to == goal and any(inside(rect, x, y) for rect in split.rects):
                return split.via
        return ""

    def can_wait(self, node: str) -> bool:
        return self.nodes[node].kind in WAITING_KINDS

    def bounds(self) -> tuple[float, float, float, float]:
        return self._bounds

    def node_pose(self, node: str) -> tuple[float, float]:
        return (self.nodes[node].x, self.nodes[node].y)

    # ---- 구역과 가까운 노드 조회 ----
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


# ---- 지도 파일 불러오기 ----
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
