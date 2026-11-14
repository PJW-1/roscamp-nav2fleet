# ---- 지도 데이터 해석과 검사: 구역, 노드, 링크, 스테이션, 주차 ----
from __future__ import annotations

import math

from .geometry import _cross, inside, overlap_area, sample, within
from .model import (
    AT_DOCK,
    DOCK,
    FILL,
    FOLLOW_PATH,
    FORK,
    LANE,
    NAV_GOAL,
    NODE_KINDS,
    PARKING,
    PASS,
    RETREAT,
    SHARED,
    SWAP,
    WAIT,
    ZONE_KINDS,
    FleetMapError,
    Link,
    Node,
    Station,
    Zone,
)

SAMPLE_M = 0.01
LANE_CAPACITY = 2


# ---- 사각형 읽기 ----
def read_rect(raw, what: str) -> tuple:
    try:
        x0, y0, x1, y1 = (float(v) for v in raw)
    except (TypeError, ValueError) as error:
        raise FleetMapError(f"{what}: a rectangle is [x0, y0, x1, y1], got {raw!r}") from error
    if not (x0 < x1 and y0 < y1):
        raise FleetMapError(f"{what}: rectangle {raw!r} has no area")
    return (x0, y0, x1, y1)


# ---- 구역이 아닌 keepout 사각형 읽기 ----
def read_keepout_extra(raw, bounds: tuple) -> tuple:
    """Nav2 계획만 막는 사각형(마스크에 덧대지 않고 더한다). 구역이 아니라 쥐기와 정원에 들지 않는다."""
    rects = tuple(read_rect(rect, "keepout_extra") for rect in raw or ())
    for rect in rects:
        if not within(rect, bounds):
            raise FleetMapError(f"keepout_extra: {rect} is outside the floor {bounds}")
    return rects


# ---- 구역 읽기와 겹침 검사 ----
def read_zones(raw: dict, bounds: tuple) -> dict[str, Zone]:
    zones: dict[str, Zone] = {}
    for name, entry in (raw or {}).items():
        kind = entry.get("kind")
        if kind not in ZONE_KINDS:
            raise FleetMapError(f"zone {name}: kind must be one of {ZONE_KINDS}, got {kind!r}")
        rects = tuple(read_rect(rect, f"zone {name}") for rect in entry.get("rects") or ())
        if not rects:
            raise FleetMapError(f"zone {name} has no rects")
        for rect in rects:
            if not within(rect, bounds):
                raise FleetMapError(f"zone {name}: {rect} is outside the floor {bounds}")
        zones[str(name)] = Zone(str(name), kind, _capacity(name, kind, entry), rects)
    names = list(zones)
    for index, one in enumerate(names):
        for other in names[index + 1:]:
            if any(overlap_area(a, b) for a in zones[one].rects for b in zones[other].rects):
                raise FleetMapError(f"zones {one} and {other} overlap")
    return zones


def _capacity(name: str, kind: str, entry: dict) -> int:
    given = entry.get("capacity")
    if kind == DOCK:
        if given not in (None, 1):
            raise FleetMapError(f"zone {name}: a dock zone takes one robot, not {given}")
        return 1
    if given is None:
        if kind == SHARED:
            raise FleetMapError(f"zone {name}: a shared zone needs a capacity")
        return LANE_CAPACITY
    if int(given) < 1:
        raise FleetMapError(f"zone {name}: capacity {given} is below one")
    return int(given)


# ---- 점이 속한 구역 찾기 ----
def zone_under(zones: dict[str, Zone], x: float, y: float) -> list[str]:
    return [zone.id for zone in zones.values() if any(inside(r, x, y) for r in zone.rects)]


# ---- 노드 읽기와 구역 규칙 검사 ----
def read_nodes(raw: dict, zones: dict[str, Zone]) -> dict[str, Node]:
    nodes: dict[str, Node] = {}
    for name, entry in (raw or {}).items():
        kind = entry.get("kind")
        if kind not in NODE_KINDS:
            raise FleetMapError(f"node {name}: kind must be one of {NODE_KINDS}, got {kind!r}")
        x, y = float(entry["x"]), float(entry["y"])
        under = zone_under(zones, x, y)
        if not under:
            raise FleetMapError(f"node {name} at ({x}, {y}) is in no zone")
        if len(under) > 1:
            raise FleetMapError(f"node {name} is on the border of {under[0]} and {under[1]}")
        zone = zones[under[0]]
        if kind == AT_DOCK and zone.kind != DOCK:
            raise FleetMapError(f"node {name} (dock) must stand in a dock zone, not {zone.id}")
        if (kind in (PARKING, FORK) and zone.kind != SHARED
                or kind == WAIT and zone.kind not in (SHARED, LANE)):
            raise FleetMapError(f"node {name} ({kind}) must stand in a shared zone, not {zone.id}")
        if kind == PASS and zone.kind == SHARED:
            raise FleetMapError(f"node {name} (pass) belongs in a lane or dock zone, not {zone.id}")
        through = bool(entry.get("through", False))
        if through and kind != FORK:
            raise FleetMapError(f"node {name} ({kind}): only a fork can be a through node")
        yaw = entry.get("yaw_deg")
        nodes[str(name)] = Node(str(name), x, y, kind, zone.id,
                                None if yaw is None else math.radians(float(yaw)), through)
    return nodes


# ---- 경로가 지나는 구역과 다시 들어가는 구역 찾기 ----
def zones_along(zones: dict[str, Zone], points) -> tuple[str, ...]:
    seen: list[str] = []
    for a, b in zip(points, points[1:], strict=False):
        for x, y in sample(a, b, SAMPLE_M):
            for zone in zone_under(zones, x, y):
                if zone not in seen:
                    seen.append(zone)
    return tuple(seen)


def _reentered(zones: dict[str, Zone], points) -> str:
    left: set[str] = set()
    here: set[str] = set()
    for a, b in zip(points, points[1:], strict=False):
        for x, y in sample(a, b, SAMPLE_M):
            under = set(zone_under(zones, x, y))
            if not under:
                continue
            again = sorted(under & left)
            if again:
                return again[0]
            left |= here - under
            here = under
    return ""


# ---- 경로선 모양 검사 ----
def _line_problem(zones: dict[str, Zone], points) -> str:
    zone = _reentered(zones, points)
    if zone:
        return f"leaves {zone} and comes back into it"
    pieces = [(p, q) for p, q in zip(points, points[1:], strict=False) if math.dist(p, q) > 0.0]
    for (a, b), (c, d) in zip(pieces, pieces[1:], strict=False):
        if (b[0] - a[0]) * (d[0] - c[0]) + (b[1] - a[1]) * (d[1] - c[1]) < -1e-12:
            return f"turns back at {b}"
    for i, (a, b) in enumerate(pieces):
        for c, d in pieces[i + 2:]:
            if _cross(a, b, c, d):
                return "crosses itself"
    return ""


# ---- 링크 하나 만들기 ----
def _link(nodes, zones, a: str, b: str, via: str, path: str, crossed=None, bends=(),
          extra_m: float = 0.0) -> Link:
    points = ((nodes[a].x, nodes[a].y), *bends, (nodes[b].x, nodes[b].y))
    if via != NAV_GOAL:
        problem = _line_problem(zones, points)
        if problem:
            raise FleetMapError(f"path {path}: the line from {a} to {b} {problem}")
    crossed = tuple(crossed) if crossed else zones_along(zones, points)
    middle = [z for z in crossed if z not in (nodes[a].zone, nodes[b].zone)]
    ordered = [nodes[a].zone, *middle]
    if nodes[b].zone != nodes[a].zone:
        ordered.append(nodes[b].zone)
    lane = next((z for z in ordered if zones[z].kind == LANE), "")
    length = sum(math.dist(p, q) for p, q in zip(points, points[1:], strict=False))
    return Link(a, b, via, path, points, tuple(ordered), length, lane, extra_m)


# ---- 경로 지점 읽기 ----
def _path_stops(entry, nodes, what: str) -> list[tuple[str, tuple]]:
    stops, bends = [], []
    for point in entry.get("points") or ():
        if isinstance(point, str):
            if point not in nodes:
                raise FleetMapError(f"{what}: {point} is not a node")
            stops.append((point, tuple(bends)))
            bends = []
            continue
        try:
            x, y = (float(v) for v in point)
        except (TypeError, ValueError) as error:
            raise FleetMapError(f"{what}: a bend is [x, y], got {point!r}") from error
        if not stops:
            raise FleetMapError(f"{what}: the first point must be a node, got {point!r}")
        bends.append((x, y))
    if bends:
        raise FleetMapError(f"{what}: the last point must be a node, got {bends[-1]!r}")
    return stops


# ---- 경로와 nav 그룹에서 링크 읽기 ----
def read_links(paths: dict, nav: list, nodes: dict[str, Node], zones: dict[str, Zone]) -> dict:
    links: dict[tuple[str, str], Link] = {}

    def add(link: Link, what: str):
        if link.a == link.b:
            raise FleetMapError(f"{what}: link from {link.a} to itself")
        if (link.a, link.b) in links:
            raise FleetMapError(f"{what}: the link from {link.a} to {link.b} is written twice")
        links[(link.a, link.b)] = link

    def known(name, what: str) -> str:
        if name not in nodes:
            raise FleetMapError(f"{what}: {name} is not a node")
        return name

    for name, entry in (paths or {}).items():
        what = f"path {name}"
        stops = _path_stops(entry, nodes, what)
        if len(stops) < 2:
            raise FleetMapError(f"{what} needs two or more points")
        reverse, two_way = bool(entry.get("reverse")), bool(entry.get("two_way"))
        if reverse and two_way:
            raise FleetMapError(f"{what}: a reverse path runs one way")
        via = RETREAT if reverse else FOLLOW_PATH
        for (a, _), (b, bends) in zip(stops, stops[1:], strict=False):
            add(_link(nodes, zones, a, b, via, str(name), bends=bends), what)
            if two_way:
                add(_link(nodes, zones, b, a, via, str(name), bends=bends[::-1]), what)

    for index, group in enumerate(nav or ()):
        what = f"nav group {index + 1}"
        members = [known(node, what) for node in group.get("nodes") or ()]
        extra = _extra_m(group, what)
        if group.get("leave_dock"):
            add(_leave_dock(nodes, zones, members, what, extra), what)
            continue
        for node in members:
            if zones[nodes[node].zone].kind != SHARED:
                raise FleetMapError(f"{what}: {node} is not in a shared zone")
        crossed = group.get("zones")
        if crossed is not None:
            _check_named_zones(what, members, crossed, nodes, zones)
        for a in members:
            for b in members:
                if a == b:
                    continue
                named = None if crossed is None else (crossed if a == members[0] else crossed[::-1])
                link = _link(nodes, zones, a, b, NAV_GOAL, "", named, extra_m=extra)
                for zone in link.zones:
                    if zones[zone].kind != SHARED:
                        raise FleetMapError(
                            f"{what}: the line from {a} to {b} crosses {zone}, which is not "
                            "shared; write the zones it may use as `zones:`")
                add(link, what)
    return links


# ---- nav 그룹의 덧붙는 길이와 도크에서 풀려 나가는 이음 ----
def _extra_m(group, what: str) -> float:
    value = group.get("extra_m", 0.0)
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not 0.0 <= value < math.inf):
        raise FleetMapError(f"{what}: extra_m must be a length of 0 m or more, got {value!r}")
    return float(value)


def _leave_dock(nodes, zones, members: list, what: str, extra: float) -> Link:
    if len(members) != 2:
        raise FleetMapError(f"{what}: leave_dock is for a pair of nodes, got {len(members)}")
    dock, out = (nodes[name] for name in members)
    if dock.kind != AT_DOCK:
        raise FleetMapError(f"{what}: leave_dock starts at {dock.id}, which is not a dock")
    if zones[out.zone].kind != SHARED:
        raise FleetMapError(f"{what}: {out.id} is not in a shared zone")
    link = _link(nodes, zones, dock.id, out.id, NAV_GOAL, "", extra_m=extra)
    for zone in link.zones[1:]:
        if zones[zone].kind != SHARED:
            raise FleetMapError(f"{what}: the line from {dock.id} to {out.id} crosses {zone}, "
                                "which is not shared")
    return link


# ---- nav 그룹의 지정 구역 검사 ----
def _check_named_zones(what: str, members: list, crossed, nodes, zones) -> None:
    if len(members) != 2:
        raise FleetMapError(f"{what}: `zones:` is for a pair of nodes, got {len(members)}")
    for zone in crossed:
        if zone not in zones:
            raise FleetMapError(f"{what}: zones names {zone}, which is not a zone")
        if zones[zone].kind != SHARED:
            raise FleetMapError(f"{what}: zones may only name shared zones, {zone} is not")
    first, last = nodes[members[0]].zone, nodes[members[1]].zone
    if not crossed or crossed[0] != first or crossed[-1] != last:
        raise FleetMapError(f"{what}: zones must run from {first} to {last}")


# ---- 스테이션 읽기와 검사 ----
def read_stations(raw: dict, nodes: dict[str, Node]) -> dict[str, Station]:
    stations: dict[str, Station] = {}
    for name, entry in (raw or {}).items():
        dock = entry.get("dock")
        if dock not in nodes or nodes[dock].kind != AT_DOCK:
            raise FleetMapError(f"station {name}: dock {dock!r} is not a dock node")
        queue = tuple(entry.get("queue") or ())
        for spot in queue:
            if spot not in nodes or nodes[spot].kind not in (WAIT, PARKING):
                raise FleetMapError(f"station {name}: queue spot {spot!r} is not a wait or "
                                    "parking node")
        if entry.get("action") not in (FILL, SWAP):
            raise FleetMapError(f"station {name}: action must be fill or swap")
        line = entry.get("line", False)
        if not isinstance(line, bool):
            raise FleetMapError(f"station {name}: line must be true or false")
        stations[str(name)] = Station(str(name), dock, queue, str(entry.get("server", "")),
                                      tuple(entry.get("arms") or ()), entry["action"], line)
    actions = sorted(station.action for station in stations.values())
    if actions != [FILL, SWAP]:
        raise FleetMapError("the map needs exactly one fill station and one swap station, got "
                            f"{actions}")
    return stations


# ---- 주차 노드 목록 읽기와 검사 ----
def read_parking(raw, nodes: dict[str, Node]) -> tuple[str, ...]:
    parking = tuple(raw or ())
    marked = {name for name, node in nodes.items() if node.kind == PARKING}
    if not parking or set(parking) != marked or len(parking) != len(set(parking)):
        raise FleetMapError(f"parking must list every parking node once: nodes {sorted(marked)}, "
                            f"list {list(parking)}")
    return parking
