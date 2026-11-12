"""지도 파일의 구획을 기록으로 바꾼다. 구획 하나에 함수 하나다."""
from __future__ import annotations

import math

from .geometry import inside, overlap_area, sample, within
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

SAMPLE_M = 0.01  # 선이 지나는 구역을 이 간격으로 찾는다
LANE_CAPACITY = 2  # 한 줄 구역에 적지 않았을 때 같은 방향으로 함께 들어가는 대수


def read_rect(raw, what: str) -> tuple:
    try:
        x0, y0, x1, y1 = (float(v) for v in raw)
    except (TypeError, ValueError) as error:
        raise FleetMapError(f"{what}: a rectangle is [x0, y0, x1, y1], got {raw!r}") from error
    if not (x0 < x1 and y0 < y1):
        raise FleetMapError(f"{what}: rectangle {raw!r} has no area")
    return (x0, y0, x1, y1)


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


def zone_under(zones: dict[str, Zone], x: float, y: float) -> list[str]:
    return [zone.id for zone in zones.values() if any(inside(r, x, y) for r in zone.rects)]


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
        if kind in (WAIT, PARKING, FORK) and zone.kind != SHARED:
            raise FleetMapError(f"node {name} ({kind}) must stand in a shared zone, not {zone.id}")
        if kind == PASS and zone.kind == SHARED:
            raise FleetMapError(f"node {name} (pass) belongs in a lane or dock zone, not {zone.id}")
        yaw = entry.get("yaw_deg")
        nodes[str(name)] = Node(str(name), x, y, kind, zone.id,
                                None if yaw is None else math.radians(float(yaw)))
    return nodes


def zones_along(zones: dict[str, Zone], points) -> tuple[str, ...]:
    """선이 지나는 구역을 처음 만나는 순서로. 구역이 없는 자리(진열대)는 건너뛴다."""
    seen: list[str] = []
    for a, b in zip(points, points[1:], strict=False):
        for x, y in sample(a, b, SAMPLE_M):
            for zone in zone_under(zones, x, y):
                if zone not in seen:
                    seen.append(zone)
    return tuple(seen)


def _link(nodes, zones, a: str, b: str, via: str, path: str, crossed=None, bends=()) -> Link:
    points = ((nodes[a].x, nodes[a].y), *bends, (nodes[b].x, nodes[b].y))
    crossed = tuple(crossed) if crossed else zones_along(zones, points)
    # 끝 지점의 구역이 처음과 끝에 오게 한다: 경계 위의 표본이 순서를 바꿀 수 있다
    middle = [z for z in crossed if z not in (nodes[a].zone, nodes[b].zone)]
    ordered = [nodes[a].zone, *middle]
    if nodes[b].zone != nodes[a].zone:
        ordered.append(nodes[b].zone)
    lane = next((z for z in ordered if zones[z].kind == LANE), "")
    length = sum(math.dist(p, q) for p, q in zip(points, points[1:], strict=False))
    return Link(a, b, via, path, points, tuple(ordered), length, lane)


def _path_stops(entry, nodes, what: str) -> list[tuple[str, tuple]]:
    """고정 경로의 점 목록을 (지점, 그 지점 앞의 꺾는 점들)로 바꾼다.

    꺾는 점은 [x, y]이고 지점이 아니다. 두 지점 사이의 선이 꺾이는 곳일 뿐이라 처음과 끝은
    지점이어야 한다.
    """
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
                link = _link(nodes, zones, a, b, NAV_GOAL, "", named)
                for zone in link.zones:
                    if zones[zone].kind != SHARED:
                        raise FleetMapError(
                            f"{what}: the line from {a} to {b} crosses {zone}, which is not "
                            "shared; write the zones it may use as `zones:`")
                add(link, what)
    return links


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
        stations[str(name)] = Station(str(name), dock, queue, str(entry.get("server", "")),
                                      tuple(entry.get("arms") or ()), entry["action"])
    actions = sorted(station.action for station in stations.values())
    if actions != [FILL, SWAP]:
        raise FleetMapError("the map needs exactly one fill station and one swap station, got "
                            f"{actions}")
    return stations


def read_parking(raw, nodes: dict[str, Node]) -> tuple[str, ...]:
    parking = tuple(raw or ())
    marked = {name for name, node in nodes.items() if node.kind == PARKING}
    if not parking or set(parking) != marked or len(parking) != len(set(parking)):
        raise FleetMapError(f"parking must list every parking node once: nodes {sorted(marked)}, "
                            f"list {list(parking)}")
    return parking
