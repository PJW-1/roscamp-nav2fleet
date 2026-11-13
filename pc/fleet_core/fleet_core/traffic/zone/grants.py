from __future__ import annotations

from fleet_core.fleet_map import LANE


class ZoneGrants:
    def __init__(self, site, clock, follow_gap_s: float = 0.0):
        self._site = site
        self._clock = clock
        self._gap = follow_gap_s
        self._holders: dict[str, dict[str, float]] = {zone: {} for zone in site.zones}
        self._nodes: dict[str, str] = {node: "" for node in site.nodes}
        self._direction: dict[str, str] = {}
        self._held: dict[str, list[str]] = {}
        self._held_nodes: dict[str, list[str]] = {}
        self._waiting: dict[str, tuple[str, ...]] = {}

    def blockers(self, robot: str, zones=(), nodes=(), lanes=None) -> dict[str, tuple[str, ...]]:
        found: dict[str, tuple[str, ...]] = {}
        for zone in zones:
            others = self._others_in_the_way(robot, zone, (lanes or {}).get(zone))
            if others:
                found[zone] = others
        for node in nodes:
            holder = self._nodes[node]
            if holder and holder != robot:
                found[node] = (holder,)
        return found

    def _others_in_the_way(self, robot: str, zone: str, direction) -> tuple[str, ...]:
        holders = self._holders[zone]
        if robot in holders:
            return ()
        spec = self._site.zones[zone]
        if spec.kind == LANE:
            if direction is None:
                raise ValueError(f"lane zone {zone} needs a direction")
            if not holders:
                return ()
            if self._direction.get(zone) != direction or len(holders) >= spec.capacity:
                return tuple(holders)
            last = max(holders, key=holders.get)
            return (last,) if self._clock() - holders[last] < self._gap else ()
        return tuple(holders) if len(holders) >= spec.capacity else ()

    def take(self, robot: str, zones=(), nodes=(), lanes=None) -> bool:
        zones, nodes = list(zones), list(nodes)
        blocked = self.blockers(robot, zones, nodes, lanes)
        if blocked:
            self._waiting[robot] = tuple(blocked)
            return False
        self._waiting.pop(robot, None)
        self._grant(robot, zones, nodes, lanes)
        return True

    def force(self, robot: str, zones=(), nodes=()) -> bool:
        zones, nodes = list(zones), list(nodes)
        for zone in zones:
            holders = self._holders[zone]
            if robot not in holders and len(holders) >= self._site.zones[zone].capacity:
                return False
        if any(self._nodes[node] not in ("", robot) for node in nodes):
            return False
        self._grant(robot, zones, nodes, None)
        return True

    def _grant(self, robot: str, zones, nodes, lanes) -> None:
        now = self._clock()
        held = self._held.setdefault(robot, [])
        for zone in zones:
            if robot not in self._holders[zone]:
                self._holders[zone][robot] = now
                held.append(zone)
                if lanes and zone in lanes:
                    self._direction[zone] = lanes[zone]
        mine = self._held_nodes.setdefault(robot, [])
        for node in nodes:
            if self._nodes[node] != robot:
                self._nodes[node] = robot
                mine.append(node)

    def stop_waiting(self, robot: str) -> None:
        self._waiting.pop(robot, None)

    def note_waiting(self, robot: str, items) -> None:
        if items:
            self._waiting[robot] = tuple(items)
        else:
            self._waiting.pop(robot, None)

    def release_zone(self, robot: str, zone: str) -> bool:
        if robot not in self._holders[zone]:
            return False
        del self._holders[zone][robot]
        self._held[robot].remove(zone)
        if not self._holders[zone]:
            self._direction.pop(zone, None)
        return True

    def release_node(self, robot: str, node: str) -> bool:
        if self._nodes[node] != robot:
            return False
        self._nodes[node] = ""
        self._held_nodes[robot].remove(node)
        return True

    def holders(self, zone: str) -> tuple[str, ...]:
        return tuple(self._holders[zone])

    def held_by(self, robot: str) -> tuple[str, ...]:
        return tuple(self._held.get(robot, ()))

    def node_holder(self, node: str) -> str:
        return self._nodes[node]

    def nodes_of(self, robot: str) -> tuple[str, ...]:
        return tuple(self._held_nodes.get(robot, ()))

    def lane_direction(self, zone: str) -> str:
        return self._direction.get(zone, "")

    def waiting(self) -> dict[str, tuple[str, ...]]:
        return dict(self._waiting)

    def acquired_at(self, zone: str) -> float | None:
        times = self._holders[zone].values()
        return min(times) if times else None
