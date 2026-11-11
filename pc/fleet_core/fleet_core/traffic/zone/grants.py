"""구역 허락 기록: 구역마다 정원까지의 로봇, 지점마다 로봇 하나, 한 줄 구역마다 방향 하나.

한 구간(지금 선 자리에서 다음에 설 자리까지)에 필요한 구역과 지점은 한꺼번에 잡는다. 하나라도
못 잡으면 아무것도 잡지 않는다(지시서 2-2의 2번). 누가 먼저인지는 여기서 정하지 않는다.
"""
from __future__ import annotations

from fleet_core.fleet_map import LANE


class ZoneGrants:
    def __init__(self, site, clock, follow_gap_s: float = 0.0):
        self._site = site
        self._clock = clock
        self._gap = follow_gap_s
        # 구역 -> {로봇: 잡은 시각}
        self._holders: dict[str, dict[str, float]] = {zone: {} for zone in site.zones}
        self._nodes: dict[str, str] = {node: "" for node in site.nodes}
        self._direction: dict[str, str] = {}  # 한 줄 구역 -> 지금 방향(들어간 지점)
        self._held: dict[str, list[str]] = {}  # 로봇 -> 잡은 순서대로의 구역
        self._held_nodes: dict[str, list[str]] = {}  # 로봇 -> 잡은 순서대로의 지점
        self._waiting: dict[str, tuple[str, ...]] = {}  # 로봇 -> 기다리는 구역과 지점

    # ---- 잡기 ----

    def blockers(self, robot: str, zones=(), nodes=(), lanes=None) -> dict[str, tuple[str, ...]]:
        """이 구간을 지금 못 잡게 하는 것: 구역이나 지점 -> 비켜야 할 로봇들. 비면 잡을 수 있다."""
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
        """구간을 통째로 잡는다. 못 잡으면 아무것도 잡지 않고, 기다리는 것을 적어 둔다."""
        zones, nodes = list(zones), list(nodes)
        blocked = self.blockers(robot, zones, nodes, lanes)
        if blocked:
            self._waiting[robot] = tuple(blocked)
            return False
        self._waiting.pop(robot, None)
        self._grant(robot, zones, nodes, lanes)
        return True

    def force(self, robot: str, zones=(), nodes=()) -> bool:
        """지도에 올릴 때: 자리가 있으면 잡는다. 한 줄 구역의 방향은 따지지 않는다."""
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

    # ---- 놓기 ----

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

    def release_all(self, robot: str) -> tuple[tuple[str, ...], tuple[str, ...]]:
        """로봇이 쥔 것을 모두 놓는다. 놓은 (구역들, 지점들)."""
        zones, nodes = self.held_by(robot), self.nodes_of(robot)
        for zone in zones:
            self.release_zone(robot, zone)
        for node in nodes:
            self.release_node(robot, node)
        self._waiting.pop(robot, None)
        return zones, nodes

    # ---- 보기 ----

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
