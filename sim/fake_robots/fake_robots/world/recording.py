"""시험용 관제 대역: 받은 입력을 적고, 시험이 넣어 둔 명령을 tick()으로 내보낸다.

ZoneTraffic이 없어도 세계의 명령 실행과 보고를 볼 수 있게 한다. grants는 시험이 채운다.
"""
from __future__ import annotations


class FakeGrants:
    def __init__(self):
        self.zone_holders: dict[str, tuple[str, ...]] = {}
        self.robot_zones: dict[str, tuple[str, ...]] = {}
        self.robot_nodes: dict[str, tuple[str, ...]] = {}
        self.waits: dict[str, tuple[str, ...]] = {}
        self.directions: dict[str, str] = {}

    def give(self, robot: str, *zones: str) -> None:
        """robot이 zones를 쥔다(이미 쥔 것에 더한다)."""
        self.robot_zones[robot] = tuple(dict.fromkeys(self.robot_zones.get(robot, ()) + zones))
        for zone in zones:
            if robot not in self.zone_holders.get(zone, ()):
                self.zone_holders[zone] = self.zone_holders.get(zone, ()) + (robot,)

    def holders(self, zone):
        return self.zone_holders.get(zone, ())

    def held_by(self, robot):
        return self.robot_zones.get(robot, ())

    def node_holder(self, node):
        return next((r for r, nodes in self.robot_nodes.items() if node in nodes), "")

    def nodes_of(self, robot):
        return self.robot_nodes.get(robot, ())

    def lane_direction(self, zone):
        return self.directions.get(zone, "")

    def waiting(self):
        return dict(self.waits)


class _Traffic:
    def __init__(self):
        self.grants = FakeGrants()
        self.views: dict[str, dict] = {}

    def robot_view(self, robot):
        return self.views.get(robot, {})


class RecordingManager:
    def __init__(self, site, params, robots, clock):
        self.site, self.params, self.robots, self._clock = site, params, list(robots), clock
        self.traffic = _Traffic()
        self.calls: list[tuple] = []  # (시각, 메서드, 인자들)
        self.outbox: list = []

    def __getattr__(self, name):
        if not name.startswith("on_") and name not in ("confirm_clear", "reinstate"):
            raise AttributeError(name)

        def record(*args):
            self.calls.append((self._clock(), name, args))
            return [] if name == "confirm_clear" else True
        return record

    def tick(self) -> list:
        out, self.outbox = self.outbox, []
        return out

    def of(self, name: str, robot: str | None = None) -> list[tuple]:
        """그 메서드로 받은 (시각, 인자들). robot을 주면 첫 인자가 그 로봇인 것만."""
        return [(t, args) for t, n, args in self.calls
                if n == name and (robot is None or (args and args[0] == robot))]
