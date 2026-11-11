"""밖에서 읽는 것: 로봇 목록, /fleet/zones, 로봇마다의 상태."""
from __future__ import annotations

from fleet_core.common.commands import ZoneView
from fleet_core.traffic.mission import _COMM_REASON, NO_FAULT, OUT_OF_SERVICE, RUN


class Views:
    @property
    def robots(self) -> list[str]:
        return list(self._robots)

    @property
    def everyone_placed(self) -> bool:
        return all(m.placed or m.fault == OUT_OF_SERVICE or m.name in self._joining
                   for m in self._robots.values())

    @property
    def running(self) -> bool:
        return self._run_mode == RUN

    @property
    def zones(self):
        return self.grants

    def available(self, robot) -> bool:
        m = self._robots.get(robot)
        return m is not None and m.available

    def take_output(self) -> list:
        out, self._out = self._out, []
        return out

    def zones_view(self) -> list[ZoneView]:
        """구역마다 쥔 로봇(여럿이면 이름 순서로 쉼표로 잇는다)과 그 구역을 기다리는 로봇."""
        views = []
        for zone in self.site.zones:
            holders = ",".join(sorted(self.grants.holders(zone)))
            queue = tuple(sorted(m.name for m in self._robots.values() if zone in m.wait_for))
            views.append(ZoneView(zone, holders, queue, self.grants.acquired_at(zone)))
        return views

    def robot_view(self, robot) -> dict:
        m = self._robots.get(robot)
        if m is None:
            return {}
        route, granted = self._ahead(m)
        return {
            "state": m.state, "phase": m.phase, "on_map": m.placed, "fault": m.fault,
            "comm": m.comm, "called": m.called, "charging": m.charge,
            "held": list(self.grants.held_by(robot)), "nodes": list(self.grants.nodes_of(robot)),
            "node": m.node, "waypoint": m.node, "target": m.goal.end if m.goal else "",
            "route": route, "path": route, "base": route[:granted], "horizon": route[granted:],
            "destination": m.dest, "goal_id": m.goal.id if m.goal else None,
            "attempts": m.attempts, "blocked_lane": None, "x": m.x, "y": m.y,
        }

    @staticmethod
    def _ahead(m) -> tuple[list[str], int]:
        """앞에 남은 지점들과, 그 가운데 구역을 허락받은 수."""
        trip = m.trip
        if trip is None:
            return [], 0
        return trip.route(), trip.granted - trip.done

    def status_view(self, robot) -> dict:
        m = self._robots.get(robot)
        if m is None:
            return {}
        route, granted = self._ahead(m)
        want = self._wants.get(robot)
        blockers = (self.grants.blockers(robot, want.zones, want.nodes, want.lanes)
                    if want is not None else {})
        return {
            "fsm_state": m.robot_state,
            "fault": m.fault or m.comm or NO_FAULT,
            "fault_reason": m.fault_reason or _COMM_REASON.get(m.comm, ""),
            "wait_zone": m.wait_for[0] if m.wait_for else "",
            "wait_robots": tuple(sorted({name for group in blockers.values() for name in group})),
            "battery": m.battery, "x": m.x, "y": m.y, "yaw": m.yaw,
            "base": tuple(route[:granted]), "horizon": tuple(route[granted:]),
        }
