# ---- 관제 상태 조회 객체 ----
from __future__ import annotations

import math

from fleet_core.common.commands import ZoneView
from fleet_core.traffic.mission import _COMM_REASON, NO_FAULT, OUT_OF_SERVICE


class ZoneViews:
    def __init__(self, site, grants, robots, joining, wants, blocked, marks):
        self.site = site
        self.grants = grants
        self._robots = robots
        self._joining = joining
        self._wants = wants
        self._blocked = blocked
        self._marks = marks

    # ---- 모두 자리에 있는지 ----
    @property
    def everyone_placed(self) -> bool:
        return all(m.placed or m.health.fault == OUT_OF_SERVICE or m.name in self._joining
                   for m in self._robots.values())

    # ---- 막힘 표시 관측(읽기 전용 질의, bool만 돌려준다) ----
    def live_block(self, keys, point, radius: float, now: float) -> bool:
        if any(self._blocked.get(key, 0.0) > now for key in keys):
            return True
        return any(math.dist((mk.x, mk.y), point) <= radius for mk in self._marks.values())

    # ---- 구역별 주인과 대기 줄 보기 ----
    def zones_view(self) -> list[ZoneView]:
        views = []
        for zone in self.site.zones:
            holders = ",".join(sorted(self.grants.holders(zone)))
            queue = tuple(sorted(m.name for m in self._robots.values() if zone in m.watch.wait.key))
            views.append(ZoneView(zone, holders, queue, self.grants.acquired_at(zone)))
        return views

    # ---- 로봇 보기 ----
    def robot_view(self, robot) -> dict:
        m = self._robots.get(robot)
        if m is None:
            return {}
        nodes, granted = self._route_left(m)
        return {
            "state": m.state, "phase": m.phase, "on_map": m.placed, "fault": m.health.fault,
            "comm": m.health.comm, "called": m.called, "charging": m.report.charge,
            "held": list(self.grants.held_by(robot)), "nodes": list(self.grants.nodes_of(robot)),
            "node": m.node, "waypoint": m.node, "target": m.goal.end if m.goal else "",
            "route": nodes, "path": nodes, "base": nodes[:granted], "horizon": nodes[granted:],
            "destination": m.dest, "goal_id": m.goal.id if m.goal else None,
            "attempts": m.attempts, "blocked_lane": None, "x": m.x, "y": m.y,
        }

    # ---- 남은 경로 계산과 상태 보기 ----
    @staticmethod
    def _route_left(m) -> tuple[list[str], int]:
        route = m.route
        if route is None:
            return [], 0
        return route.nodes_left(), route.granted - route.done

    def status_view(self, robot) -> dict:
        m = self._robots.get(robot)
        if m is None:
            return {}
        nodes, granted = self._route_left(m)
        want = self._wants.get(robot)
        blockers = (self.grants.blockers(robot, want.zones, want.nodes, want.lanes)
                    if want is not None else {})
        return {
            "fsm_state": m.report.robot_state,
            "fault": m.health.fault or m.health.comm or NO_FAULT,
            "fault_reason": m.health.fault_reason or _COMM_REASON.get(m.health.comm, ""),
            "wait_zone": m.watch.wait.key[0] if m.watch.wait.key else "",
            "wait_robots": tuple(sorted({name for group in blockers.values() for name in group})),
            "battery": m.report.battery, "x": m.x, "y": m.y, "yaw": m.yaw,
            "base": tuple(nodes[:granted]), "horizon": tuple(nodes[granted:]),
            "priority": m.sent_priority,
        }
