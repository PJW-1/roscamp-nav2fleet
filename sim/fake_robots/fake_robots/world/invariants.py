"""TRAFFIC_CONTRACT 7절의 불변식을 매 걸음 본다. 예외를 내지 않고 위반을 모은다.

교통 층의 grants(holders, held_by, nodes_of ...)와 로봇의 실제 자리를 쓴다. 같은 위반이 이어지면
처음 한 번만 적고, 풀렸다가 다시 생기면 또 적는다.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

from fleet_core.fleet_map import DOCK, LANE, PASS, SHARED
from fleet_core.fleet_map.geometry import sample

INSIDE_TOL_M = 0.01  # 3번: 쥔 구역 밖 허용
STILL_S = 5.0  # 5번: 한 줄 구역이나 갈림목에 이만큼 넘게 서 있으면 위반
STILL_M = 0.005  # 한 걸음에 이보다 덜 움직이면 서 있다
CAPACITY, LANE_WAY, OUTSIDE, COMMAND, STANDING, TOUCH = (
    "I1_CAPACITY", "I2_LANE_DIRECTION", "I3_OUTSIDE_HELD", "I4_COMMAND", "I5_STANDING",
    "I6_TOO_CLOSE")


@dataclass(frozen=True)
class Violation:
    t: float
    code: str
    detail: str
    robot: str = ""


class Invariants:
    def __init__(self, site, params):
        self.site, self.params = site, params
        self.violations: list[Violation] = []
        self._active: set = set()
        self._now: set = set()
        self._prev: dict[str, tuple[float, float]] = {}
        self._still: dict[str, float] = {}
        # 갈림목: pass 지점이 있는 구역
        self._junctions = {n.zone for n in site.nodes.values() if n.kind == PASS}
        self._lanes = [z for z in site.zones.values() if z.kind == LANE]

    def _flag(self, t, code, key, detail, robot="") -> None:
        self._now.add((code, key))
        if (code, key) not in self._active:
            self.violations.append(Violation(t, code, detail, robot))

    def check(self, t, grants, robots, views, quiet) -> None:
        """robots: 이름 -> SimRobot(바닥 위만). views: 이름 -> robot_view(없으면 {}).
        quiet: 이름 -> 5번을 따지지 않을 이유가 있다(BLOCKED, 끊김)."""
        self._now = set()
        if grants is not None:
            self._capacity(t, grants)
            self._inside_held(t, grants, robots, views)
        self._lane_direction(t, robots)
        self._standing(t, robots, views, quiet)
        self._touch(t, robots)
        self._active = self._now
        self._prev = {name: r.pose for name, r in robots.items()}

    def _capacity(self, t, grants) -> None:
        for zone in self.site.zones.values():
            holders = grants.holders(zone.id)
            cap = 1 if zone.kind == DOCK else zone.capacity
            if len(holders) > cap:
                self._flag(t, CAPACITY, zone.id, f"{zone.id} held by {holders} (capacity {cap})")

    def _inside_held(self, t, grants, robots, views) -> None:
        for name, r in robots.items():
            view = views.get(name) or {}
            if (t < r.localizing_until or not view.get("on_map", True)
                    or view.get("fault") == "OUT_OF_SERVICE"):
                continue
            held = grants.held_by(name)
            gap = min((self.site.distance_to_zone(r.x, r.y, z) for z in held), default=math.inf)
            if gap > INSIDE_TOL_M:
                where = self.site.zone_at(r.x, r.y)
                self._flag(t, OUTSIDE, name, f"{name} at ({r.x:.3f}, {r.y:.3f}) in {where}, "
                           f"holds {held}", name)

    def _lane_direction(self, t, robots) -> None:
        for lane in self._lanes:
            rect = lane.rects[0]
            along = 0 if rect[2] - rect[0] >= rect[3] - rect[1] else 1
            ways = {}
            for name, r in robots.items():
                if name in self._prev and self.site.distance_to_zone(r.x, r.y, lane.id) <= 0.0:
                    moved = r.pose[along] - self._prev[name][along]
                    if abs(moved) > 1e-6:
                        ways[name] = moved > 0
            if len(set(ways.values())) > 1:
                self._flag(t, LANE_WAY, lane.id, f"{lane.id}: {ways}")

    def _standing(self, t, robots, views, quiet) -> None:
        for name, r in robots.items():
            zone = self.site.zone_at(r.x, r.y)
            kind = self.site.zones[zone].kind if zone else SHARED
            view = views.get(name) or {}
            excused = quiet.get(name) or view.get("fault") or view.get("comm")
            moved = math.dist(r.pose, self._prev.get(name, r.pose))
            if not (kind == LANE or zone in self._junctions) or excused or moved > STILL_M:
                self._still.pop(name, None)
                continue
            since = self._still.setdefault(name, t)
            if t - since > STILL_S:
                self._flag(t, STANDING, name, f"{name} stands in {zone} since {since:.1f}", name)

    def _touch(self, t, robots) -> None:
        least = 2 * self.params.robot_radius_m
        for (a, ra), (b, rb) in itertools.combinations(sorted(robots.items()), 2):
            gap = ra.gap(rb)
            if gap < least - 1e-9:
                self._flag(t, TOUCH, (a, b), f"{a} and {b} {gap:.3f} m apart")

    # ---- 4번: 명령을 받을 때 ----

    def check_command(self, t, grants, cmd, mode) -> None:
        """mode: nav_goal, follow_path, retreat. grants가 없으면 보지 않는다."""
        if grants is None:
            return
        held = grants.held_by(cmd.robot)
        if mode == "nav_goal":
            node = self.site.nodes.get(cmd.target_id)
            zone = node.zone if node else self.site.zone_at(cmd.x, cmd.y)
            if zone is None or self.site.zones[zone].kind != SHARED or zone not in held:
                self.violations.append(Violation(
                    t, COMMAND, f"nav_goal {cmd.goal_id} to {cmd.target_id} in {zone}, "
                    f"holds {held}", cmd.robot))
            return
        points = [tuple(p[:2]) for p in cmd.path] or ([(cmd.x, cmd.y)] if mode != "retreat"
                                                       else [])
        for a, b in zip(points, points[1:], strict=False):
            for x, y in sample(a, b, 0.02):
                if min((self.site.distance_to_zone(x, y, z) for z in held),
                       default=math.inf) > INSIDE_TOL_M:
                    self.violations.append(Violation(
                        t, COMMAND, f"{mode} {cmd.goal_id} passes ({x:.2f}, {y:.2f}) "
                        f"outside {held}", cmd.robot))
                    return
