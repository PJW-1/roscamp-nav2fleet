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
SHARED_GRACE_S = 2.0  # 3번: 자리가 있는 공유 구역에 쥐지 않고 이만큼 넘게 있으면 위반
STILL_S = 5.0  # 5번: 한 줄 구역이나 갈림목에 이만큼 넘게 서 있으면 위반
STILL_M = 0.02  # 선 자리에서 이보다 덜 움직였으면 아직 서 있다
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
        self._still: dict[str, tuple[float, tuple[float, float]]] = {}  # 선 시각, 선 자리
        # 3번: 쥐지 않은 공유 구역에 있는 로봇 -> (구역, 자리가 있게 된 시각 또는 None)
        self._shared_since: dict[str, tuple[str, float | None]] = {}
        self.shared_outside = 0  # 쥐지 않은 공유 구역에 들어간 횟수("구역 밖" 지표)
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
        """한 줄 구역과 도크 구역은 엄격히 본다. 공유 구역은 Nav2 goal이 가로지를 수 있어
        (교통 층은 자리가 나면 그 구역을 잡는다) 들어간 횟수만 세고, 자리가 있는데도
        SHARED_GRACE_S 넘게 쥐지 않고 머물 때만 위반이다."""
        for name, r in robots.items():
            view = views.get(name) or {}
            if (t < r.localizing_until or not view.get("on_map", True)
                    or view.get("fault") == "OUT_OF_SERVICE"):
                self._shared_since.pop(name, None)
                continue
            held = grants.held_by(name)
            gap = min((self.site.distance_to_zone(r.x, r.y, z) for z in held), default=math.inf)
            where = self.site.zone_at(r.x, r.y)
            if gap <= INSIDE_TOL_M:
                self._shared_since.pop(name, None)
                continue
            text = f"{name} at ({r.x:.3f}, {r.y:.3f}) in {where}, holds {held}"
            if where is None or self.site.zones[where].kind != SHARED:
                self._shared_since.pop(name, None)
                self._flag(t, OUTSIDE, name, text, name)
                continue
            last = self._shared_since.get(name)
            if last is None or last[0] != where:
                self.shared_outside += 1
                last = (where, None)
            room = len(grants.holders(where)) < self.site.zones[where].capacity
            since = (t if last[1] is None else last[1]) if room else None
            self._shared_since[name] = (where, since)
            if since is not None and t - since > SHARED_GRACE_S:
                self._flag(t, OUTSIDE, name, f"{text}; {where} had room for "
                           f"{t - since:.1f} s", name)

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
            since, spot = self._still.get(name, (t, r.pose))
            if (not (kind == LANE or zone in self._junctions) or excused
                    or math.dist(r.pose, spot) > STILL_M):
                self._still.pop(name, None)
                if not excused and (kind == LANE or zone in self._junctions):
                    self._still[name] = (t, r.pose)  # 움직였다: 여기서 다시 잰다
                continue
            self._still[name] = (since, spot)
            if t - since > STILL_S:
                self._flag(t, STANDING, name, f"{name} stands in {zone} since {since:.1f}", name)

    def _touch(self, t, robots) -> None:
        least = 2 * self.params.robot_radius_m
        for (a, ra), (b, rb) in itertools.combinations(sorted(robots.items()), 2):
            gap = ra.gap(rb)
            if gap < least - 1e-9:
                self._flag(t, TOUCH, (a, b), f"{a} and {b} {gap:.3f} m apart")

    # ---- 4번: 명령을 받을 때 ----

    def check_command(self, t, grants, cmd, mode, start=None) -> None:
        """mode: nav_goal, follow_path, retreat. grants가 없으면 보지 않는다.
        start: 로봇의 지금 자리. 점이 둘보다 적은 path는 거기서 곧게 가는 것으로 본다."""
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
        points = [tuple(p[:2]) for p in cmd.path]
        if len(points) < 2 and start is not None:
            if mode == "retreat":
                yaw = start[2] if len(start) > 2 else 0.0
                points = [start[:2], (start[0] - math.cos(yaw) * cmd.distance_m,
                                      start[1] - math.sin(yaw) * cmd.distance_m)]
            else:
                points = [start[:2], (cmd.x, cmd.y)]
        for a, b in zip(points, points[1:], strict=False):
            for x, y in sample(a, b, 0.02):
                if min((self.site.distance_to_zone(x, y, z) for z in held),
                       default=math.inf) > INSIDE_TOL_M:
                    self.violations.append(Violation(
                        t, COMMAND, f"{mode} {cmd.goal_id} passes ({x:.2f}, {y:.2f}) "
                        f"outside {held}", cmd.robot))
                    return
