"""Pode 세 대 Gazebo 실행의 주행 검사. ROS 없이 돈다(노드는 fleet_check_node).
실제 위치, 로봇 상태, 관제 구역 보유자를 받아 로봇 간 최근접 거리, 정지 횟수, 구역 이탈, 시간비, keepout 침입 깊이를 쌓는다.
"""
from __future__ import annotations

import json
import math
from collections import deque

from fleet_core.fleet_map.model import SHARED

EDGE_STEP_M = 0.005  # 몸 윤곽을 이 간격으로 점검

STILL_MPS = 0.02
STILL_RADPS = 0.1
MOVING_MPS = 0.05
STOP_MIN_S = 0.5
SYNC_S = 0.1
KEEP_S = 1.0  # 늦게 처리한 표본도 짝을 만나도록 남겨 두는 기록 길이
MARGIN_M = 0.0
LEFT_S = 5.0  # 이 시간 안에 보유한 구역은 떠나는 중이라 침입이 아니다
MOVING = "MOVING"


def footprint_of(nav_params: dict) -> list[tuple[float, float]] | None:
    """Nav2가 쓰는 몸 윤곽(Nav2 파라미터 파일의 local costmap 발자국)."""
    try:
        text = nav_params["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    except KeyError:
        return None
    return [(float(x), float(y)) for x, y in json.loads(text)]


def outline_points(body, step: float = EDGE_STEP_M) -> list[tuple[float, float]]:
    """닫힌 윤곽을 step 이하 간격 점으로 만든다(꼭짓점 포함)."""
    points = []
    for (ax, ay), (bx, by) in zip(body, [*body[1:], body[0]], strict=True):
        count = max(1, math.ceil(math.dist((ax, ay), (bx, by)) / step))
        points += [(ax + (bx - ax) * k / count, ay + (by - ay) * k / count) for k in range(count)]
    return points


def _depth(rects, x: float, y: float) -> float:
    """점이 사각형 안으로 얼마나 깊은지(밖이면 0)."""
    return max((min(x - x0, x1 - x, y - y0, y1 - y) for x0, y0, x1, y1 in rects
                if x0 < x < x1 and y0 < y < y1), default=0.0)


class _Robot:
    def __init__(self):
        self.recent: deque[tuple[float, float, float]] = deque()  # (sim t, x, y), 최신이 뒤
        self.state = ""
        self.moved = False  # 마지막 정지 뒤로 움직였는가
        self.still_since: float | None = None
        self.still_state = ""
        self.stops = 0
        self.stops_while_moving = 0
        self.zone_samples = 0
        self.outside = 0
        self.no_zone = 0
        self.worst_outside_m = 0.0
        self.held_at: dict[str, float] = {}  # 구역 -> 그 구역을 가진 마지막 표본의 sim t
        self.keepout_checked = 0  # yard 안에서 방향이 있는 표본 수(깊이를 잼)
        self.keepout_samples = 0  # 그중 몸이 keepout 구역 안에 든 표본 수
        self.keepout: tuple[float, str, float] = (0.0, "", 0.0)  # (depth m, zone, sim t)


class FleetCheck:
    def __init__(self, site, robots, *, stop_min_s: float = STOP_MIN_S, sync_s: float = SYNC_S,
                 margin_m: float = MARGIN_M, body=None, left_s: float = LEFT_S):
        self.site = site
        # 로봇 틀의 윤곽. 없으면 None(깊이 안 잼)
        self.body = outline_points(body) if body else None
        self.reach = max((math.hypot(x, y) for x, y in body), default=0.0) if body else 0.0
        self.left_s = left_s
        self.yards = {zone.id for zone in site.zones.values() if zone.kind == SHARED}
        self.now = -math.inf  # 가장 새 표본의 sim t
        self.robots = {robot: _Robot() for robot in robots}
        self.stop_min_s = stop_min_s
        self.sync_s = sync_s
        self.margin_m = margin_m
        self.holders: dict[str, tuple[str, ...]] | None = None
        self.min_gap: tuple[float, float, str, str] | None = None  # (m, sim t, one, other)
        self.first: tuple[float, float] | None = None  # (sim t, wall t)
        self.last: tuple[float, float] | None = None

    def on_state(self, robot: str, state: str) -> None:
        if robot in self.robots:
            self.robots[robot].state = state

    def on_zones(self, holders: dict[str, str]) -> None:
        """/fleet/zones의 구역 -> 보유자(여럿이면 쉼표로 이음)."""
        new = {zone: tuple(name for name in holder.split(",") if name)
               for zone, holder in holders.items()}
        # 두 표본 사이에 잡았다 놓은 구역도 방금 가진 것으로 본다
        for held in (self.holders or {}, new):
            for zone, names in held.items():
                for name in names:
                    if name in self.robots:
                        self.robots[name].held_at[zone] = self.now
        self.holders = new

    def on_truth(self, robot: str, t: float, x: float, y: float, speed: float, turn: float,
                 wall: float | None = None, yaw: float | None = None) -> None:
        me = self.robots.get(robot)
        if me is None:
            return
        self.now = max(self.now, t)
        me.recent.append((t, x, y))
        while me.recent and me.recent[0][0] < me.recent[-1][0] - KEEP_S - self.sync_s:
            me.recent.popleft()
        if wall is not None:
            self.first = self.first or (t, wall)
            self.last = (t, wall)
        self._gaps(robot, t, x, y)
        self._stop(me, t, speed, turn)
        self._zones(robot, me, x, y)
        self._keepout(robot, me, t, x, y, yaw)

    def _gaps(self, robot: str, t: float, x: float, y: float) -> None:
        """sync_s 안에 있는 다른 로봇의 최근 표본 전부."""
        for other, them in self.robots.items():
            if other == robot:
                continue
            for t_other, ox, oy in them.recent:
                if abs(t_other - t) > self.sync_s:
                    continue
                gap = math.dist((x, y), (ox, oy))
                if self.min_gap is None or gap < self.min_gap[0]:
                    self.min_gap = (gap, max(t, t_other), *sorted((robot, other)))

    def _stop(self, me: _Robot, t: float, speed: float, turn: float) -> None:
        if speed > MOVING_MPS:
            me.moved = True
        if speed >= STILL_MPS or abs(turn) >= STILL_RADPS:
            me.still_since = None
            return
        if me.still_since is None:
            me.still_since, me.still_state = t, me.state
        if me.moved and t - me.still_since >= self.stop_min_s - 1e-9:
            me.moved = False
            me.stops += 1
            if me.still_state == MOVING:
                me.stops_while_moving += 1

    def _zones(self, robot: str, me: _Robot, x: float, y: float) -> None:
        if self.holders is None:
            return
        me.zone_samples += 1
        held = [zone for zone, holders in self.holders.items() if robot in holders]
        if not held:
            me.no_zone += 1
            return
        away = min(self.site.distance_to_zone(x, y, zone) for zone in held)
        if away > self.margin_m:
            me.outside += 1
            me.worst_outside_m = max(me.worst_outside_m, away)

    def _keepout(self, robot: str, me: _Robot, t: float, x: float, y: float, yaw) -> None:
        if self.holders is None or self.body is None or yaw is None:
            return
        for zone, holders in self.holders.items():
            if robot in holders:
                me.held_at[zone] = t
        if self.site.zone_at(x, y) not in self.yards:
            return
        me.keepout_checked += 1
        near = [zone for zone in self.site.zones.values()
                if zone.id not in self.yards
                and t - me.held_at.get(zone.id, -math.inf) > self.left_s
                and self.site.distance_to_zone(x, y, zone.id) < self.reach]
        if not near:
            return
        c, s = math.cos(yaw), math.sin(yaw)
        outline = [(x + c * bx - s * by, y + s * bx + c * by) for bx, by in self.body]
        deep, where = 0.0, ""
        for zone in near:
            depth = max(_depth(zone.rects, px, py) for px, py in outline)
            if depth > deep:
                deep, where = depth, zone.id
        if deep > 0.0:
            me.keepout_samples += 1
            if deep > me.keepout[0]:
                me.keepout = (deep, where, t)

    def summary(self) -> dict:
        # 깊이는 mm로 반올림하지 않는다(0.0204가 0.02 한도를 통과하면 안 된다). None은 못 잼
        measured = any(r.keepout_checked for r in self.robots.values())
        robots = {name: {"stops": r.stops, "stops_while_moving": r.stops_while_moving,
                         "outside_held_samples": r.outside, "no_zone_held_samples": r.no_zone,
                         "zone_samples": r.zone_samples,
                         "worst_outside_m": round(r.worst_outside_m, 3),
                         "keepout_depth_m": round(r.keepout[0], 6) if r.keepout_checked else None,
                         "keepout_samples": r.keepout_samples,
                         "keepout_checked_samples": r.keepout_checked}
                  for name, r in self.robots.items()}
        worst = max(self.robots, key=lambda name: self.robots[name].keepout[0], default=None)
        deep = self.robots[worst].keepout if worst is not None else (0.0, "", 0.0)
        sim_span = wall_span = rtf = None
        if self.first and self.last:
            sim_span = self.last[0] - self.first[0]
            wall_span = self.last[1] - self.first[1]
            rtf = round(sim_span / wall_span, 3) if wall_span > 0 else None
        gap = self.min_gap
        return {
            "min_distance_m": None if gap is None else round(gap[0], 3),
            "min_distance_pair": None if gap is None else [gap[2], gap[3]],
            "min_distance_at_s": None if gap is None else round(gap[1], 2),
            "stops": sum(r.stops for r in self.robots.values()),
            "stops_while_moving": sum(r.stops_while_moving for r in self.robots.values()),
            "outside_held_samples": sum(r.outside for r in self.robots.values()),
            "no_zone_held_samples": sum(r.no_zone for r in self.robots.values()),
            "zone_samples": sum(r.zone_samples for r in self.robots.values()),
            "sim_span_s": None if sim_span is None else round(sim_span, 2),
            "wall_span_s": None if wall_span is None else round(wall_span, 2),
            "real_time_factor": rtf,
            "keepout_depth_m": round(deep[0], 6) if measured else None,
            "keepout_zone": deep[1] if deep[0] > 0.0 else None,
            "keepout_robot": worst if deep[0] > 0.0 else None,
            "keepout_at_s": round(deep[2], 2) if deep[0] > 0.0 else None,
            "robots": robots,
        }
