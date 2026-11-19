# ---- route 자료형, 경로 계산, 이동 명령 만들기 ----
from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise

from fleet_core.common.commands import MoveCmd, RetreatCmd
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, SHARED, Edge, distance

ON_NODE_M = 0.03


# ---- edge 목록과 진행 위치를 담은 route ----
@dataclass
class Route:
    goal: str
    edges: list
    done: int = 0
    granted: int = 0
    sent: int = 0
    stale: bool = False
    turn: bool = False
    standby: bool = False

    @property
    def finished(self) -> bool:
        return self.done >= len(self.edges)

    def nodes_left(self) -> list[str]:
        return [edge.b for edge in self.edges[self.done:]]


# ---- 멈출 수 있는 곳, 필요한 구역과 노드, 같은 방식 구간 끝 찾기 ----
def stretch_end(site, edges, start: int) -> int:
    for index in range(start, len(edges)):
        if site.can_wait(edges[index].b):
            return index + 1
    return len(edges)


def needs(edges) -> tuple[list, list, dict]:
    zones, nodes, lanes = [], [], {}
    for edge in edges:
        zones.extend(zone for zone in edge.zones if zone not in zones)
        if edge.lane:
            lanes.setdefault(edge.lane, edge.a)
        if edge.b not in nodes:
            nodes.append(edge.b)
    return zones, nodes, lanes


def joins(site, a, b) -> bool:
    return (a.mode == b.mode == NAV_GOAL and len(a.zones) == 1 and a.zones == b.zones
            and site.zones[a.zones[0]].kind == SHARED and site.nodes[a.b].through)


def _shortest(site, edges) -> bool:
    return bool(edges[0].a) and (
        sum(edge.length for edge in edges) <= distance(site, edges[0].a, edges[-1].b) + 1e-6)


def segment_end(site, edges, start: int, limit: int) -> int:
    mode = edges[start].mode
    end = start + 1
    if mode != NAV_GOAL:
        while end < limit and edges[end].mode == mode:
            end += 1
        return end
    while (end < limit and joins(site, edges[end - 1], edges[end])
           and _shortest(site, edges[start:end + 1])):
        end += 1
    return end


# ---- edge 위 진행 거리와 지나감 판정 ----
def heading(a, b) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _pieces(edge) -> list:
    return [(a, b) for a, b in pairwise(edge.points)
            if math.dist(a, b) > 0.0]


def along(edge, x: float, y: float) -> float:
    best, best_gap, start = 0.0, math.inf, 0.0
    for (ax, ay), (bx, by) in _pieces(edge):
        span = math.dist((ax, ay), (bx, by))
        t = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / (span * span)
        t = min(max(t, 0.0), 1.0)
        gap = math.hypot(x - ax - t * (bx - ax), y - ay - t * (by - ay))
        if gap < best_gap - 1e-12:
            best, best_gap = start + t * span, gap
        start += span
    return min(max(best, 0.0), edge.length)


def passed(edge, x: float, y: float) -> bool:
    return along(edge, x, y) >= edge.length - ON_NODE_M


# ---- 현재 위치부터 이어지는 점과 방향 ----
def _from_here(edge, x: float, y: float) -> tuple[list, list]:
    pieces = _pieces(edge)
    s = along(edge, x, y)
    start, first, point = 0.0, 0, pieces[0][0]
    if s > ON_NODE_M:
        for index, (a, b) in enumerate(pieces):
            span = math.dist(a, b)
            if s < start + span - 1e-9 or index == len(pieces) - 1:
                t = min((s - start) / span, 1.0)
                first, point = index, (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
                break
            start += span
    rest = pieces[first:]
    return [point, *(b for _, b in rest)], [heading(a, b) for a, b in rest]


def _line(edges, x: float, y: float) -> tuple[list, list]:
    points, yaws = _from_here(edges[0], x, y)
    for edge in edges[1:]:
        for a, b in _pieces(edge):
            points.append(b)
            yaws.append(heading(a, b))
    return points, yaws


# ---- 이동 명령과 물러나기 명령 만들기 ----
def move_command(site, robot: str, goal_id: int, edges, x: float, y: float,
                 after=None, priority: int = -1) -> MoveCmd:
    node = site.nodes[edges[-1].b]
    if edges[0].mode == NAV_GOAL:
        if node.yaw is not None:
            yaw = node.yaw
        elif after is not None and after.mode == FOLLOW_PATH:
            yaw = heading(*_pieces(after)[0])
        else:
            yaw = heading(edges[-1].points[-2] if len(edges) > 1 else (x, y),
                          (node.x, node.y))
        return MoveCmd(robot, goal_id, node.id, node.x, node.y, yaw, (), True, NAV_GOAL,
                       priority)
    points, yaws = _line(edges, x, y)
    end_yaw = node.yaw if node.yaw is not None else yaws[-1]
    path = tuple((px, py, yaw) for (px, py), yaw in zip(points, [*yaws, end_yaw], strict=True))
    return MoveCmd(robot, goal_id, node.id, node.x, node.y, end_yaw, path, True, FOLLOW_PATH,
                   priority)


def retreat_command(params, robot: str, goal_id: int, edges, x: float, y: float) -> RetreatCmd:
    points, yaws = _line(edges, x, y)
    facing = [math.remainder(yaw + math.pi, math.tau) for yaw in yaws]
    path = tuple((px, py, yaw) for (px, py), yaw in zip(points, [facing[0], *facing],
                                                         strict=True))
    distance = sum(math.dist(a, b) for a, b in pairwise(points))
    return RetreatCmd(robot, goal_id, distance, params.retreat_speed_mps, path)


# ---- 현재 위치에서 노드로 가는 임시 edge ----
def free_edge(site, x: float, y: float, node: str, zones=()) -> Edge:
    end = site.nodes[node]
    points = ((x, y), (end.x, end.y))
    return Edge("", node, NAV_GOAL, "", points, tuple(zones) or (end.zone,), math.dist(*points))


def back_edge(site, x: float, y: float, node: str, zones, via: str = "", bends=()) -> Edge:
    corner = [site.node_pose(via)] if via else []
    end = site.nodes[node]
    points = ((x, y), *corner, *(tuple(p) for p in bends), (end.x, end.y))
    length = sum(math.dist(a, b) for a, b in pairwise(points))
    return Edge("", node, RETREAT, "", points, tuple(zones), length)


# ---- 속도와 남은 주행 시간 계산 ----
def speed(params, mode: str) -> float:
    if mode == NAV_GOAL:
        return params.nav_speed_mps
    return params.follow_speed_mps if mode == FOLLOW_PATH else params.retreat_speed_mps


def travel_s(params, edges) -> float:
    return sum(edge.length / speed(params, edge.mode) for edge in edges)


def left_s(params, edges, x: float, y: float) -> float:
    if not edges:
        return 0.0
    first = edges[0]
    here = (math.dist((x, y), first.points[-1]) if first.mode == NAV_GOAL
            else first.length - along(first, x, y))
    return here / speed(params, first.mode) + travel_s(params, edges[1:])
