# ---- route 자료형, 경로 계산, 이동 명령 만들기 ----
from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise

from fleet_core.common.commands import MoveCmd, RetreatCmd
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, SHARED, Edge, distance

ON_NODE_M = 0.03
# Nav2 goal 끝 보정의 뜻: 다음 조각으로 넘어갈 때까지, 끝 노드에 닿을 때까지, 끝에 설 때까지
PASS, ARRIVE, STOP = "pass", "arrive", "stop"


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


def _nearest(pieces, x: float, y: float) -> tuple[int, float]:
    best, best_gap, start = (0, 0.0), math.inf, 0.0
    for index, ((ax, ay), (bx, by)) in enumerate(pieces):
        span = math.dist((ax, ay), (bx, by))
        t = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / (span * span) if span else 0.0
        t = min(max(t, 0.0), 1.0)
        gap = math.hypot(x - ax - t * (bx - ax), y - ay - t * (by - ay))
        if gap < best_gap - 1e-12:
            best, best_gap = (index, start + t * span), gap
        start += span
    return best


def along(edge, x: float, y: float) -> float:
    _, best = _nearest(_pieces(edge), x, y)
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
# 길 비용(route 고르기)용 속도. 비율을 곱하지 않는다
def speed(params, mode: str) -> float:
    if mode == NAV_GOAL:
        return params.nav_speed_mps
    return params.follow_speed_mps if mode == FOLLOW_PATH else params.retreat_speed_mps


# ETA(남은 시간)용 속도. Nav2 goal만 nav_eta_speed_ratio를 곱한다
def eta_speed(params, mode: str) -> float:
    if mode == NAV_GOAL:
        return params.nav_speed_mps * params.nav_eta_speed_ratio
    return speed(params, mode)


def travel_s(params, edges) -> float:
    return sum(edge.length / eta_speed(params, edge.mode) for edge in edges)


def nav_stretch(edges) -> int:
    end = 1
    while end < len(edges) and edges[end].mode == NAV_GOAL and edges[end].a == edges[end - 1].b:
        end += 1
    return end


def left_s(params, edges, x: float, y: float) -> float:
    if not edges:
        return 0.0
    first = edges[0]
    if first.mode != NAV_GOAL:
        here = first.length - along(first, x, y)
        return here / eta_speed(params, first.mode) + travel_s(params, edges[1:])
    end = nav_stretch(edges)
    pieces = [(edge.points[0], edge.points[-1]) for edge in edges[:end]]
    index, _ = _nearest(pieces, x, y)
    here = math.dist((x, y), pieces[index][1])
    return here / eta_speed(params, NAV_GOAL) + travel_s(params, edges[index + 1:])


# ---- Nav2 goal 끝: 다음 조각으로 넘어가는 거리, 서서 끝 방향으로 도는 시간 ----
def nav_end_s(params, site, edges, end: int, x: float, y: float, yaw: float,
              ending: str = PASS) -> float:
    if ending == ARRIVE:
        return 0.0
    after = edges[end] if end < len(edges) else None
    if ending == PASS and after is not None and after.mode in (NAV_GOAL, FOLLOW_PATH):
        return -params.nav_pass_m / eta_speed(params, NAV_GOAL)
    if params.nav_turn_rps <= 0.0:
        return 0.0
    node = site.nodes[edges[end - 1].b]
    approach = heading(edges[end - 1].points[0], (node.x, node.y))
    if node.yaw is not None:
        end_yaw = node.yaw
    elif after is not None and after.mode == FOLLOW_PATH:
        end_yaw = heading(*_pieces(after)[0])
    else:
        end_yaw = approach
    at_end = math.dist((x, y), (node.x, node.y)) <= params.nav_goal_tolerance_m
    facing = yaw if at_end else approach
    return abs(math.remainder(end_yaw - facing, math.tau)) / params.nav_turn_rps
