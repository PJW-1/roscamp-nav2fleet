# ---- 트립 자료형, 경로 계산, 이동 명령 만들기 ----
from __future__ import annotations

import math
from dataclasses import dataclass

from fleet_core.common.commands import MoveCmd, RetreatCmd
from fleet_core.fleet_map import AT_DOCK, FOLLOW_PATH, NAV_GOAL, RETREAT, SHARED, Link

ON_NODE_M = 0.03


# ---- 링크 목록과 진행 위치를 담은 트립 ----
@dataclass
class Trip:
    goal: str
    links: list
    done: int = 0
    granted: int = 0
    sent: int = 0
    stale: bool = False
    turn: bool = False

    @property
    def finished(self) -> bool:
        return self.done >= len(self.links)

    def route(self) -> list[str]:
        return [link.b for link in self.links[self.done:]]


# ---- 멈출 수 있는 곳, 필요한 구역과 노드, 같은 방식 구간 끝 찾기 ----
def stretch_end(site, links, start: int) -> int:
    for index in range(start, len(links)):
        if site.can_wait(links[index].b):
            return index + 1
    return len(links)


def needs(links) -> tuple[list, list, dict]:
    zones, nodes, lanes = [], [], {}
    for link in links:
        zones.extend(zone for zone in link.zones if zone not in zones)
        if link.lane:
            lanes.setdefault(link.lane, link.a)
        if link.b not in nodes:
            nodes.append(link.b)
    return zones, nodes, lanes


def joins(site, a, b) -> bool:
    return (a.via == b.via == NAV_GOAL and len(a.zones) == 1 and a.zones == b.zones
            and site.zones[a.zones[0]].kind == SHARED)


def segment_end(site, links, start: int, limit: int) -> int:
    via = links[start].via
    end = start + 1
    while end < limit and (joins(site, links[end - 1], links[end]) if via == NAV_GOAL
                           else links[end].via == via):
        end += 1
    return end


def reach_end(site, links, start: int) -> int:
    end = stretch_end(site, links, start)
    while end < len(links) and joins(site, links[end - 1], links[end]):
        more = stretch_end(site, links, end)
        if any(link.lane or site.nodes[link.b].kind == AT_DOCK for link in links[end:more]):
            break
        end = more
    return end


# ---- 링크 위 진행 거리와 지나감 판정 ----
def heading(a, b) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _pieces(link) -> list:
    return [(a, b) for a, b in zip(link.points, link.points[1:], strict=False)
            if math.dist(a, b) > 0.0]


def along(link, x: float, y: float) -> float:
    best, best_gap, start = 0.0, math.inf, 0.0
    for (ax, ay), (bx, by) in _pieces(link):
        span = math.dist((ax, ay), (bx, by))
        t = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / (span * span)
        t = min(max(t, 0.0), 1.0)
        gap = math.hypot(x - ax - t * (bx - ax), y - ay - t * (by - ay))
        if gap < best_gap - 1e-12:
            best, best_gap = start + t * span, gap
        start += span
    return min(max(best, 0.0), link.length)


def passed(link, x: float, y: float) -> bool:
    return along(link, x, y) >= link.length - ON_NODE_M


# ---- 현재 위치부터 이어지는 점과 방향 ----
def _from_here(link, x: float, y: float) -> tuple[list, list]:
    pieces = _pieces(link)
    s = along(link, x, y)
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


def _line(links, x: float, y: float) -> tuple[list, list]:
    points, yaws = _from_here(links[0], x, y)
    for link in links[1:]:
        for a, b in _pieces(link):
            points.append(b)
            yaws.append(heading(a, b))
    return points, yaws


# ---- 이동 명령과 물러나기 명령 만들기 ----
def move_command(site, robot: str, goal_id: int, links, x: float, y: float,
                 after=None, priority: int = -1) -> MoveCmd:
    node = site.nodes[links[-1].b]
    if links[0].via == NAV_GOAL:
        if node.yaw is not None:
            yaw = node.yaw
        elif after is not None and after.via == FOLLOW_PATH:
            yaw = heading(*_pieces(after)[0])
        else:
            yaw = heading(links[-1].points[-2] if len(links) > 1 else (x, y),
                          (node.x, node.y))
        return MoveCmd(robot, goal_id, node.id, node.x, node.y, yaw, (), True, NAV_GOAL,
                       priority)
    points, yaws = _line(links, x, y)
    end_yaw = node.yaw if node.yaw is not None else yaws[-1]
    path = tuple((px, py, yaw) for (px, py), yaw in zip(points, [*yaws, end_yaw], strict=True))
    return MoveCmd(robot, goal_id, node.id, node.x, node.y, end_yaw, path, True, FOLLOW_PATH,
                   priority)


def retreat_command(params, robot: str, goal_id: int, links, x: float, y: float) -> RetreatCmd:
    points, yaws = _line(links, x, y)
    facing = [math.remainder(yaw + math.pi, math.tau) for yaw in yaws]
    path = tuple((px, py, yaw) for (px, py), yaw in zip(points, [facing[0], *facing],
                                                         strict=True))
    distance = sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))
    return RetreatCmd(robot, goal_id, distance, params.retreat_speed_mps, path)


# ---- 현재 위치에서 노드로 가는 임시 링크 ----
def free_link(site, x: float, y: float, node: str, zones=()) -> Link:
    end = site.nodes[node]
    points = ((x, y), (end.x, end.y))
    return Link("", node, NAV_GOAL, "", points, tuple(zones) or (end.zone,), math.dist(*points))


def back_link(site, x: float, y: float, node: str, zones, via: str = "", bends=()) -> Link:
    corner = [site.node_pose(via)] if via else []
    end = site.nodes[node]
    points = ((x, y), *corner, *(tuple(p) for p in bends), (end.x, end.y))
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))
    return Link("", node, RETREAT, "", points, tuple(zones), length)


# ---- 속도와 남은 주행 시간 계산 ----
def speed(params, via: str) -> float:
    if via == NAV_GOAL:
        return params.nav_speed_mps
    return params.follow_speed_mps if via == FOLLOW_PATH else params.retreat_speed_mps


def travel_s(params, links) -> float:
    return sum(link.length / speed(params, link.via) for link in links)


def left_s(params, links, x: float, y: float) -> float:
    if not links:
        return 0.0
    first = links[0]
    here = (math.dist((x, y), first.points[-1]) if first.via == NAV_GOAL
            else first.length - along(first, x, y))
    return here / speed(params, first.via) + travel_s(params, links[1:])
