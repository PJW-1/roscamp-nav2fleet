"""가는 길 하나(Trip)를 구간(한꺼번에 잡는 단위)과 조각(명령 하나)으로 나눈다. 순수 함수다.

- 구간: 지금 선 자리에서 다음에 서서 기다릴 수 있는 지점(도크, 대기 자리, 주차)까지의 연결들.
- 조각: 명령 하나. 이어지는 고정 경로는 꺾은선 하나, 후진도 하나, Nav2 goal은 연결마다 하나다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from fleet_core.common.commands import MoveCmd, RetreatCmd
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, Link

ON_NODE_M = 0.03  # 선 위에서 이만큼 안이면 그 지점에 있다고 본다


@dataclass
class Trip:
    goal: str  # 목표 지점
    links: list  # 지금 자리에서 목표까지의 연결
    done: int = 0  # 다 지난 연결 수
    granted: int = 0  # 구역을 잡은 연결 수(앞에서부터)
    sent: int = 0  # 명령을 낸 연결 수(앞에서부터)
    stale: bool = False  # 목적이 바뀌었다. 새 길을 잡으면 갈아타고, 못 잡으면 잡은 데까지 간다
    turn: bool = False  # 도크 차례를 받아 떠난 길이다(turns.py). stale이면 차례도 내놓은 것이다

    @property
    def finished(self) -> bool:
        return self.done >= len(self.links)

    def route(self) -> list[str]:
        """아직 앞에 있는 지점들. 목표가 마지막이다."""
        return [link.b for link in self.links[self.done:]]


def stretch_end(site, links, start: int) -> int:
    """start번째 연결부터 다음에 기다릴 수 있는 지점까지: 그 끝 연결의 다음 번호."""
    for index in range(start, len(links)):
        if site.can_wait(links[index].b):
            return index + 1
    return len(links)


def needs(links) -> tuple[list, list, dict]:
    """이 연결들을 달리려면 잡을 (구역들, 지점들, 한 줄 구역 -> 들어가는 쪽 지점)."""
    zones, nodes, lanes = [], [], {}
    for link in links:
        zones.extend(zone for zone in link.zones if zone not in zones)
        if link.lane:
            lanes.setdefault(link.lane, link.a)
        if link.b not in nodes:
            nodes.append(link.b)
    return zones, nodes, lanes


def segment_end(links, start: int, limit: int) -> int:
    """start번째 연결에서 시작하는 명령 하나가 덮는 끝(다음 번호). limit는 잡은 연결 수다."""
    via = links[start].via
    end = start + 1
    if via != NAV_GOAL:
        while end < limit and links[end].via == via:
            end += 1
    return end


def heading(a, b) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _pieces(link) -> list:
    """연결의 선을 이루는 곧은 조각들(길이 0인 것은 뺀다). 꺾는 점이 없으면 하나다."""
    return [(a, b) for a, b in zip(link.points, link.points[1:], strict=False)
            if math.dist(a, b) > 0.0]


def along(link, x: float, y: float) -> float:
    """(x, y)를 연결의 선(꺾은선)에 내린 자리. a에서부터의 거리이고 0과 길이 사이다."""
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
    """(x, y)가 연결의 끝 지점에 닿았거나 지났다."""
    return along(link, x, y) >= link.length - ON_NODE_M


def _from_here(link, x: float, y: float) -> tuple[list, list]:
    """연결의 남은 선: (점들, 조각마다의 방향). 첫 점은 로봇이 연결의 시작 지점에 있으면 그 지점,
    달리던 중이면 선 위의 지금 자리다."""
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
    """조각 하나(links)의 꺾은선: (점들, 조각마다 달리는 방향)."""
    points, yaws = _from_here(links[0], x, y)
    for link in links[1:]:
        for a, b in _pieces(link):
            points.append(b)
            yaws.append(heading(a, b))
    return points, yaws


def move_command(site, robot: str, goal_id: int, links, x: float, y: float,
                 after=None, priority: int = -1) -> MoveCmd:
    """조각 하나(links)를 달리는 MoveCmd. after는 그다음에 달릴 연결이다(없으면 None).

    Nav2 goal의 방향은 지점에 적힌 방향이다. 없으면 이어질 고정 경로의 방향, 그것도 없으면
    가는 방향이다.
    """
    node = site.nodes[links[-1].b]
    if links[0].via == NAV_GOAL:
        if node.yaw is not None:
            yaw = node.yaw
        elif after is not None and after.via == FOLLOW_PATH:
            yaw = heading(*_pieces(after)[0])
        else:
            yaw = heading((x, y), (node.x, node.y))
        return MoveCmd(robot, goal_id, node.id, node.x, node.y, yaw, (), True, NAV_GOAL,
                       priority)
    points, yaws = _line(links, x, y)
    end_yaw = node.yaw if node.yaw is not None else yaws[-1]
    path = tuple((px, py, yaw) for (px, py), yaw in zip(points, [*yaws, end_yaw], strict=True))
    return MoveCmd(robot, goal_id, node.id, node.x, node.y, end_yaw, path, True, FOLLOW_PATH,
                   priority)


def retreat_command(params, robot: str, goal_id: int, links, x: float, y: float) -> RetreatCmd:
    """조각 하나(후진 연결들)를 뒤로 달리는 RetreatCmd. 로봇은 가는 쪽의 반대를 본다."""
    points, yaws = _line(links, x, y)
    facing = [math.remainder(yaw + math.pi, math.tau) for yaw in yaws]
    path = tuple((px, py, yaw) for (px, py), yaw in zip(points, [facing[0], *facing],
                                                         strict=True))
    distance = sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))
    return RetreatCmd(robot, goal_id, distance, params.retreat_speed_mps, path)


def free_link(site, x: float, y: float, node: str, zones=()) -> Link:
    """넓은 구역 안의 (x, y)에서 지점으로 가는 Nav2 goal 연결. 지도에는 없다.

    zones는 지나는 구역이다. 주지 않으면 같은 구역의 지점으로 가는 것이다.
    """
    end = site.nodes[node]
    points = ((x, y), (end.x, end.y))
    return Link("", node, NAV_GOAL, "", points, tuple(zones) or (end.zone,), math.dist(*points))


def back_link(site, x: float, y: float, node: str, zones) -> Link:
    """(x, y)에서 지나온 지점까지 후진으로 물러나는 연결.

    zones는 지금 구역부터 그 지점의 구역까지다.
    """
    end = site.nodes[node]
    points = ((x, y), (end.x, end.y))
    return Link("", node, RETREAT, "", points, tuple(zones), math.dist(*points))


def speed(params, via: str) -> float:
    if via == NAV_GOAL:
        return params.nav_speed_mps
    return params.follow_speed_mps if via == FOLLOW_PATH else params.retreat_speed_mps


def travel_s(params, links) -> float:
    return sum(link.length / speed(params, link.via) for link in links)


def left_s(params, links, x: float, y: float) -> float:
    """links의 첫 연결 위 (x, y)에서 마지막 연결의 끝까지 남은 시간."""
    if not links:
        return 0.0
    first = links[0]
    here = (math.dist((x, y), first.points[-1]) if first.via == NAV_GOAL
            else first.length - along(first, x, y))
    return here / speed(params, first.via) + travel_s(params, links[1:])
