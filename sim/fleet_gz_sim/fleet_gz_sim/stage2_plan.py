"""2단계 Gazebo 한 대 주행(지시서 6-3, 7절 2단계)의 계획과 지표. ROS를 쓰지 않는다.

관제 없이 로봇에 바로 보낼 이동을 구역 지도에서 만든다. 이동 하나(Step)는 계약의 구간 하나다
(TRAFFIC_CONTRACT 2절): nav_goal(MoveTo mode nav_goal), follow_path(MoveTo mode follow_path,
고정 경로 꺾은선), retreat(Retreat path, FollowPathReverse). drive_check가 이것을 보낸다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, SHARED

# 계획 이름 -> (방식, 지점들). 시작 지점에 로봇이 서 있어야 한다. 한 바퀴가 끝나면 시작 지점이다.
# stage2(D2, 추가 지시서 3의 2-5 구간 모두): 마당 Nav2 goal로 N_W, work_in으로 작업 도크, Retreat로
# 마당(N_W)까지, 급행 동쪽으로 가서 J_E로 내려옴, out_in으로 출고 도크, Retreat로 N_E, 급행 서쪽으로
# 가서 J_W로 내려옴, 마당 Nav2 goal로 P-1. 좁은 띠 안에서는 제자리 회전이 없다.
PLANS = {
    "stage2": (
        (NAV_GOAL, ("P-1", "N_W")),
        (FOLLOW_PATH, ("N_W", "X_W", "W")),
        (RETREAT, ("W", "N_W")),
        (FOLLOW_PATH, ("N_W", "X_W", "X_E", "N_E")),
        (FOLLOW_PATH, ("N_E", "X_E", "O")),
        (RETREAT, ("O", "N_E")),
        (FOLLOW_PATH, ("N_E", "X_E", "X_W", "N_W")),
        (NAV_GOAL, ("N_W", "P-1")),
    ),
    # 다른 핑키가 서 있는 마당을 가로지르는 Nav2 goal 한 번(item 6). 서 있는 핑키는 launch가 놓는다
    "cross": ((NAV_GOAL, ("P-1", "P-3")), (NAV_GOAL, ("P-3", "P-1"))),
}


class PlanError(ValueError):
    """계획이 구역 지도와 맞지 않는다."""


@dataclass(frozen=True)
class Step:
    kind: str  # nav_goal, follow_path, retreat
    nodes: tuple[str, ...]
    poses: tuple[tuple[float, float, float], ...]  # 경로(x, y, yaw). nav_goal은 끝 자세 하나

    @property
    def target(self) -> tuple[float, float, float]:
        return self.poses[-1]

    @property
    def label(self) -> str:
        return f"{self.kind} {'>'.join(self.nodes)}"


def _heading(a, b) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def nav_yaw(site, start: str, end: str) -> float:
    """계약 2절: 끝 지점의 yaw, 없으면 거기서 이어지는 고정 경로의 방향, 없으면 시작→끝 방향."""
    node = site.nodes[end]
    if node.yaw is not None:
        return node.yaw
    fixed = [link for link in site.links_from(end) if link.via == FOLLOW_PATH]
    if fixed:
        return _heading(*fixed[0].points[:2])
    return _heading(site.node_pose(start), site.node_pose(end))


def path_poses(site, nodes, reverse: bool = False) -> tuple[tuple[float, float, float], ...]:
    """지점과 꺾는 점마다 (x, y, yaw). yaw는 그 점에서 나가는 방향(끝 점은 들어온 방향). reverse면
    로봇이 보는 방향(달리는 방향의 반대)이다."""
    xy = [site.node_pose(nodes[0])]
    for a, b in zip(nodes, nodes[1:], strict=False):
        xy += site.link(a, b).points[1:]
    out = []
    for i, p in enumerate(xy):
        yaw = _heading(p, xy[i + 1]) if i + 1 < len(xy) else _heading(xy[i - 1], p)
        if reverse:
            yaw = math.remainder(yaw + math.pi, math.tau)
        out.append((p[0], p[1], yaw))
    return tuple(out)


def make_step(site, kind: str, nodes) -> Step:
    nodes = tuple(nodes)
    for a, b in zip(nodes, nodes[1:], strict=False):
        link = site.link(a, b)
        if link is None or link.via != kind:
            raise PlanError(f"{a}->{b} is not a {kind} link of {site.name}")
    if kind == NAV_GOAL:
        if len(nodes) != 2:
            raise PlanError("a nav_goal step has one start and one end node")
        x, y = site.node_pose(nodes[1])
        return Step(kind, nodes, ((x, y, nav_yaw(site, *nodes)),))
    return Step(kind, nodes, path_poses(site, nodes, reverse=kind == RETREAT))


def build(site, plan: str, laps: int = 1) -> list[Step]:
    if plan not in PLANS:
        raise PlanError(f"plan must be one of {', '.join(PLANS)}, got {plan!r}")
    return [make_step(site, kind, nodes) for _ in range(laps) for kind, nodes in PLANS[plan]]


def start_node(plan: str) -> str:
    return PLANS[plan][0][1][0]


# ---- 지표 ----

def yaw_error(yaw: float, target: float) -> float:
    """-π..π로 감은 방향 오차(rad)."""
    return math.remainder(yaw - target, math.tau)


def cross_track(points, x: float, y: float) -> float:
    """점에서 꺾은선(points의 (x, y))까지 가장 짧은 거리."""
    best = math.inf
    for (ax, ay, *_), (bx, by, *_) in zip(points, points[1:], strict=False):
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def keepout_zones(site, points) -> list[str]:
    """점들이 들어간 shared 아닌 구역(처음 들어간 순서, 한 번씩). 어느 구역에도 없는 점은 무시."""
    seen = []
    for x, y, *_ in points:
        zone = site.zone_at(x, y)
        if zone is not None and site.zones[zone].kind != SHARED and zone not in seen:
            seen.append(zone)
    return seen
