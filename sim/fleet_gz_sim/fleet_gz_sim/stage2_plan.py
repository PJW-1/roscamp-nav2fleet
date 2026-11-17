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
    # 다른 Pode가 서 있는 마당을 가로지르는 Nav2 goal 한 번(item 6). 서 있는 Pode는 launch가 놓는다
    "cross": ((NAV_GOAL, ("P-1", "P-3")), (NAV_GOAL, ("P-3", "P-1"))),
    # v7 가운데 통로 재현(2026-11-17): O_IN에서 M_E로 갔다가 돌아온다. 상자는 v7_corridor_box가 놓는다
    "corridor": ((NAV_GOAL, ("O_IN", "M_E")), (NAV_GOAL, ("M_E", "O_IN"))),
    # v7 남쪽 진열대 사이 건넘 틈(2026-11-17): 틈 바로 북쪽(GAP_N)과 남쪽(GAP_S)을 Nav2 goal로 오간다.
    # 둘을 잇는 지점 쌍이 구역 지도에 없어 계획 전용 점을 쓴다(PLAN_POINTS)
    "gap": ((NAV_GOAL, ("M_E", "GAP_N")), (NAV_GOAL, ("GAP_N", "GAP_S")),
            (NAV_GOAL, ("GAP_S", "GAP_N")), (NAV_GOAL, ("GAP_N", "M_E"))),
}
PLAN_LAYOUTS = {"corridor": "v7", "gap": "v7"}  # 계획의 배치. 없으면 d2
# 계획 전용 점(구역 지도에 없는 Nav2 goal 자리, 이름 -> (x, y)). 이음 검사를 하지 않는다.
# GAP_N, GAP_S: 틈 가운데(x 1.12) 북쪽 가운데 통로와 남쪽 통로(칸 입구 높이 0.24). 틈으로 0.31 m,
# 진열대 끝을 돌면 약 1.2 m(1 cm 격자 계산)
PLAN_POINTS = {"gap": {"GAP_N": (1.12, 0.55), "GAP_S": (1.12, 0.24)}}
# 통로를 지났는지 보는 사각형(x0, y0, x1, y1): v7 가운데 통로, 상자 4 둘레의 x 0.90~1.00
CORRIDOR_GATES = {"arena_v7": (0.90, 0.405, 1.00, 0.77)}
# 남쪽 진열대 사이 건넘 틈(y 방향으로 지난다). 네 모서리가 진열대 끝 모서리다
GAP_GATES = {"arena_v7": (0.99, 0.365, 1.25, 0.405)}


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


def nav_yaw(site, start: str, end: str, points=None) -> float:
    """계약 2절: 끝 지점의 yaw, 없으면 거기서 이어지는 고정 경로의 방향, 없으면 시작→끝 방향.
    points(계획 전용 점)에 있는 끝은 시작→끝 방향."""
    points = points or {}
    if end not in points:
        node = site.nodes[end]
        if node.yaw is not None:
            return node.yaw
        fixed = [link for link in site.links_from(end) if link.via == FOLLOW_PATH]
        if fixed:
            return _heading(*fixed[0].points[:2])
    return _heading(_xy(site, start, points), _xy(site, end, points))


def _xy(site, name: str, points) -> tuple[float, float]:
    return tuple(points[name]) if name in points else site.node_pose(name)


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


def make_step(site, kind: str, nodes, points=None) -> Step:
    nodes = tuple(nodes)
    points = points or {}
    if any(n in points for n in nodes):  # 계획 전용 점: 이음 없이 Nav2 goal 하나
        if kind != NAV_GOAL or len(nodes) != 2:
            raise PlanError(f"a plan point needs one nav_goal step, got {kind} {nodes}")
        x, y = _xy(site, nodes[1], points)
        return Step(kind, nodes, ((x, y, nav_yaw(site, *nodes, points)),))
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


def plan_layout(plan: str) -> str:
    return PLAN_LAYOUTS.get(plan, "d2")


def build(site, plan: str, laps: int = 1, points=None) -> list[Step]:
    """points: 계획 전용 점을 바꾸거나 더한다(이름 -> (x, y))."""
    if plan not in PLANS:
        raise PlanError(f"plan must be one of {', '.join(PLANS)}, got {plan!r}")
    if plan in PLAN_LAYOUTS and site.name != f"arena_{PLAN_LAYOUTS[plan]}":
        raise PlanError(f"plan {plan!r} is for layout {PLAN_LAYOUTS[plan]}, not {site.name}")
    own = PLAN_POINTS.get(plan, {})
    for name, xy in (points or {}).items():  # 계획 전용 점만, 유한한 좌표로
        if name not in own:
            raise PlanError(f"plan {plan!r} has no plan point {name!r}"
                            f" (it has: {', '.join(own) or 'none'})")
        if len(xy) != 2 or not all(math.isfinite(v) for v in xy):
            raise PlanError(f"plan point {name} must be two finite numbers, got {xy!r}")
    points = {**own, **(points or {})}
    return [make_step(site, kind, nodes, points)
            for _ in range(laps) for kind, nodes in PLANS[plan]]


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


def through_gate(points, gate, axis: int = 0) -> bool:
    """사각형(x0, y0, x1, y1)을 axis(0이면 x, 1이면 y) 방향으로 한쪽에서 들어가 반대쪽으로 나간 적이
    있으면 참. 들어갔다 되돌아오거나 안에서 멈추면 거짓. 다른 축으로 사각형 밖이면 처음부터 다시 본다.
    점 사이는 보지 않는다: 문 옆이 진열대 면이고 실제 위치가 촘촘하다(약 30 Hz)고 본다."""
    lo, hi = gate[axis], gate[axis + 2]
    other_lo, other_hi = gate[1 - axis], gate[3 - axis]
    last = None  # 마지막으로 있던 바깥쪽(-1 앞, 1 뒤)
    for p in points:
        a, other = p[axis], p[1 - axis]
        if not other_lo <= other <= other_hi:
            last = None
            continue
        if lo <= a <= hi:
            continue
        side = -1 if a < lo else 1
        if last == -side:
            return True
        last = side
    return False


def decimate(points, step_m: float) -> list:
    """앞 점에서 step_m 이상 떨어진 점만 남긴다. 처음과 끝 점은 늘 남긴다."""
    out = []
    for p in points:
        if not out or math.dist(out[-1][:2], p[:2]) >= step_m - 1e-9:
            out.append(p)
    if points and out[-1] is not points[-1]:
        out.append(points[-1])
    return out


def min_distance(points, centre) -> float | None:
    """점들에서 centre까지 가장 짧은 거리. 점이 없으면 None."""
    return min((math.dist(p[:2], centre) for p in points), default=None)


def parse_footprint(text: str) -> list[tuple[float, float]]:
    """Nav2 footprint 문자열 "[[x, y], ...]"."""
    import yaml
    return [(float(x), float(y)) for x, y in yaml.safe_load(text)]


def _inside(polygon, x: float, y: float) -> bool:
    inside = False
    polygon = list(polygon)
    for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1], strict=False):
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def body_distance(pose, polygon, point) -> float:
    """자세 (x, y, yaw)의 몸 다각형(로봇 틀)에서 점까지 거리. 점이 몸 안이면 0."""
    x, y, yaw = pose
    dx, dy = point[0] - x, point[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    px, py = c * dx + s * dy, -s * dx + c * dy  # 로봇 틀
    if _inside(polygon, px, py):
        return 0.0
    return cross_track(list(polygon) + [polygon[0]], px, py)  # 닫힌 꺾은선


def max_pose_error(estimates, truth, max_dt_s: float = math.inf) -> float | None:
    """추정 (t, x, y)마다 시각이 가장 가까운 실제 위치 (t, x, y, ...)와의 거리 중 가장 큰 것.
    시각 차가 max_dt_s보다 큰 추정은 버린다. truth는 시각 순서다. 맞춘 것이 없으면 None."""
    import bisect
    if not estimates or not truth:
        return None
    times = [p[0] for p in truth]
    worst = None
    for t, x, y in estimates:
        i = bisect.bisect_left(times, t)
        near = min((truth[j] for j in (i - 1, i) if 0 <= j < len(truth)),
                   key=lambda p: abs(p[0] - t))
        if abs(near[0] - t) <= max_dt_s:
            worst = max(worst or 0.0, math.dist((x, y), near[1:3]))
    return worst
