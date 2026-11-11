"""관제 화면(시뮬레이션 전용): 관제가 보는 함대를 마커 명세로 바꾼다. rclpy를 쓰지 않는다.

구역 지도(fleet_core.fleet_map)를 그린다.
- 구역: 종류마다 옅은 바닥색(shared 회색, lane 파랑, dock 호박색).
- 고정 경로(FollowPath, Retreat)는 얇은 선, 지점은 점과 이름.
- 쥔 구역: 구역 바닥을 쥔 로봇 색으로 덮는다. 여러 대가 쥔 구역(shared, lane: holder가
  "pinky1,pinky3")은 흰 테두리와 이름 글로 그린다. 기다리는 로봇이 있으면 그 목록도 글로.
- 로봇: 몸, 방향, 상태 글. 지금 가는 곳(핑키가 보고한 current_goal의 지점이나 스테이션 도크)은
  로봇 색 깃대와 로봇에서 그곳까지의 가는 선.
"""
from __future__ import annotations

from dataclasses import dataclass

from fleet_core.fleet_map import DOCK, FOLLOW_PATH, LANE, RETREAT, SHARED

STALE_S = 3.0  # 이만큼 자세가 없으면 회색

GRAY = (0.55, 0.55, 0.55, 1.0)
RED = (1.0, 0.1, 0.1, 1.0)
AMBER = (1.0, 0.75, 0.0, 1.0)
WHITE = (1.0, 1.0, 1.0, 1.0)
LABEL = (0.95, 0.95, 0.95, 1.0)
ZONE_COLORS = {SHARED: (0.6, 0.6, 0.6, 0.15), LANE: (0.2, 0.4, 1.0, 0.25),
               DOCK: (1.0, 0.65, 0.1, 0.25)}
ROBOT_COLORS = {
    "pinky1": (0.2, 0.45, 1.0, 1.0),
    "pinky2": (0.1, 0.8, 0.25, 1.0),
    "pinky3": (1.0, 0.6, 0.0, 1.0),
}
_SPARE = ((0.8, 0.3, 0.9, 1.0), (0.0, 0.8, 0.8, 1.0), (0.9, 0.9, 0.2, 1.0))


@dataclass(frozen=True)
class Spec:
    """마커 하나. kind는 visualization_msgs/Marker 형식 이름, pose는 (x, y, z, yaw)."""
    ns: str
    id: int
    kind: str
    pose: tuple = (0.0, 0.0, 0.0, 0.0)
    scale: tuple = (1.0, 1.0, 1.0)
    color: tuple = LABEL
    points: tuple = ()  # (x, y) 목록. 높이는 pose의 z
    text: str = ""


def robot_color(robot: str) -> tuple:
    if robot in ROBOT_COLORS:
        return ROBOT_COLORS[robot]
    return _SPARE[sum(map(ord, robot)) % len(_SPARE)]


def _faded(color: tuple, alpha: float) -> tuple:
    return (*color[:3], alpha)


def holders_of(holder: str) -> list[str]:
    """ZoneView.holder("" 또는 "pinky1,pinky3")를 이름 목록으로."""
    return [name for name in holder.split(",") if name]


def _rect_center(rect) -> tuple[float, float]:
    x0, y0, x1, y1 = rect
    return (x0 + x1) / 2, (y0 + y1) / 2


def _outline(rect) -> tuple:
    """LINE_LIST 점: 사각형 네 변."""
    x0, y0, x1, y1 = rect
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return tuple(p for a, b in zip(corners, corners[1:] + corners[:1], strict=True)
                 for p in (a, b))


def zone_cubes(rects, z: float) -> list[tuple]:
    """사각형마다 (중심, 크기)."""
    return [(_rect_center(r), (r[2] - r[0], r[3] - r[1], z)) for r in rects]


def map_specs(site) -> list[Spec]:
    """구역 바닥(종류별 색), 고정 경로 선, 지점 점과 이름."""
    specs = []
    i = 0
    for zone in sorted(site.zones.values(), key=lambda z: z.id):
        for (cx, cy), size in zone_cubes(zone.rects, 0.002):
            specs.append(Spec("zones", i, "CUBE", (cx, cy, 0.001, 0.0), size,
                              ZONE_COLORS[zone.kind]))
            i += 1
        cx, cy = _rect_center(zone.rects[0])
        specs.append(Spec("zone_labels", len(specs), "TEXT", (cx, cy, 0.03, 0.0),
                          (0.0, 0.0, 0.04), _faded(LABEL, 0.6), text=zone.id))
    seen, points = set(), []
    for link in site.links():
        key = frozenset((link.a, link.b))
        if link.via not in (FOLLOW_PATH, RETREAT) or key in seen:
            continue
        seen.add(key)
        for p, q in zip(link.points, link.points[1:], strict=False):
            points += [tuple(p), tuple(q)]
    specs.append(Spec("paths", 0, "LINE_LIST", (0.0, 0.0, 0.005, 0.0), (0.008, 0.0, 0.0),
                      _faded(GRAY, 0.9), tuple(points)))
    names = sorted(site.nodes)
    poses = [site.node_pose(n) for n in names]
    specs.append(Spec("nodes", 0, "SPHERE_LIST", (0.0, 0.0, 0.01, 0.0), (0.03, 0.03, 0.03),
                      GRAY, tuple(poses)))
    for j, (name, (x, y)) in enumerate(zip(names, poses, strict=True)):
        specs.append(Spec("node_labels", j, "TEXT", (x + 0.04, y + 0.04, 0.05, 0.0),
                          (0.0, 0.0, 0.045), LABEL, text=name))
    return specs


def robot_label(status, stale: bool) -> tuple[str, tuple]:
    """pinky1  MOVING  TO_WORK R-003 글과 색. 고장, BLOCKED는 빨강, RECOVERING은 호박색."""
    fault = getattr(status, "fault", "") or "NONE"
    state = getattr(status, "fsm_state", "")
    parts = [status.robot_id, state or "?",
             " ".join(p for p in (getattr(status, "task_type", ""),
                                  getattr(status, "order_id", "")) if p)]
    if fault != "NONE":
        reason = getattr(status, "fault_reason", "")
        parts.append(f"{fault}:{reason}" if reason else fault)
    if stale:
        parts.append("STALE")
    text = "  ".join(p for p in parts if p)
    if fault != "NONE" or state == "BLOCKED":
        return text, RED
    if state == "RECOVERING":
        return text, AMBER
    return text, GRAY if stale else robot_color(status.robot_id)


@dataclass
class _NoStatus:
    robot_id: str
    fsm_state: str = ""


class FleetView:
    """받은 상태를 모아 두고 그릴 때마다 명세 목록을 만든다. 시각은 초(호출하는 쪽 시계)."""

    def __init__(self, site):
        self.site = site
        self._map = map_specs(site)
        self._status: dict = {}
        self._pose: dict = {}  # 로봇 -> (x, y, yaw, 시각)
        self._zones: list = []
        self._goal: dict = {}  # robot -> current_goal as the robot reports it

    def set_statuses(self, statuses) -> None:
        self._status = {s.robot_id: s for s in statuses}

    def set_pose(self, robot: str, x: float, y: float, yaw: float, now: float) -> None:
        self._pose[robot] = (x, y, yaw, now)

    def set_goal(self, robot: str, goal: str) -> None:
        self._goal[robot] = goal

    def goal_pose(self, goal: str):
        """(x, y) of a MoveTo target node or a Dock station, None if unknown."""
        if goal in self.site.nodes:
            return self.site.node_pose(goal)
        station = self.site.stations.get(goal)
        return self.site.node_pose(station.dock) if station else None

    def set_zones(self, zones) -> None:
        self._zones = list(zones)

    def specs(self, now: float) -> list[Spec]:
        return [*self._map, *self._zone_specs(), *self._goal_specs(), *self._robot_specs(now)]

    def _zone_specs(self) -> list[Spec]:
        specs = []
        for i, view in enumerate(self._zones):
            zone = self.site.zones.get(view.zone_id)
            if zone is None:
                continue
            names = holders_of(view.holder)
            cx, cy = _rect_center(zone.rects[0])
            if len(names) == 1:
                color = _faded(robot_color(names[0]), 0.45)
                for j, ((x, y), size) in enumerate(zone_cubes(zone.rects, 0.004)):
                    specs.append(Spec("zone_held", i * 10 + j, "CUBE", (x, y, 0.003, 0.0),
                                      size, color))
            elif names:
                specs.append(Spec("zone_shared_by", i, "LINE_LIST", (0.0, 0.0, 0.006, 0.0),
                                  (0.012, 0.0, 0.0), WHITE,
                                  tuple(p for r in zone.rects for p in _outline(r))))
            if names:
                specs.append(Spec("zone_holder", i, "TEXT", (cx, cy - 0.05, 0.05, 0.0),
                                  (0.0, 0.0, 0.045), WHITE,
                                  text=f"{zone.id}: {', '.join(names)}"))
            if view.queue:
                specs.append(Spec("zone_queue", i, "TEXT", (cx, cy - 0.10, 0.05, 0.0),
                                  (0.0, 0.0, 0.045), LABEL,
                                  text=f"{zone.id} wait: {' '.join(view.queue)}"))
        return specs

    def _goal_specs(self) -> list[Spec]:
        specs = []
        for i, robot in enumerate(sorted(self._goal)):
            goal = self._goal[robot]
            where = self.goal_pose(goal) if goal else None
            if where is None:
                continue
            gx, gy = where
            color = robot_color(robot)
            specs += [
                Spec("goal", i, "CYLINDER", (gx, gy, 0.1, 0.0), (0.03, 0.03, 0.2), color),
                Spec("goal_text", i, "TEXT", (gx, gy, 0.26, 0.0), (0.0, 0.0, 0.06), color,
                     text=f"{robot} -> {goal}"),
            ]
            if robot in self._pose:
                x, y = self._pose[robot][:2]
                specs.append(Spec("goal_line", i, "LINE_LIST", (0.0, 0.0, 0.02, 0.0),
                                  (0.012, 0.0, 0.0), _faded(color, 0.6), ((x, y), (gx, gy))))
        return specs

    def _robot_specs(self, now: float) -> list[Spec]:
        specs = []
        for i, robot in enumerate(sorted(self._pose)):
            x, y, yaw, seen = self._pose[robot]
            status = self._status.get(robot) or _NoStatus(robot)
            stale = (now - seen > STALE_S
                     or getattr(status, "fault", "") == "COMM_LOST")
            body = GRAY if stale else robot_color(robot)
            text, color = robot_label(status, stale)
            specs += [
                Spec("robot_body", i, "CYLINDER", (x, y, 0.04, yaw), (0.12, 0.12, 0.08), body),
                Spec("robot_heading", i, "ARROW", (x, y, 0.09, yaw), (0.15, 0.025, 0.025),
                     body),
                Spec("robot_label", i, "TEXT", (x, y, 0.3, 0.0), (0.0, 0.0, 0.07), color,
                     text=text),
            ]
        return specs
