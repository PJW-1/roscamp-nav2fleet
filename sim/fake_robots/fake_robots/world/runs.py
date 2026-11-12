"""로봇이 하는 goal 하나: FollowPath, Retreat, Dock. Nav2 goal은 nav_goal.py.

각 run은 step(ctx, robot)마다 로봇을 움직이고, 끝나면 Result를 돌려준다. ctx는 세계가 주는
t, dt, radius, rng, discs(robot)(다른 로봇과 물체의 원 (x, y, 반지름))다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .geom import advance, ahead, angle_diff, heading, nearest_index, path_gap

LOOK_M = 0.10  # 갈 길을 이만큼 앞까지 보고 몸이 닿을 것이 있으면 선다(충돌 검사)
GAP_M = 0.01  # 몸과 물체 사이에 남기는 틈
TURN_RAD_S = 1.0  # 제자리 회전 속도(RPP rotate_to_heading_angular_vel)
FOLLOW_BLOCKED_S = 1.0  # FollowPath: 이만큼 막혀 서 있으면 BLOCKED(핑키 계약 4절: 2초 안)
MOVE, RETREAT, DOCK = "move", "retreat", "dock"
HEARTBEAT_LOST = "HEARTBEAT_LOST"  # 관제 heartbeat가 끊겨 로봇이 스스로 끝낸 이동의 이유


@dataclass(frozen=True)
class Result:
    kind: str  # move, retreat, dock
    goal_id: int
    success: bool
    reason: str = ""
    last_done_seq: int | None = None  # order goal이면 끝낸 마지막 조각 번호


def blocked_ahead(ctx, robot, way, look) -> bool:
    """way(첫 점은 로봇)의 앞 look 안에 몸이 닿을 원이 있고, 가면 그 원에 더 가까워진다."""
    seg = ahead(way, look)
    for ox, oy, reach in ctx.discs(robot):
        gap = path_gap((ox, oy), seg)
        if gap <= ctx.radius + reach + GAP_M and gap < math.dist((ox, oy), robot.pose) - 1e-9:
            return True
    return False


def jittered(ctx, speed: float) -> float:
    spread = ctx.imperfect.speed_jitter
    return speed * (1.0 + ctx.rng.uniform(-spread, spread)) if spread else speed


def move_along(ctx, robot, way, step):
    """way를 따라 step만큼 간다. 그 걸음이 다른 로봇 몸에 닿으면(2r 안) 가지 않는다.
    (남은 way, 갔는가)."""
    new, rest, _ = advance(way, step)
    if not ctx.clear_to(robot, new):
        return way, False
    robot.x, robot.y = new
    return rest, True


def foot(p, a, b):
    """p에서 선분 ab에 내린 발."""
    vx, vy = b[0] - a[0], b[1] - a[1]
    span = vx * vx + vy * vy
    t = max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / span)) if span else 0.0
    return (a[0] + t * vx, a[1] + t * vy)


def end_along(rng, error_m: float, way):
    """도착 오차: 마지막 점을 마지막 선분 방향으로 error_m 안에서 앞이나 뒤로 민다.
    선을 떠나지 않는다."""
    if error_m <= 0.0 or len(way) < 2 or math.dist(way[-2], way[-1]) <= 1e-9:
        return way
    (ax, ay), (bx, by) = way[-2], way[-1]
    span = math.dist(way[-2], way[-1])
    shift = max(-0.5 * span, rng.uniform(-error_m, error_m))
    return [*way[:-1], (bx + (bx - ax) / span * shift, by + (by - ay) / span * shift)]


def end_offset(rng, error_m: float, target):
    """도착 오차: 목표에서 error_m 안의 아무 자리."""
    if error_m <= 0.0:
        return target
    angle, size = rng.uniform(-math.pi, math.pi), rng.uniform(0.0, error_m)
    return (target[0] + math.cos(angle) * size, target[1] + math.sin(angle) * size)


class Run:
    kind = MOVE
    blocked = False  # 보고를 BLOCKED로

    def __init__(self, goal_id: int):
        self.goal_id = goal_id
        self.started = False

    def done(self, success: bool, reason: str = "") -> Result:
        return Result(self.kind, self.goal_id, success, reason)

    def step(self, ctx, robot) -> Result | None:
        raise NotImplementedError


class Turning:
    """끝에서 목표 방향으로 돈다. 다 돌면 True."""

    @staticmethod
    def toward(ctx, robot, yaw) -> bool:
        if yaw is None:
            return True
        turn = angle_diff(robot.yaw, yaw)
        step = TURN_RAD_S * ctx.dt
        if abs(turn) <= step:
            robot.yaw = yaw
            return True
        robot.yaw += math.copysign(step, turn)
        return False


class FollowRun(Run):
    """주어진 선을 따라간다. 충돌 검사가 세우고, 선을 떠나지 않는다(지시서 2-2, R14)."""

    def __init__(self, goal_id, points, target, yaw, speed, connect_m, turn_at_end=True):
        super().__init__(goal_id)
        self.points = [tuple(p[:2]) for p in points]
        self.target, self.yaw, self.speed = tuple(target), yaw, speed
        self.connect_m, self.turn_at_end = connect_m, turn_at_end
        self.way: list | None = None
        self.stood = 0.0

    def _start(self, ctx, robot) -> Result | None:
        self.started = True
        if self.points and path_gap(robot.pose, self.points) > self.connect_m:
            return self.done(False, "PATH_NOT_CONNECTED")
        if len(self.points) < 2:
            way = [robot.pose, self.target]
        else:  # 선 위 가장 가까운 자리로 돌아와 선을 따른다(pinky_fsm처럼 지난 꼭짓점은 건너뛴다)
            i = nearest_index(robot.pose, self.points)
            way = [robot.pose, foot(robot.pose, *self.points[i:i + 2]), *self.points[i + 1:]]
        way = [p for k, p in enumerate(way) if k == 0 or math.dist(p, way[k - 1]) > 1e-9]
        self.way = end_along(ctx.rng, ctx.imperfect.arrival_error_m, way)
        return None

    def step(self, ctx, robot) -> Result | None:
        if not self.started:
            failed = self._start(ctx, robot)
            if failed:
                return failed
        if len(self.way) > 1:
            step = jittered(ctx, self.speed) * ctx.dt
            if blocked_ahead(ctx, robot, self.way, LOOK_M + step):
                self.stood += ctx.dt
                self.blocked = self.stood >= FOLLOW_BLOCKED_S
                return None
            robot.yaw = heading(self.way[0], self.way[1], robot.yaw)
            self.way, moved = move_along(ctx, robot, self.way, step)
            if not moved:
                self.stood += ctx.dt
                self.blocked = self.stood >= FOLLOW_BLOCKED_S
                return None
            self.stood, self.blocked = 0.0, False
            if len(self.way) > 1:
                return None
        if self.turn_at_end and not Turning.toward(ctx, robot, self.yaw):
            return None
        return self.done(True)


class RetreatRun(Run):
    """뒤로 간다(돌아서지 않는다). 뒤에 무엇이 가까우면 그 자리에 서서 OBSTACLE_BEHIND."""

    kind = RETREAT

    def __init__(self, goal_id, points, distance, speed, connect_m):
        super().__init__(goal_id)
        self.points = [tuple(p[:2]) for p in points]
        self.distance, self.speed, self.connect_m = distance, speed, connect_m
        self.way: list | None = None

    def _start(self, robot) -> Result | None:
        self.started = True
        if self.points:
            if math.dist(robot.pose, self.points[0]) > self.connect_m:
                return self.done(False, "FAILED")
            self.way = [robot.pose, *self.points[1:]]
        else:
            self.way = [robot.pose, (robot.x - math.cos(robot.yaw) * self.distance,
                                     robot.y - math.sin(robot.yaw) * self.distance)]
        return None

    def step(self, ctx, robot) -> Result | None:
        if not self.started:
            failed = self._start(robot)
            if failed:
                return failed
        if len(self.way) > 1:
            step = jittered(ctx, self.speed) * ctx.dt
            if blocked_ahead(ctx, robot, self.way, LOOK_M + step):
                return self.done(False, "OBSTACLE_BEHIND")
            self.way, _ = move_along(ctx, robot, self.way, step)
            if len(self.way) > 1:
                return None
        return self.done(True)


class DockRun(Run):
    kind = DOCK

    def __init__(self, goal_id, station, duration_s, fail=False):
        super().__init__(goal_id)
        self.station, self.left, self.fail = station, duration_s, fail

    def step(self, ctx, robot) -> Result | None:
        self.started = True
        self.left -= ctx.dt
        if self.left > 1e-9:
            return None
        if self.fail:
            return self.done(False, "DOCK_FAILED")
        robot.docked_at = self.station
        return self.done(True)
