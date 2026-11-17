# ---- 로봇단 양보 사다리 흉내(pode_fsm의 판정을 그대로 쓴다) ----
from __future__ import annotations

import math
from dataclasses import dataclass, field

from pode_fsm.backup import backup_distance
from pode_fsm.local_yield import (
    BACKUP,
    NO_PRIORITY,
    RETRY,
    WAIT,
    Peer,
    blocking_peer,
    extend_trail,
    straight_back_m,
    yields_to,
)
from pode_fsm.params import FsmParams
from pode_fsm.yield_nav import TRAIL_KEEP_M, TRAIL_STEP_M

from .geom import ahead, length, nearest_index
from .nav_goal import NavGoalRun
from .order_run import OrderRun
from .runs import FOLLOW_BLOCKED_S, LOOK_M, FollowRun, blocked_ahead


# ---- 로봇 한 대의 사다리 상태 ----
@dataclass
class _Ladder:
    run: object = None
    leaf: object = None
    step: str = ""
    stood_since: float = 0.0
    wait_until: float = 0.0
    clear_since: float | None = None
    back_left: float = 0.0
    retry_since: float = 0.0
    left_at_stuck: float = 0.0
    used: bool = False
    origin: tuple | None = None
    trail: list = field(default_factory=list)
    trail_leaf: object = None
    driven: float = 0.0


# ---- 걸음마다 run을 대신 돌리는 사다리 ----
class LocalYieldLadder:
    def __init__(self, params: FsmParams | None = None):
        self.p = params or FsmParams()
        self._robots: dict[str, _Ladder] = {}

    def step_of(self, robot) -> str:
        st = self._robots.get(robot.name)
        if st is None or robot.run is None or st.run is not robot.run:
            return ""
        return st.step

    def step(self, ctx, robot, run):
        st = self._robots.setdefault(robot.name, _Ladder())
        if st.run is not run:
            st.run, st.step, st.trail, st.driven = run, "", [], 0.0
            st.trail_leaf = None
        if st.step and _leaf(run) is not st.leaf:
            st.step = ""
        if st.step == BACKUP:
            self._back_up(ctx, robot, st)
            return None
        if st.step == WAIT and isinstance(st.leaf, FollowRun):
            self._wait_on_path(ctx, robot, st)
            return None
        was_blocked = getattr(_leaf(run), "blocked", False)
        before = robot.pose
        result = run.step(ctx, robot)
        if result is not None:
            st.step = ""
            if result.success:
                st.used, st.origin = False, None
            return result
        leaf = _leaf(run)
        if st.step and leaf is not st.leaf:
            st.step = ""
        if leaf is not None and leaf is not st.trail_leaf:
            if not (isinstance(leaf, FollowRun) and isinstance(st.trail_leaf, FollowRun)):
                st.trail, st.driven = [before], 0.0
            st.trail_leaf = leaf
        if not isinstance(leaf, (FollowRun, NavGoalRun)):
            return None
        st.driven += extend_trail(st.trail, robot.pose, TRAIL_STEP_M, TRAIL_KEEP_M)
        if st.step == WAIT:
            self._wait_on_nav(ctx, robot, st)
        elif st.step == RETRY:
            self._retry(ctx, robot, st)
        else:
            self._maybe_start(ctx, robot, st, leaf, was_blocked)
        return None

    # ---- 섬: 1초 서면 기다림 시작(한 자리에서 한 번) ----
    def _maybe_start(self, ctx, robot, st, leaf, was_blocked) -> None:
        if (st.origin is not None
                and math.dist(robot.pose, st.origin) >= self.p.recovery_reset_m):
            st.used, st.origin = False, None
        if st.used:
            return
        if isinstance(leaf, FollowRun):
            if not leaf.blocked or was_blocked:
                return
            stood_since = ctx.t - leaf.stood
            leaf.blocked = False
        else:
            if leaf.stuck or ctx.t - leaf.mark_t < self.p.block_report_s - 1e-9:
                return
            stood_since = leaf.mark_t
            leaf.held_at, leaf.blocked = None, False
        st.used, st.origin = True, robot.pose
        st.leaf, st.step = leaf, WAIT
        st.stood_since, st.wait_until = stood_since, ctx.t + self.p.local_wait_s
        st.clear_since = None
        st.left_at_stuck = _left(leaf, robot)

    # ---- 기다림: 고정 경로는 서서 앞을 보고, 넓은 구역은 Nav2를 그대로 둔다 ----
    def _wait_on_path(self, ctx, robot, st) -> None:
        leaf = st.leaf
        obstacle = _obstacle(ctx, robot, leaf)
        if ctx.t - st.stood_since >= self.p.blocked_after_s - 1e-9 and obstacle:
            self._give_up(ctx, robot, st)
            return
        if not obstacle:
            if st.clear_since is None:
                st.clear_since = ctx.t
            if ctx.t - st.clear_since >= self.p.block_clear_s - 1e-9:
                st.step = ""
                leaf.stood, leaf.blocked = 0.0, False
            return
        st.clear_since = None
        if ctx.t >= st.wait_until - 1e-9:
            self._decide(ctx, robot, st, self._path_ahead(robot, leaf))

    def _wait_on_nav(self, ctx, robot, st) -> None:
        leaf = st.leaf
        if ctx.t - st.stood_since >= self.p.blocked_after_s - 1e-9:
            self._give_up(ctx, robot, st)
            return
        if leaf.mark_t > st.stood_since:
            st.step = ""
            return
        leaf.held_at, leaf.blocked = None, False
        if ctx.t >= st.wait_until - 1e-9:
            self._decide(ctx, robot, st, self._ray(robot))

    # ---- 물러날지 판정과 거리 ----
    def _decide(self, ctx, robot, st, ahead_points) -> None:
        d = self._backup_m(robot, st) if self._backs_up(ctx, robot, ahead_points) else 0.0
        if d < self.p.backup_min_m:
            self._give_up(ctx, robot, st)
            return
        st.step, st.back_left = BACKUP, d

    def _backs_up(self, ctx, robot, ahead_points) -> bool:
        views = ctx.views()
        peers = [Peer(name, other.x, other.y,
                      int((views.get(name) or {}).get("priority", NO_PRIORITY)))
                 for name, other in ctx.present().items()]
        peer = blocking_peer(ahead_points, peers, robot.name, self.p.local_peer_near_m)
        if peer is None:
            return False
        mine = next((p.priority for p in peers if p.name == robot.name), NO_PRIORITY)
        return yields_to(robot.name, mine, peer)

    def _backup_m(self, robot, st) -> float:
        leaf = st.leaf
        if isinstance(leaf, FollowRun):
            since = st.driven
            if len(leaf.points) >= 2:
                i = nearest_index(robot.pose, leaf.points)
                since = math.dist(leaf.points[i], robot.pose)
            limit = backup_distance(since, self.p.backup_max_m,
                                    self.p.release_distance_m + self.p.backup_margin_m)
            return max(0.0, min(self.p.local_backup_m, limit, st.driven))
        keep = self.p.release_distance_m + self.p.backup_margin_m
        pose = (robot.x, robot.y, robot.yaw)
        straight = straight_back_m(st.trail, pose, self.p.local_backup_m,
                                   self.p.backup_lateral_limit_m)
        return max(0.0, min(straight, st.driven - keep))

    # ---- 물러남: 곧게 뒤로, 뒤가 막히면 그만 ----
    def _back_up(self, ctx, robot, st) -> None:
        step = min(st.back_left, self.p.backup_speed_mps * ctx.dt)
        ux, uy = math.cos(robot.yaw), math.sin(robot.yaw)
        end = (robot.x - ux * st.back_left, robot.y - uy * st.back_left)
        new = (robot.x - ux * step, robot.y - uy * step)
        if blocked_ahead(ctx, robot, [robot.pose, end], LOOK_M + step) or not ctx.clear_to(
                robot, new):
            self._retry_start(ctx, robot, st)
            return
        robot.x, robot.y = new
        st.back_left -= step
        if st.back_left <= 1e-9:
            self._retry_start(ctx, robot, st)

    # ---- 다시 감: 막힌 곳을 지나면 끝, 다시 서면 BLOCKED ----
    def _retry_start(self, ctx, robot, st) -> None:
        leaf = st.leaf
        st.step, st.retry_since = RETRY, ctx.t
        if leaf.way and math.dist(robot.pose, leaf.way[0]) > 1e-9:
            leaf.way = [robot.pose, *leaf.way]
        if isinstance(leaf, FollowRun):
            leaf.stood, leaf.blocked = 0.0, False
        else:
            leaf.mark, leaf.mark_t = robot.pose, ctx.t
            leaf.stuck, leaf.held_at, leaf.blocked = False, None, False

    def _retry(self, ctx, robot, st) -> None:
        leaf = st.leaf
        if _left(leaf, robot) <= st.left_at_stuck - self.p.recovery_pass_m:
            st.step = ""
            return
        if isinstance(leaf, FollowRun):
            if leaf.blocked:
                st.step = ""
            elif (ctx.t - st.stood_since >= self.p.blocked_after_s - 1e-9
                  and _obstacle(ctx, robot, leaf)):
                self._give_up(ctx, robot, st)
            return
        again = (ctx.t - leaf.mark_t >= self.p.block_report_s - 1e-9
                 and ctx.t - st.retry_since >= self.p.block_report_s - 1e-9)
        if again or ctx.t - st.stood_since >= self.p.blocked_after_s - 1e-9:
            self._give_up(ctx, robot, st)

    # ---- 포기: BLOCKED로 보고(목표는 그대로) ----
    def _give_up(self, ctx, robot, st) -> None:
        leaf = st.leaf
        st.step = ""
        if isinstance(leaf, FollowRun):
            leaf.stood, leaf.blocked = max(leaf.stood, FOLLOW_BLOCKED_S), True
        else:
            leaf.stuck, leaf.held_at, leaf.blocked = True, robot.pose, True

    # ---- 앞쪽 검사 점 ----
    def _path_ahead(self, robot, leaf) -> list:
        if not leaf.way or len(leaf.way) < 2:
            return self._ray(robot)
        return _sample(ahead(leaf.way, self.p.block_lookahead_m), self.p.block_sample_m)

    def _ray(self, robot) -> list:
        tip = (robot.x + math.cos(robot.yaw) * self.p.block_lookahead_m,
               robot.y + math.sin(robot.yaw) * self.p.block_lookahead_m)
        return _sample([robot.pose, tip], self.p.block_sample_m)


# ---- 도움 함수 ----
def _leaf(run):
    return run.sub if isinstance(run, OrderRun) else run


def _left(leaf, robot) -> float:
    if isinstance(leaf, FollowRun):
        return length(leaf.way) if leaf.way else 0.0
    return math.dist(robot.pose, leaf.target)


def _obstacle(ctx, robot, leaf) -> bool:
    if not leaf.way or len(leaf.way) < 2:
        return False
    return blocked_ahead(ctx, robot, leaf.way, LOOK_M + leaf.speed * ctx.dt)


def _sample(points, step_m) -> list:
    out = [points[0]]
    for a, b in zip(points, points[1:], strict=False):
        span = math.dist(a, b)
        n = max(1, math.ceil(span / step_m))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                for k in range(1, n + 1)]
    return out
