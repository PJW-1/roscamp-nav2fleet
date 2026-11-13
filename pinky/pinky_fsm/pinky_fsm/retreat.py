from __future__ import annotations

import math
from collections.abc import Hashable, Sequence
from dataclasses import dataclass

from pinky_fsm.backup import FAILED, OBSTACLE_BEHIND, BackUp
from pinky_fsm.commands import SendPath
from pinky_fsm.geometry import Polyline, poses_backing, rounded_rest
from pinky_fsm.recovery import Obstacle
from pinky_fsm.results import CANCELED, TIMEOUT, Finish

RETREAT_REASONS = (OBSTACLE_BEHIND, TIMEOUT, CANCELED, FAILED)


@dataclass
class _Retreat:
    goal_id: Hashable
    seq: int
    since: float
    allowance_s: float
    distance_m: float = 0.0
    speed_mps: float = 0.0
    route: Polyline | None = None


class RetreatMoves:
    def retreat(self, goal_id: Hashable, now: float, distance_m: float,
                speed_mps: float, path: Sequence[Sequence[float]] = (),
                pose: Sequence[float] | None = None) -> list:
        self._alive = max(self._alive, now)
        was_driving = self._driving(now)
        cmds = self._end_active(CANCELED, stop=True)
        route = None
        if path:
            route = Polyline([(float(q[0]), float(q[1])) for q in path])
            if (pose is None or route.length <= 0.0
                    or route.project(pose[:2])[1] > self.p.connect_tolerance_m
                    or self._too_short(route, pose)):
                return cmds + [Finish(goal_id, False, FAILED)]
        if not (distance_m > 0.0 and speed_mps > 0.0):
            return cmds + [Finish(goal_id, False, FAILED)]
        self._stopped_since = None
        if was_driving:
            self._active = _Retreat(goal_id, 0, now, self.p.retreat_stop_wait_s,
                                    float(distance_m), float(speed_mps), route)
            return cmds
        self._active = _Retreat(goal_id, 0, now, 0.0, float(distance_m), float(speed_mps), route)
        return cmds + self._start_retreat(self._active, now, pose)

    def _start_retreat(self, g: _Retreat, now: float, pose: Sequence[float] | None) -> list:
        if g.route is not None:
            return self._start_reverse_path(g, now, pose)
        self._bseq += 1
        allowance = g.distance_m / g.speed_mps * 2.0 + 2.0
        g.seq, g.since, g.allowance_s = self._bseq, now, allowance
        self._backup_running = True
        self._backup_unanswered[g.seq] = now + allowance + self.p.backup_result_grace_s
        return [BackUp(g.seq, g.distance_m, g.speed_mps, allowance)]

    def _start_reverse_path(self, g: _Retreat, now: float, pose: Sequence[float] | None) -> list:
        if pose is not None and self._too_short(g.route, pose):
            return self._end_active(FAILED, stop=True)
        s = g.route.project(pose[:2])[0] if pose is not None else 0.0
        rest = g.route.rest(s)
        self._seq += 1
        speed = min(g.speed_mps, self.p.reverse_speed_mps)
        allowance = Polyline(rest).length / speed * 2.0 + 2.0
        g.seq, g.since, g.allowance_s = self._seq, now, allowance
        self._nav_running = True
        self._nav_live[g.seq] = math.inf
        poses = poses_backing(rounded_rest(g.route, s, self.p.turn_radius_m), self.p.path_step_m)
        return self._limit_speed() + [SendPath(g.seq, poses, final_goal=True, reverse=True)]

    def _too_short(self, route: Polyline, pose: Sequence[float]) -> bool:
        return route.length - route.project(pose[:2])[0] < self.p.backup_min_m

    def nav_cancel_accepted(self, seq: int, now: float) -> list:
        if self._nav_live.get(seq) == math.inf:
            self._nav_live[seq] = now + self.p.nav_result_grace_s
        return []

    def nav_lost(self, seq: int, now: float, pose: Sequence[float] | None,
                 obstacle: Obstacle) -> list:
        until = self._nav_live.get(seq)
        cmds = self.nav_result(seq, "aborted", now, pose, obstacle)
        if until is not None:
            self._nav_live[seq] = until
        return cmds

    def _nav_busy(self, now: float) -> bool:
        for seq, until in list(self._nav_live.items()):
            if now >= until:
                del self._nav_live[seq]
        return bool(self._nav_live)

    def _backup_busy(self, now: float) -> bool:
        for seq, until in list(self._backup_unanswered.items()):
            if now >= until:
                del self._backup_unanswered[seq]
        return bool(self._backup_unanswered)
