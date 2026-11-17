# ---- 주행 중 장애물로 막힘을 감시하는 믹스인 ----
from __future__ import annotations

import math
from collections.abc import Sequence

from pode_fsm.geometry import Point, rounded_at
from pode_fsm.records import Move
from pode_fsm.recovery import Obstacle


# ---- 정지와 장애물로 막힘 판정과 해제 ----
class ObstacleWatch:
    def _watch_obstacle(self, now, g, pose, speed, obstacle: Obstacle) -> list:
        r = self._recovery
        if r is not None and (r.backup_seq is not None or r.wait_until is not None):
            self._stopped_since = None
            return []
        # ---- 정지 시작 시각 기록 ----
        lin, ang = speed
        stopped = abs(lin) < self.p.stopped_speed_mps and abs(ang) < self.p.stopped_turn_rps
        moving_goal = isinstance(g, Move)
        if moving_goal and stopped:
            if self._stopped_since is None:
                self._stopped_since = now
        else:
            self._stopped_since = None
        if pose is None:
            return []
        if not moving_goal and not (self._blocked and self._blocked_points):
            self._blocked, self._clear_since = False, None
            return []
        # ---- 막힌 자리에서 분명히 벗어나면 막힘 해제 ----
        if self._blocked and self._moved_off(pose):
            self._unblock(g)
            if not moving_goal:
                return []
        waiting = moving_goal and g.waiting_since is not None
        stood = self._stopped_since is not None and now - self._stopped_since >= \
            self.p.block_report_s
        if not (self._blocked or waiting or stood):
            return []
        # ---- 장애물이 보이면 복구 시도 또는 막힘 표시 ----
        points = self._ahead(g, pose) if moving_goal else self._blocked_points
        if obstacle(points):
            self._clear_since = None
            if stood and moving_goal and not self._blocked and self._can_recover(pose):
                return self._start_attempt(g, now, pose)
            if stood:
                self._recovery = None
                self._mark_blocked(pose)
            if moving_goal and self._blocked and g.blocked_since is None:
                g.blocked_since = now
            return []
        # ---- 장애물이 block_clear_s 안 보이면 기다리던 경로 다시 보내기 ----
        if self._clear_since is None:
            self._clear_since = now
        gone = now - self._clear_since
        if gone < self.p.block_clear_s:
            return []
        cmds: list = []
        if moving_goal:
            g.blocked_since = None
        if waiting:
            g.waiting_since = None
            cmds = self._send_rest(g, now, pose)
        # ---- 막힘은 unblock_clear_s 동안 계속 안 보여야 해제 ----
        if not self._blocked or gone >= self.p.unblock_clear_s:
            self._unblock(g)
        return cmds

    # ---- 막힘 표시와 해제: 막힌 자리를 기억해 분명한 진전만 해제로 본다 ----
    def _mark_blocked(self, pose) -> None:
        if not self._blocked and pose is not None:
            self._blocked_at = (float(pose[0]), float(pose[1]))
        self._blocked = True

    def _moved_off(self, pose) -> bool:
        if self._blocked_at is None:
            self._blocked_at = (float(pose[0]), float(pose[1]))
            return False
        return math.dist(pose[:2], self._blocked_at) >= self.p.unblock_m

    def _unblock(self, g) -> None:
        self._blocked, self._clear_since, self._blocked_points = False, None, None
        self._blocked_at = None
        if isinstance(g, Move):
            g.blocked_since = None

    # ---- 경로 위 현재 위치와 앞쪽 검사 점 계산 ----
    def _track(self, g: Move, pose: Sequence[float]) -> float:
        s, _ = g.route.project(pose[:2], s_min=g.s - self.p.track_back_m,
                               s_max=g.s + self.p.track_ahead_m)
        return s

    def _ahead(self, g: Move, pose: Sequence[float]) -> list[Point]:
        line = g.sent
        if line is None:
            s = self._track(g, pose)
            return g.route.sample(s, s + self.p.block_lookahead_m, self.p.block_sample_m)
        gone = rounded_at(g.route, self._track(g, pose), self.p.turn_radius_m) - g.sent_from
        at = min(g.sent_lead + max(gone, 0.0), line.length)
        low = 0.0 if gone < 0.0 else at - self.p.track_back_m
        s, _ = line.project(pose[:2], s_min=low, s_max=at + self.p.track_back_m)
        return line.sample(s, s + self.p.block_lookahead_m, self.p.block_sample_m)
