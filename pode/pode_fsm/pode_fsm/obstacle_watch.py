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
        if self._hold_at is not None:
            self._watch_hold(now, g, pose, obstacle)
        if not moving_goal and not (self._blocked and self._blocked_points):
            self._blocked, self._clear_since = False, None
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
        # ---- 장애물이 사라진 채 유지되면 막힘 해제와 기다리던 경로 다시 보내기 ----
        if self._clear_since is None:
            self._clear_since = now
        if now - self._clear_since < self.p.block_clear_s:
            return []
        if self._blocked:
            # 보고만 BLOCKED로 남긴다(판단에는 쓰지 않음)
            self._hold_at, self._hold_since = self._blocked_at, self._clear_since
        self._blocked, self._clear_since, self._blocked_points = False, None, None
        if moving_goal:
            g.blocked_since = None
        if waiting:
            g.waiting_since = None
            return self._send_rest(g, now, pose)
        return []

    # ---- 막힘 표시와 보고용 막힘 유지(hold) ----
    def _mark_blocked(self, pose) -> None:
        if not self._blocked and pose is not None:
            self._blocked_at = (float(pose[0]), float(pose[1]))
        self._blocked = True
        self._drop_hold()

    def _drop_hold(self) -> None:
        self._hold_at = self._hold_since = None

    def _watch_hold(self, now, g, pose, obstacle) -> None:
        # 막힌 자리에서 unblock_m 벗어나면 해제(보고만, 정지 시각은 건드리지 않음)
        if math.dist(pose[:2], self._hold_at) >= self.p.unblock_m:
            self._drop_hold()
            return
        # 경로를 달릴 때만 물체를 본다. 목표가 없으면 시간으로만 끝낸다
        if isinstance(g, Move) and obstacle(self._ahead(g, pose)):
            self._hold_since = None
            return
        if self._hold_since is None:
            self._hold_since = now
        if now - self._hold_since >= self.p.unblock_clear_s:
            self._drop_hold()

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
