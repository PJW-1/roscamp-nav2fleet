"""PinkyFsm의 장애물 감시(계약 4절). ROS를 모른다.

core.PinkyFsm이 이 클래스를 물려받는다(core.py가 길어 책임별로 나눴다). 서 있는 시간을 재고 앞
경로에 물체가 있는지 보아 자가 복구(recovery.py)를 시작하거나 BLOCKED로 넘긴다.
"""
from __future__ import annotations

from collections.abc import Sequence

from pinky_fsm.geometry import Point, rounded_at
from pinky_fsm.records import _Move
from pinky_fsm.recovery import Obstacle


class ObstacleWatch:
    def _watch_obstacle(self, now, g, pose, speed, obstacle: Obstacle) -> list:
        """계약 4절. 움직이는 목표 중 1초 서 있고 앞 0.5 m에 물체가 있으면 자가 복구를 하고,
        시도를 다 썼으면 BLOCKED."""
        r = self._recovery
        if r is not None and (r.backup_seq is not None or r.wait_until is not None):
            self._stopped_since = None  # 물러나거나 기다리는 중이다(7-4). 끝나고 다시 잰다
            return []
        lin, ang = speed
        stopped = abs(lin) < self.p.stopped_speed_mps and abs(ang) < self.p.stopped_turn_rps
        moving_goal = isinstance(g, _Move)
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
        waiting = moving_goal and g.waiting_since is not None
        stood = self._stopped_since is not None and now - self._stopped_since >= \
            self.p.block_report_s
        if not (self._blocked or waiting or stood):
            return []
        # 목표가 끝난 뒤에도 그 경로 앞을 본다
        points = self._ahead(g, pose) if moving_goal else self._blocked_points
        if obstacle(points):
            self._clear_since = None
            if stood and moving_goal and not self._blocked and self._can_recover(pose):
                return self._start_attempt(g, now, pose)
            if stood:
                self._recovery = None  # 두 번 다 썼다. 계약 4절의 BLOCKED로 넘긴다
                self._blocked = True
            if moving_goal and self._blocked and g.blocked_since is None:
                g.blocked_since = now  # Nav2 may hold the goal open while the robot stands
            return []
        if self._clear_since is None:
            self._clear_since = now
        if now - self._clear_since < self.p.block_clear_s:
            return []
        self._blocked, self._clear_since, self._blocked_points = False, None, None
        if moving_goal:
            g.blocked_since = None
        if waiting:
            g.waiting_since = None
            return self._send_rest(g, now, pose)  # 물체가 비켰다. 남은 선을 다시 보낸다
        return []

    def _track(self, g: _Move, pose: Sequence[float]) -> float:
        s, _ = g.route.project(pose[:2], s_min=g.s - self.p.track_back_m,
                               s_max=g.s + self.p.track_ahead_m)
        return s

    def _ahead(self, g: _Move, pose: Sequence[float]) -> list[Point]:
        """로봇 앞의 길. Nav2에 보낸 선 위에서 본다: 호와 선에 붙는 조각은 받은 선과 다르다."""
        line = g.sent
        if line is None:  # 아직 보내지 않았다(앞 goal이 끝나기를 기다린다)
            s = self._track(g, pose)
            return g.route.sample(s, s + self.p.block_lookahead_m, self.p.block_sample_m)
        # 보낸 선 위의 자리는 진행 거리를 둥근 선 위의 거리로 바꾼 곳 근처다. 그 둘레에서만 찾아야
        # 같은 곳을 두 번 지나는 선에서 이미 지난 쪽을 고르지 않는다. 선과 만나기 전이면 붙인 조각
        # 위다: 그 조각은 대신하는 선보다 길 수 있어 조각 전체에서 찾는다
        gone = rounded_at(g.route, self._track(g, pose), self.p.turn_radius_m) - g.sent_from
        at = min(g.sent_lead + max(gone, 0.0), line.length)
        low = 0.0 if gone < 0.0 else at - self.p.track_back_m
        s, _ = line.project(pose[:2], s_min=low, s_max=at + self.p.track_back_m)
        return line.sample(s, s + self.p.block_lookahead_m, self.p.block_sample_m)
