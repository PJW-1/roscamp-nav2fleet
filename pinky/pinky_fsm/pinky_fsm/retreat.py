"""PinkyFsm의 물러나기(버전 2 1-A)와 결과가 안 온 FollowPath·BackUp의 기록. ROS를 모른다.

core.PinkyFsm이 이 클래스를 물려받는다(core.py가 길어 책임별로 나눴다). 여기 메서드는 core의
상태(_active, _bseq, FollowPath·BackUp 기록)를 그대로 쓴다.
"""
from __future__ import annotations

import math
from collections.abc import Hashable, Sequence
from dataclasses import dataclass

from pinky_fsm.backup import FAILED, OBSTACLE_BEHIND, BackUp
from pinky_fsm.commands import SendPath
from pinky_fsm.geometry import Polyline, poses_backing
from pinky_fsm.recovery import Obstacle
from pinky_fsm.results import CANCELED, TIMEOUT, Finish

RETREAT_REASONS = (OBSTACLE_BEHIND, TIMEOUT, CANCELED, FAILED)  # 계약 0.9 Retreat.action


@dataclass
class _Retreat:
    """관제가 시킨 물러나기(버전 2 1-A). BackUp 하나를 내고 그 결과로 끝난다. route가 있으면
    BackUp 대신 그 경로를 후진으로 따르는 FollowPath 하나다(seq는 FollowPath 순번).

    달리던 이동이나 결과가 안 온 BackUp이 있었으면 로봇이 설 때까지(stopped_speed) BackUp을
    미룬다(seq 0).
    """
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
        """뒤로 곧게 distance_m. path(점 (x, y, ...))가 있으면 그 경로를 후진으로 따른다.
        달리던 이동과 자가 복구 후진을 먼저 멈춘다."""
        self._alive = max(self._alive, now)
        # 결과가 안 온 BackUp도 기다린다. 같은 behavior_server라 바로 보내면 앞 후진이 다 서기
        # 전에 새 후진이 이어진다(V2 교차 검수 1)
        was_driving = self._driving(now)
        cmds = self._end_active(CANCELED, stop=True)
        route = None
        if path:
            route = Polyline([(float(q[0]), float(q[1])) for q in path])
            # MoveTo의 연결 확인(안내 2.2)과 같다: 로봇에서 시작하지 않는 경로는 따르지 않는다
            if (pose is None or route.length <= 0.0
                    or route.project(pose[:2])[1] > self.p.connect_tolerance_m
                    or self._too_short(route, pose)):
                return cmds + [Finish(goal_id, False, FAILED)]
        if not (distance_m > 0.0 and speed_mps > 0.0):
            return cmds + [Finish(goal_id, False, FAILED)]
        self._stopped_since = None
        if was_driving:
            # FollowPath나 BackUp이 끝나고(결과) 바퀴가 설 때까지 기다린다
            # (둘 다 cmd_vel, 1-A Codex 1)
            self._active = _Retreat(goal_id, 0, now, self.p.retreat_stop_wait_s,
                                    float(distance_m), float(speed_mps), route)
            return cmds
        self._active = _Retreat(goal_id, 0, now, 0.0, float(distance_m), float(speed_mps), route)
        return cmds + self._start_retreat(self._active, now, pose)

    def _start_retreat(self, g: _Retreat, now: float, pose: Sequence[float] | None) -> list:
        if g.route is not None:
            return self._start_reverse_path(g, now, pose)
        self._bseq += 1
        allowance = g.distance_m / g.speed_mps * 2.0 + 2.0  # 가속·감속과 멈칫거림까지 넉넉히
        g.seq, g.since, g.allowance_s = self._bseq, now, allowance
        self._backup_running = True
        self._backup_unanswered[g.seq] = now + allowance + self.p.backup_result_grace_s
        return [BackUp(g.seq, g.distance_m, g.speed_mps, allowance)]

    def _start_reverse_path(self, g: _Retreat, now: float, pose: Sequence[float] | None) -> list:
        """로봇에서 가장 가까운 점부터 끝까지 후진 FollowPath 하나를 보낸다."""
        if pose is not None and self._too_short(g.route, pose):
            return self._end_active(FAILED, stop=True)  # 기다리는 동안 끝에 닿았다
        s = g.route.project(pose[:2])[0] if pose is not None else 0.0
        rest = g.route.rest(s)
        self._seq += 1
        speed = min(g.speed_mps, self.p.reverse_speed_mps)
        allowance = Polyline(rest).length / speed * 2.0 + 2.0  # BackUp과 같은 여유
        g.seq, g.since, g.allowance_s = self._seq, now, allowance
        self._nav_running = True
        self._nav_live[g.seq] = math.inf
        poses = poses_backing(rest, self.p.path_step_m)
        # 남은 제한(잠금 경계 이동)을 먼저 바꾼다. RPP는 제한을 그대로 속도로 쓴다
        return self._limit_speed() + [SendPath(g.seq, poses, final_goal=True, reverse=True)]

    def _too_short(self, route: Polyline, pose: Sequence[float]) -> bool:
        """가장 가까운 점부터 남은 길이 backup_min_m보다 짧다. 점 하나짜리 FollowPath를 보내지
        않고 FAILED로 끝낸다(Codex 리뷰 3: 끝 근처의 로봇도 연결 허용 거리는 지난다)."""
        return route.length - route.project(pose[:2])[0] < self.p.backup_min_m

    def nav_cancel_accepted(self, seq: int, now: float) -> list:
        """FollowPath 취소 요청이 받아들여졌다. 그때부터 nav_result_grace_s 안에 결과가 안 와도
        그 goal은 잊는다(Codex 재검토 1). 취소가 거절되거나 답이 없으면 계속 기다리고, 그동안
        온 Retreat는 retreat_stop_wait_s 뒤 TIMEOUT이다. 서 있는 것만으로는 끝났다고 보지 않는다."""
        if self._nav_live.get(seq) == math.inf:
            self._nav_live[seq] = now + self.p.nav_result_grace_s
        return []

    def nav_lost(self, seq: int, now: float, pose: Sequence[float] | None,
                 obstacle: Obstacle) -> list:
        """FollowPath 결과 future가 오류로 끝났다. goal은 아직 돌 수 있다(Codex 재검토 2).
        실행 중인 목표에는 aborted처럼 다루되, 그 FollowPath는 노드가 보낸 취소가 받아들여질
        때까지(nav_cancel_accepted) 잊지 않는다."""
        until = self._nav_live.get(seq)
        cmds = self.nav_result(seq, "aborted", now, pose, obstacle)
        if until is not None:
            self._nav_live[seq] = until
        return cmds

    def _nav_busy(self, now: float) -> bool:
        """결과가 안 온 FollowPath가 아직 돌 수 있다. 취소가 받아들여진 뒤 여유가 지난 것은
        잊는다."""
        for seq, until in list(self._nav_live.items()):
            if now >= until:
                del self._nav_live[seq]
        return bool(self._nav_live)

    def _backup_busy(self, now: float) -> bool:
        """결과가 안 온 BackUp이 아직 움직이고 있을 수 있다. time_allowance(+ 여유)가 지난 것은
        nav2가 이미 세웠으므로 잊는다. 결과를 잃어도 다음 Retreat가 늘 TIMEOUT으로 끝나지 않는다."""
        for seq, until in list(self._backup_unanswered.items()):
            if now >= until:
                del self._backup_unanswered[seq]
        return bool(self._backup_unanswered)
