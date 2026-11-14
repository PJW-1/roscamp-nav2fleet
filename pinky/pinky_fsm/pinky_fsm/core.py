# ---- 핑키 상태 기계 본체 PinkyFsm, ROS 없이 판단만 한다 ----
from __future__ import annotations

import math
from collections.abc import Hashable, Sequence

from pinky_fsm.backup import FAILED, CancelBackUp, reason_of
from pinky_fsm.commands import (  # noqa: F401 - 명령 이름은 core에서도 내보낸다
    CancelNav,
    ClearCostmap,
    LimitSpeed,
    SendPath,
)
from pinky_fsm.geometry import Point, Polyline, Pose, joined_rest, poses_along
from pinky_fsm.local_yield import NO_PRIORITY, Peer
from pinky_fsm.nav_goal import MODE_NAV_GOAL, MODES, NavGoal, NavGoals
from pinky_fsm.nav_moves import NavGoalMoves
from pinky_fsm.obstacle_watch import ObstacleWatch
from pinky_fsm.params import FsmParams
from pinky_fsm.records import _Dock, _Move, _Recovery
from pinky_fsm.recovery import Obstacle, SelfRecovery
from pinky_fsm.results import (  # noqa: F401 - 상태와 결과 이름은 core에서도 내보낸다
    BLOCKED,
    CANCELED,
    DOCK_UNAVAILABLE,
    DOCKED,
    DOCKING,
    HEARTBEAT_LOST,
    IDLE,
    LOCALIZING,
    MOVING,
    NAV_ABORTED,
    PATH_NOT_CONNECTED,
    RECOVERING,
    TIMEOUT,
    Finish,
)
from pinky_fsm.retreat import RETREAT_REASONS, RetreatMoves, _Retreat
from pinky_fsm.yield_moves import LocalYield
from pinky_fsm.yield_nav import NavYield


# ---- 믹스인을 묶은 PinkyFsm과 내부 상태 초기화 ----
class PinkyFsm(NavGoalMoves, RetreatMoves, LocalYield, NavYield, SelfRecovery, ObstacleWatch):
    def __init__(self, params: FsmParams, robot_id: str = ""):
        self.p = params
        self.robot_id = robot_id
        self._priority = NO_PRIORITY
        self._peers: list[Peer] = []
        self._peers_at = -math.inf
        self._active: _Move | _Dock | _Retreat | NavGoal | None = None
        self._goals = NavGoals(params.blocked_after_s, params.nav_progress_m,
                               params.nav_result_grace_s, params.nav_turn_progress_rad,
                               params.nav_turn_budget_rad)
        self._rest = IDLE
        self._localized = False
        self._pose: Sequence[float] | None = None
        self._seq = 0
        self._nav_running = False
        self._speed_limit: float | None = None
        self._nav_live: dict[int, float] = {}
        self._alive = -math.inf
        self._lost_contact = False
        self._stopped_since: float | None = None
        self._blocked = False
        self._clear_since: float | None = None
        self._blocked_points: list[Point] | None = None
        self._recovery: _Recovery | None = None
        self._attempts = 0
        self._attempt_origin: Point | None = None
        self._bseq = 0
        self._backup_running = False
        self._backup_unanswered: dict[int, float] = {}

    # ---- 상태, 현재 목표, 남은 거리 조회와 하트비트 기록 ----
    def state(self) -> str:
        if not self._localized:
            return LOCALIZING
        if self._recovery is not None:
            return RECOVERING
        if self._blocked or (isinstance(self._active, NavGoal) and self._active.blocked):
            return BLOCKED
        if isinstance(self._active, (_Move, _Retreat, NavGoal)):
            return MOVING
        if isinstance(self._active, _Dock):
            return DOCKING
        return self._rest

    def current_goal(self) -> str:
        if isinstance(self._active, (_Move, NavGoal)):
            return self._active.target_id
        if isinstance(self._active, _Dock):
            return self._active.dock_id
        return ""

    def distance_remaining(self, pose: Sequence[float] | None) -> float:
        g = self._active
        if isinstance(g, NavGoal):
            return NavGoals.distance_remaining(g, pose)
        if not isinstance(g, _Move):
            return 0.0
        s = self._track(g, pose) if pose is not None else g.s
        return max(0.0, g.route.length - s)

    def is_running(self, goal_id: Hashable) -> bool:
        return self._active is not None and self._active.goal_id == goal_id

    @property
    def recovery_attempts_used(self) -> int:
        return self._attempts

    def heartbeat(self, now: float) -> None:
        self._alive = max(self._alive, now)

    # ---- 경로 이동 요청: 경로 연결 검사 후 _Move 시작 ----
    def move(self, goal_id: Hashable, now: float, target_id: str, target: Pose,
             path: Sequence[Point], pose: Sequence[float] | None,
             final_goal: bool = True, mode: str = "", priority: int = NO_PRIORITY) -> list:
        self._alive = max(self._alive, now)
        self._priority = int(priority)
        if pose is None or mode not in MODES:
            return self._end_active(CANCELED, stop=True) + [Finish(goal_id, False, NAV_ABORTED)]
        if mode == MODE_NAV_GOAL:
            return self._move_nav(goal_id, now, target_id, target, pose)
        here = (float(pose[0]), float(pose[1]))
        tx, ty, tyaw = (float(v) for v in target)
        if path:
            points = [(float(p[0]), float(p[1])) for p in path]
            _, off = Polyline(points).project(here)
            if off > self.p.connect_tolerance_m:
                return (self._end_active(CANCELED, stop=True)
                        + [Finish(goal_id, False, PATH_NOT_CONNECTED)])
            if math.dist(points[-1], (tx, ty)) > 1e-6:
                points.append((tx, ty))
        else:
            points = [here, (tx, ty)]
        handover = self._goals.busy(now)
        cmds = self._end_active(CANCELED, stop=handover)
        g = _Move(goal_id, target_id, (tx, ty, tyaw), Polyline(points), final_goal=final_goal)
        g.s, _ = g.route.project(here)
        g.s_start = g.s
        self._active = g
        self._stopped_since = None
        if handover:
            g.handover_since = now
            return cmds
        return cmds + self._send_rest(g, now, pose)

    # ---- 도킹과 취소 요청 처리 ----
    def dock(self, goal_id: Hashable, now: float, dock_id: str) -> list:
        self._alive = max(self._alive, now)
        cmds = self._end_active(CANCELED, stop=True)
        return cmds + [Finish(goal_id, False, DOCK_UNAVAILABLE)]

    def cancel(self, goal_id: Hashable, now: float) -> list:
        if not self.is_running(goal_id):
            return []
        return self._end_active(CANCELED, stop=True)

    # ---- FollowPath 결과 처리: 도착, 재전송, 복구, 대기, 실패 ----
    def nav_result(self, seq: int, outcome: str, now: float, pose: Sequence[float] | None,
                   obstacle: Obstacle, error_code: int = 0) -> list:
        self._nav_live.pop(seq, None)
        g = self._active
        if isinstance(g, _Retreat):
            if g.route is None or g.seq == 0 or seq != g.seq:
                return []
            self._active = None
            self._rest = IDLE
            return ([Finish(g.goal_id, outcome == "succeeded", reason_of(outcome, error_code))]
                    + self._end_active("", stop=True))
        if not isinstance(g, _Move) or seq != g.seq:
            return []
        g.nav_running = False
        if self._recovery is not None and (self._recovery.backup_seq is not None
                                           or self._recovery.wait_until is not None):
            return []
        if outcome == "rejected":
            return self._end_active(NAV_ABORTED, stop=True)
        if outcome == "succeeded":
            if pose is not None and math.dist(pose[:2], g.target[:2]) <= self.p.arrival_tolerance_m:
                self._active = None
                self._rest = IDLE
                self._attempts, self._attempt_origin = 0, None
                return [Finish(g.goal_id, True, "")] + self._end_active("", stop=True)
            g.far_successes += 1
            if g.far_successes > self.p.far_success_limit:
                return self._end_active(NAV_ABORTED, stop=True)
            return self._send_rest(g, now, pose)
        if outcome == "aborted" and not self._blocked and self._can_recover(pose):
            return self._start_attempt(g, now, pose)
        self._recovery = None
        if pose is not None and obstacle(self._ahead(g, pose)):
            g.waiting_since = now
            if g.blocked_since is None:
                g.blocked_since = now
            return []
        return self._end_active(NAV_ABORTED, stop=True)

    # ---- 후진(BackUp) 결과 처리 ----
    def backup_result(self, seq: int, outcome: str, now: float,
                      pose: Sequence[float] | None, error_code: int = 0) -> list:
        self._backup_unanswered.pop(seq, None)
        r, g = self._recovery, self._active
        if isinstance(g, _Retreat):
            if g.route is not None or g.seq != seq:
                return []
            self._backup_running = False
            self._active = None
            self._rest = IDLE
            return [Finish(g.goal_id, outcome == "succeeded", reason_of(outcome, error_code))]
        if r is not None and r.local and r.backup_seq == seq and isinstance(g, NavGoal):
            return self._nav_backup_result(g, now, pose)
        if r is None or r.backup_seq != seq or not isinstance(g, _Move):
            return []
        self._backup_running = False
        return self._follow_again(g, r, now, pose)

    # ---- 주기 처리(tick) ----
    def tick(self, now: float, pose: Sequence[float] | None, speed: tuple[float, float],
             obstacle: Obstacle) -> list:
        # ---- 위치 갱신과 하트비트 끊김, 위치 상실 처리 ----
        self._localized = pose is not None
        self._lost_contact = False
        if pose is not None:
            self._pose = pose
        cmds: list = []
        g = self._active
        if g is not None and self._heartbeat_lost(now) and self._stops_on_heartbeat_loss():
            self._lost_contact = True
            cmds += self._end_active(HEARTBEAT_LOST, stop=True)
            g = None
        if g is not None and pose is None:
            cmds += self._end_active(NAV_ABORTED, stop=True)
            g = None
        if isinstance(g, _Move) and pose is not None:
            g.s = self._track(g, pose)
        # ---- goal 넘겨받기와 NavGoal 감시, 로봇단 교착 처리 ----
        if self._handing_over(g):
            cmds += self._hand_over(now, g, pose, speed)
            if self._handing_over(self._active):
                return cmds + self._limit_speed()
            g = self._active
        if isinstance(g, NavGoal):
            self._goals.watch(g, now, pose)
            if self.p.local_yield:
                cmds += self._watch_nav_yield(now, g, pose, speed)
        # ---- 물러나기 시작과 시간 초과 확인 ----
        if isinstance(g, _Retreat) and g.seq == 0:
            if self._motion_ended(now, speed):
                cmds += self._start_retreat(g, now, pose)
            elif now - g.since >= g.allowance_s:
                cmds += self._end_active(TIMEOUT, stop=True)
                g = None
        elif (isinstance(g, _Retreat)
                and now - g.since >= g.allowance_s + self.p.backup_result_grace_s):
            cmds += self._end_active(TIMEOUT, stop=True)
            g = None
        # ---- 복구, 장애물, 막힘 시간 초과 감시와 남은 경로 재전송 ----
        cmds += self._watch_recovery(now, g, pose, obstacle)
        g = self._active
        cmds += self._watch_obstacle(now, g, pose, speed, obstacle)
        g = self._active
        if (isinstance(g, _Move) and g.blocked_since is not None
                and now - g.blocked_since >= self.p.block_timeout_s):
            cmds += self._end_active(TIMEOUT, stop=True)
            g = None
        if (isinstance(g, _Move) and g.nav_running and pose is not None
                and g.route.length - g.s < self.p.refresh_within_m
                and now - g.last_send >= self.p.refresh_every_s):
            cmds += self._send_rest(g, now, pose)
        return cmds + self._limit_speed()

    # ---- 하트비트 끊김 판정과 속도 제한 명령 만들기 ----
    def _heartbeat_lost(self, now: float) -> bool:
        return self.p.heartbeat_timeout_s > 0 and now - self._alive >= self.p.heartbeat_timeout_s

    def _stops_on_heartbeat_loss(self) -> bool:
        return True

    def _limit_speed(self) -> list:
        g = self._active
        limit = 0.0
        if (isinstance(g, _Retreat) and g.route is not None
                and g.speed_mps < self.p.reverse_speed_mps):
            limit = g.speed_mps
        if isinstance(g, _Move) and not g.final_goal:
            left = max(0.0, g.route.length - g.s)
            left = max(0.0, left - self.p.horizon_cap_mps * self.p.horizon_latency_s)
            v = max(self.p.horizon_min_mps, math.sqrt(2.0 * self.p.horizon_decel_mps2 * left))
            limit = math.floor(v * 100.0) / 100.0 if v < self.p.horizon_cap_mps else 0.0
        if limit == self._speed_limit:
            return []
        self._speed_limit = limit
        return [LimitSpeed(limit)]

    # ---- 남은 경로를 이어 SendPath 보내기 ----
    def _send_rest(self, g: _Move, now: float, pose: Sequence[float] | None = None) -> list:
        if pose is not None:
            g.s = self._track(g, pose)
        self._seq += 1
        g.seq = self._seq
        g.nav_running = True
        g.last_send = now
        self._nav_running = True
        self._nav_live[g.seq] = math.inf
        rest, g.sent_from, g.sent_lead = joined_rest(
            g.route, g.s, self.p.turn_radius_m, pose, self.p.join_min_off_m, self.p.join_slope)
        g.sent = Polyline(rest)
        poses = poses_along(rest, g.target[2], self.p.path_step_m)
        return [SendPath(g.seq, poses, final_goal=g.final_goal)]

    # ---- 현재 goal 종료와 주행, 후진 취소 ----
    def _end_active(self, reason: str, stop: bool) -> list:
        g = self._active
        cmds: list = []
        if g is not None:
            if isinstance(g, _Move) and self._blocked and self._pose is not None:
                self._blocked_points = self._ahead(g, self._pose)
            self._active = None
            self._rest = IDLE
            if isinstance(g, _Retreat) and reason not in RETREAT_REASONS:
                reason = FAILED
            cmds.append(Finish(g.goal_id, False, reason))
        if stop and self._nav_running:
            self._nav_running = False
            cmds.append(CancelNav())
        if stop:
            cmds += self._goals.stop()
        self._recovery = None
        if self._backup_running:
            self._backup_running = False
            cmds.append(CancelBackUp())
        return cmds
