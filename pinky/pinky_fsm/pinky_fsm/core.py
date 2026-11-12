"""핑키 FSM의 판단. ROS를 모르고, 입력을 받아 명령 목록을 돌려준다.

규칙의 출처
- 안내 = Drive 핑키_FSM_Nav2_구현_안내_2026-11-07.md (2.1 교체 때 서지 않음, 2.2 연결 확인)
- 계약 = docs/핑키_계약.md (3절 지킬 것, 4절 BLOCKED)
- R번호 = Drive 핑키_제어파트_전달사항_2026-11-06 3절
- 자가 복구 = 핑키_계약 3절 7번(2026-11-09 제안): 막히면 쥔 구역 안에서 짧게 물러나고 지도
  장애물을 비운 뒤 같은 선을 다시 따른다. 두 번까지 하고, 그동안 RECOVERING으로 보고한다.
- 물러나기 = FMS 개선 계획 버전 2 1-A(계약 0.9-draft 제안): 관제의 Retreat goal. 이동을 먼저
  멈추고(둘 다 cmd_vel) 자가 복구와 같은 BackUp을 낸다. 결과는 backup.reason_of로 만든다.
  path가 있으면(자유바닥, 계약 0.9-draft 제안) BackUp 대신 그 경로를 후진 controller로 보낸다
  (SendPath reverse).
- nav_goal = MoveTo mode nav_goal(계약 0.11-draft 제안): Nav2 NavigateToPose로 간다. 판단은
  nav_goal.py에 있고 여기는 잇기만 한다. 다른 Nav2 서버(FollowPath, BackUp)의 움직임과 바꿀 때는
  앞 움직임을 멈추고 결과가 오고 바퀴가 설 때까지 기다린다(물러나기와 같은 규칙).

시각은 모두 벽시계 초다(heartbeat는 벽시계로 재야 하고, 나머지도 같은 축을 쓴다).
노드는 SendPath를 Nav2 FollowPath 목표로, CancelNav를 그 취소로, Finish를 액션 결과로 옮긴다.
"""
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
from pinky_fsm.geometry import Point, Polyline, Pose, poses_along, rounded_rest
from pinky_fsm.local_yield import NO_PRIORITY, Peer
from pinky_fsm.nav_goal import MODE_NAV_GOAL, MODES, NavGoal, NavGoals
from pinky_fsm.nav_moves import NavGoalMoves
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


class PinkyFsm(NavGoalMoves, RetreatMoves, LocalYield, NavYield, SelfRecovery):
    def __init__(self, params: FsmParams, robot_id: str = ""):
        self.p = params
        self.robot_id = robot_id  # 로봇단 교착 처리의 이름 순(7-4)
        self._priority = NO_PRIORITY  # 지금 MoveTo의 priority(7-4)
        self._peers: list[Peer] = []  # 관제 /fleet/robots의 다른 로봇들
        self._peers_at = -math.inf
        self._active: _Move | _Dock | _Retreat | NavGoal | None = None
        self._goals = NavGoals(params.blocked_after_s, params.nav_progress_m,
                               params.nav_result_grace_s)
        self._rest = IDLE
        self._localized = False
        self._pose: Sequence[float] | None = None  # 마지막으로 받은 자세
        self._seq = 0
        # A FollowPath may still be out there: one was sent since the last CancelNav. The latest
        # result does not clear it, since a replaced one may be accepted late.
        self._nav_running = False
        # 마지막으로 낸 LimitSpeed. 처음에는 모른다: 재시작 전에 건 제한을 첫 tick에 푼다
        self._speed_limit: float | None = None
        # 결과가 아직 안 온 FollowPath(seq → 이 시각 뒤에는 잊는다). 물러나기 전에 비운다. 도는
        # 동안은 끝이 없고(inf), 물러나기가 취소하면 그때부터 nav_result_grace_s다
        self._nav_live: dict[int, float] = {}
        self._alive = -math.inf  # 마지막 heartbeat나 목표 수락
        self._lost_contact = False  # 이번 tick에 heartbeat 끊김으로 끝냈다
        self._stopped_since: float | None = None
        self._blocked = False
        self._clear_since: float | None = None
        self._blocked_points: list[Point] | None = None  # 목표가 끝난 뒤에도 지켜볼 앞 구간
        self._recovery: _Recovery | None = None
        self._attempts = 0  # 쓴 자가 복구 시도 수. 목표가 바뀌어도 남는다
        self._attempt_origin: Point | None = None  # 첫 시도를 시작한 자리
        self._bseq = 0
        self._backup_running = False  # 보낸 BackUp이 아직 끝나지 않았을 수 있다
        # 결과가 아직 안 온 BackUp(순번 → 이 시각 뒤에는 nav2가 time_allowance로 이미 세웠다).
        # 목표가 바뀌거나 끝나도 남는다: 취소 요청은 그 BackUp이 섰다는 뜻이 아니다
        self._backup_unanswered: dict[int, float] = {}

    # -- 보고 -----------------------------------------------------------------------------

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

    # -- 입력 -----------------------------------------------------------------------------

    def heartbeat(self, now: float) -> None:
        self._alive = max(self._alive, now)

    def move(self, goal_id: Hashable, now: float, target_id: str, target: Pose,
             path: Sequence[Point], pose: Sequence[float] | None,
             final_goal: bool = True, mode: str = "", priority: int = NO_PRIORITY) -> list:
        self._alive = max(self._alive, now)  # 목표 수락도 관제가 살아 있다는 증거(계약 3절 5번)
        self._priority = int(priority)
        if pose is None or mode not in MODES:
            return self._end_active(CANCELED, stop=True) + [Finish(goal_id, False, NAV_ABORTED)]
        if mode == MODE_NAV_GOAL:
            return self._move_nav(goal_id, now, target_id, target, pose)
        here = (float(pose[0]), float(pose[1]))
        tx, ty, tyaw = (float(v) for v in target)
        if path:
            points = [(float(p[0]), float(p[1])) for p in path]
            # 받은 path로 판정한다. target까지 붙이는 선이 로봇을 지나도 연결된 것이 아니다
            _, off = Polyline(points).project(here)
            if off > self.p.connect_tolerance_m:
                # 안내 2.2: 따라가지 않고 실패. 이전 목표도 끝나므로 선다
                return (self._end_active(CANCELED, stop=True)
                        + [Finish(goal_id, False, PATH_NOT_CONNECTED)])
            if math.dist(points[-1], (tx, ty)) > 1e-6:
                points.append((tx, ty))
        else:
            points = [here, (tx, ty)]  # 점 목표: 지금 자리에서 곧게
        # 안내 2.1: 서지 않고 새 경로로. 단 NavigateToPose가 돌 수 있으면 멈추고 끝나기를 기다린다
        # (BT의 FollowPath와 이 FollowPath가 같은 controller를 두고 다툰다)
        handover = self._goals.busy(now)
        cmds = self._end_active(CANCELED, stop=handover)
        g = _Move(goal_id, target_id, (tx, ty, tyaw), Polyline(points), final_goal=final_goal)
        g.s, _ = g.route.project(here)  # 가장 가까운 점부터(계약 2절)
        g.s_start = g.s
        self._active = g
        self._stopped_since = None
        if handover:
            g.handover_since = now
            return cmds
        return cmds + self._send_rest(g, now)

    def dock(self, goal_id: Hashable, now: float, dock_id: str) -> list:
        """pinky_dock이 아직 없다. 도킹을 했다고 꾸미지 않고 실패로 끝낸다."""
        self._alive = max(self._alive, now)
        cmds = self._end_active(CANCELED, stop=True)
        return cmds + [Finish(goal_id, False, DOCK_UNAVAILABLE)]

    def cancel(self, goal_id: Hashable, now: float) -> list:
        if not self.is_running(goal_id):
            return []
        return self._end_active(CANCELED, stop=True)

    def nav_result(self, seq: int, outcome: str, now: float, pose: Sequence[float] | None,
                   obstacle: Obstacle, error_code: int = 0) -> list:
        """FollowPath 결과. outcome은 succeeded, aborted, canceled, rejected 중 하나.
        error_code는 FollowPath 결과의 것이다(후진 경로 물러나기의 reason에만 쓴다)."""
        self._nav_live.pop(seq, None)
        g = self._active
        if isinstance(g, _Retreat):
            if g.route is None or g.seq == 0 or seq != g.seq:
                return []  # 멈춘 이동의 결과
            self._active = None
            self._rest = IDLE
            return ([Finish(g.goal_id, outcome == "succeeded", reason_of(outcome, error_code))]
                    + self._end_active("", stop=True))
        if not isinstance(g, _Move) or seq != g.seq:
            return []  # 교체된 옛 목표의 결과(Nav2는 새 목표를 받으면 옛 것을 ABORTED로 끝낸다)
        g.nav_running = False
        if self._recovery is not None and (self._recovery.backup_seq is not None
                                           or self._recovery.wait_until is not None):
            return []  # 물러나거나 기다리려고 취소한 FollowPath의 결과
        # Every end below stops Nav2: a replacement does not cancel the previous FollowPath
        # (guide 2.1), so an older one may still run or be accepted late.
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
            return self._start_attempt(g, now, pose)  # RPP가 스스로 선 것도 막힌 것이다
        self._recovery = None  # 시도를 다 썼다. 아래는 자가 복구가 없던 때의 동작이다
        if pose is not None and obstacle(self._ahead(g, pose)):
            g.waiting_since = now  # 계약 4절: wait while blocked, up to block_timeout_s
            if g.blocked_since is None:
                g.blocked_since = now
            return []
        return self._end_active(NAV_ABORTED, stop=True)

    def backup_result(self, seq: int, outcome: str, now: float,
                      pose: Sequence[float] | None, error_code: int = 0) -> list:
        """BackUp 결과. 자가 복구면 어떤 결과든(뒤에 물체, 시간 초과 포함) 지도를 비우고 남은
        선을 다시 따른다. 물러나기면 그 결과로 goal을 끝낸다."""
        self._backup_unanswered.pop(seq, None)
        r, g = self._recovery, self._active
        if isinstance(g, _Retreat):
            if g.route is not None or g.seq != seq:
                return []  # 경로 물러나기의 seq는 FollowPath 순번이다
            self._backup_running = False
            self._active = None
            self._rest = IDLE
            return [Finish(g.goal_id, outcome == "succeeded", reason_of(outcome, error_code))]
        if r is not None and r.local and r.backup_seq == seq and isinstance(g, NavGoal):
            return self._nav_backup_result(g, now, pose)  # 로봇단 교착 처리(7-4)
        if r is None or r.backup_seq != seq or not isinstance(g, _Move):
            return []  # 멈춘 BackUp이나 끝난 목표의 결과
        self._backup_running = False
        return self._follow_again(g, r, now, pose)

    def tick(self, now: float, pose: Sequence[float] | None, speed: tuple[float, float],
             obstacle: Obstacle) -> list:
        """주기 판단. speed = (선속도 m/s, 각속도 rad/s)."""
        self._localized = pose is not None
        self._lost_contact = False
        if pose is not None:
            self._pose = pose
        cmds: list = []
        g = self._active
        if g is not None and self._heartbeat_lost(now) and self._stops_on_heartbeat_loss():
            self._lost_contact = True  # order_fsm가 Retreat의 FAILED 대신 HEARTBEAT_LOST로 끝낸다
            cmds += self._end_active(HEARTBEAT_LOST, stop=True)
            g = None
        if g is not None and pose is None:
            cmds += self._end_active(NAV_ABORTED, stop=True)  # never drive without a pose
            g = None
        if isinstance(g, _Move) and pose is not None:
            g.s = self._track(g, pose)
        if self._handing_over(g):
            cmds += self._hand_over(now, g, pose, speed)
            if self._handing_over(self._active):
                return cmds + self._limit_speed()  # 아직 앞 움직임이 끝나지 않았다
            g = self._active
        if isinstance(g, NavGoal):
            self._goals.watch(g, now, pose)
            if self.p.local_yield:
                cmds += self._watch_nav_yield(now, g, pose, speed)
        if isinstance(g, _Retreat) and g.seq == 0:
            if self._motion_ended(now, speed):
                cmds += self._start_retreat(g, now, pose)
            elif now - g.since >= g.allowance_s:
                cmds += self._end_active(TIMEOUT, stop=True)  # 서지 않는다. 물러나지 않는다
                g = None
        elif (isinstance(g, _Retreat)
                and now - g.since >= g.allowance_s + self.p.backup_result_grace_s):
            cmds += self._end_active(TIMEOUT, stop=True)  # BackUp 결과가 오지 않는다
            g = None
        cmds += self._watch_recovery(now, g, pose, obstacle)
        g = self._active
        cmds += self._watch_obstacle(now, g, pose, speed, obstacle)  # a clear comes first
        g = self._active
        if (isinstance(g, _Move) and g.blocked_since is not None
                and now - g.blocked_since >= self.p.block_timeout_s):
            cmds += self._end_active(TIMEOUT, stop=True)  # cannot go on; stop Nav2 if it runs
            g = None
        if (isinstance(g, _Move) and g.nav_running and pose is not None
                and g.route.length - g.s < self.p.refresh_within_m
                and now - g.last_send >= self.p.refresh_every_s):
            cmds += self._send_rest(g, now, pose)  # R13: 끝 근처 재전송
        return cmds + self._limit_speed()

    # -- 안쪽 -----------------------------------------------------------------------------

    def _heartbeat_lost(self, now: float) -> bool:
        return self.p.heartbeat_timeout_s > 0 and now - self._alive >= self.p.heartbeat_timeout_s

    def _stops_on_heartbeat_loss(self) -> bool:
        """heartbeat가 끊기면 목표를 끝내고 선다(계약 3절 5번). order_fsm은 base 끝까지 간다."""
        return True

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
        if moving_goal:
            points = self._ahead(g, pose)
        elif self._blocked and self._blocked_points:
            points = self._blocked_points  # 목표가 끝난 뒤에도 그 경로 앞을 본다
        else:
            self._blocked, self._clear_since = False, None
            return []
        waiting = moving_goal and g.waiting_since is not None
        stood = self._stopped_since is not None and now - self._stopped_since >= \
            self.p.block_report_s
        if not (self._blocked or waiting or stood):
            return []
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

    # -- 자가 복구 -------------------------------------------------------------------------

    def _track(self, g: _Move, pose: Sequence[float]) -> float:
        s, _ = g.route.project(pose[:2], s_min=g.s - self.p.track_back_m,
                               s_max=g.s + self.p.track_ahead_m)
        return s

    def _ahead(self, g: _Move, pose: Sequence[float]) -> list[Point]:
        s = self._track(g, pose)
        return g.route.sample(s, s + self.p.block_lookahead_m, self.p.block_sample_m)

    def _limit_speed(self) -> list:
        """잠금 경계에서 끝나는 목표만 설 수 있는 속도로 제한한다. 바뀔 때만 낸다."""
        g = self._active
        limit = 0.0
        if (isinstance(g, _Retreat) and g.route is not None
                and g.speed_mps < self.p.reverse_speed_mps):
            limit = g.speed_mps
        if isinstance(g, _Move) and not g.final_goal:
            left = max(0.0, g.route.length - g.s)
            # the robot may cover this much at full speed before the limit acts
            left = max(0.0, left - self.p.horizon_cap_mps * self.p.horizon_latency_s)
            v = max(self.p.horizon_min_mps, math.sqrt(2.0 * self.p.horizon_decel_mps2 * left))
            # 내려서 자른다. 올려 반올림하면 √(2·a·d)를 넘는다
            limit = math.floor(v * 100.0) / 100.0 if v < self.p.horizon_cap_mps else 0.0
        if limit == self._speed_limit:
            return []
        self._speed_limit = limit
        return [LimitSpeed(limit)]

    def _send_rest(self, g: _Move, now: float, pose: Sequence[float] | None = None) -> list:
        if pose is not None:
            g.s = self._track(g, pose)
        self._seq += 1
        g.seq = self._seq
        g.nav_running = True
        g.last_send = now
        self._nav_running = True
        self._nav_live[g.seq] = math.inf
        rest = rounded_rest(g.route, g.s, self.p.turn_radius_m)  # 꺾이는 곳은 호로(2-1)
        poses = poses_along(rest, g.target[2], self.p.path_step_m)
        return [SendPath(g.seq, poses, final_goal=g.final_goal)]

    def _end_active(self, reason: str, stop: bool) -> list:
        g = self._active
        cmds: list = []
        if g is not None:
            if isinstance(g, _Move) and self._blocked and self._pose is not None:
                # 계약 4절: 목표가 끝나도 물체가 그 경로 앞에 있는 동안은 BLOCKED다
                self._blocked_points = self._ahead(g, self._pose)
            self._active = None
            self._rest = IDLE
            if isinstance(g, _Retreat) and reason not in RETREAT_REASONS:
                reason = FAILED  # 계약 0.9: Retreat reason은 넷뿐이다(heartbeat, 위치 잃음 등)
            cmds.append(Finish(g.goal_id, False, reason))
        if stop and self._nav_running:
            self._nav_running = False
            cmds.append(CancelNav())
        if stop:
            cmds += self._goals.stop()
        self._recovery = None
        if self._backup_running:  # 새 목표로 바뀔 때도 멈춘다: 둘 다 cmd_vel을 낸다
            self._backup_running = False
            cmds.append(CancelBackUp())
        return cmds
