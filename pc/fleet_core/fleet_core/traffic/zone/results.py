# ---- goal 결과 처리 믹스인 ----
from __future__ import annotations

import math

from fleet_core.common.commands import CancelCmd
from fleet_core.fleet_map import NAV_GOAL
from fleet_core.traffic.mission import (
    AT_STATION,
    DOCK,
    GOAL_RESULT_LOST,
    HEARTBEAT_LOST,
    LOCALIZING,
    MOVE,
    NAV_ABORTED,
    NAV_UNAVAILABLE,
    NOT_AT_TARGET,
    PATH_NOT_CONNECTED,
    RETREAT,
    ROBOT_DOCKED,
    ROBOT_IDLE,
    ROBOT_TIMEOUT,
)

from .robots import ORDER, Goal
from .routes import along


# ---- 지금 goal과 맞는 로봇 찾기와 이동 결과 ----
class Results:
    def _active(self, robot, goal_id, *kinds):
        m = self._robots.get(robot)
        if m is None or m.goal is None or m.goal.id != goal_id or m.goal.kind not in kinds:
            return None
        return m

    def on_move_result(self, robot, goal_id, success, reason, last_done_seq=0) -> None:
        m = self._robots.get(robot)
        if m is not None and m.move_order is not None and self._old_goal_result(m, goal_id, reason):
            return
        m = self._active(robot, goal_id, MOVE)
        if m is None or self._past_unknown(m):
            return
        m.result_lost_at = None  # 늦게 온 결과가 결과 모름을 정한다
        if m.move_order is not None and self._order_result(m, reason, last_done_seq):
            return
        if success:
            self._succeeded(m)
        else:
            self._move_failed(m, reason)

    # ---- 물러나기 결과와 도킹 결과 ----
    def on_retreat_result(self, robot, goal_id, success, reason="") -> None:
        m = self._active(robot, goal_id, RETREAT)
        if m is None or self._past_unknown(m):
            return
        m.result_lost_at = None
        if success:
            m.avoid.behind_since = None
            self._succeeded(m)
        else:
            self._move_failed(m, reason or "RETREAT_FAILED")

    def on_dock_result(self, robot, goal_id, success, error_x_cm=0.0, error_y_cm=0.0,
                       error_yaw_deg=0.0) -> None:
        m = self._active(robot, goal_id, DOCK)
        if m is None or self._past_unknown(m):
            return
        self._clear_goal(m)
        if not success:
            self._dock_failed(m)
            return
        m.attempts = m.dock_tries = 0
        m.phase = AT_STATION
        m.dock_error_m = math.hypot(error_x_cm, error_y_cm) / 100.0
        self._event("INFO", "DOCKED", f"at {m.dest}: x {error_x_cm:.1f} cm, y {error_y_cm:.1f} cm,"
                    f" yaw {error_yaw_deg:.1f} deg", m.name)
        self._finish_dispatch(m, error_x_cm, error_y_cm, error_yaw_deg)

    # ---- goal 거절, 미전송, 잃음 처리 ----
    def on_goal_rejected(self, robot, goal_id) -> None:
        m = self._active(robot, goal_id, MOVE, RETREAT, DOCK)
        if m is None or self._update_unclear(m, "the goal was rejected or lost its result"):
            return
        if m.goal.kind == DOCK:
            self._clear_goal(m)
            self._dock_failed(m, "REJECTED")
        else:
            self._move_failed(m, "REJECTED")

    def on_goal_not_sent(self, robot, goal_id) -> None:
        m = self._active(robot, goal_id, MOVE, RETREAT, DOCK)
        if m is None or self._update_lost(m, "the goal was not sent"):
            return
        self._forget_goal(m)
        m.retry_at = self._clock() + self.params.move_retry_wait_s
        if not m.not_sent_warned:
            m.not_sent_warned = True
            self._event("WARN", "GOAL_NOT_SENT", "the action server is not ready, sends it again",
                        m.name)

    def on_goal_lost(self, robot, goal_id) -> None:
        mo = getattr(self._robots.get(robot), "move_order", None)
        if mo is not None and mo.before is not None and mo.before[0] == goal_id:
            mo.before_lost = goal_id  # 갱신이 실패해 되살리면 결과 모름으로 둔다
            return
        m = self._active(robot, goal_id, MOVE, RETREAT, DOCK)
        if m is None or self._update_unclear(m, "the goal was lost"):
            return
        if m.result_lost_at is None:
            self._lose_result(m, goal_id)

    def _lose_result(self, m, goal_id) -> None:
        # IR-62: 실패가 아니라 결과 모름. goal과 구역을 쥔 채 상태 보고로 정한다
        m.result_lost_at = self._clock()
        settle = self.params.robot_timeout_settle_s
        self._event("WARN", ROBOT_TIMEOUT,
                    f"{m.goal.kind} goal {goal_id} lost its result: result unknown, holds its "
                    f"zones, settles from the state report within {settle:.0f} s", m.name)

    # ---- 결과 모름 정하기: 끝에서 IDLE이면 완료, 다른 곳이면 실패, 도킹은 도크 자세로, 기한 넘으면 격리 ----
    def _settle_lost(self, m, now: float, state: str = "") -> None:
        """state는 지금 들은 상태 보고. 비면 기한만 본다."""
        if now - m.result_lost_at >= self.params.robot_timeout_settle_s:
            self._fail(m, ROBOT_TIMEOUT)
            self._cancel_goal(m)  # 이미 고장이던 로봇은 _fail이 취소하지 않는다
            return
        dock = m.goal.kind == DOCK
        idle = state == ROBOT_IDLE or (dock and state == ROBOT_DOCKED)
        if idle:
            self._out.append(CancelCmd(m.name, m.goal.id))  # 노드가 쥔 goal을 놓게 한다
        if idle and dock:
            error = self._dock_pose_error(m)
            if error is not None:
                # TODO(v0.2): pode_fsm이 DOCKED와 도킹 오차를 보고하면 그 값으로 정한다
                m.result_lost_at = None  # 기한은 이 보고를 받은 시각으로 이미 봤다
                self.on_dock_result(m.name, m.goal.id, True, *error)
            else:
                self._clear_goal(m)
                self._dock_failed(m, GOAL_RESULT_LOST)
        elif idle:
            self._move_failed(m, GOAL_RESULT_LOST)

    # ---- 보고 자세가 도크 자세 허용치 안이면 (x cm, y cm, 방향 deg) 오차, 밖이면 None ----
    def _dock_pose_error(self, m):
        spec, p = self.site.nodes[self.site.stations[m.dest].dock], self.params
        if spec.yaw is None:
            return None  # 지도에 도크 방향이 없으면 자세로 정하지 않는다
        ex, ey = m.x - spec.x, m.y - spec.y
        eyaw = math.degrees(math.remainder(m.yaw - spec.yaw, math.tau))
        if math.hypot(ex, ey) > p.lost_dock_reach_m or abs(eyaw) > p.lost_dock_yaw_deg:
            return None
        return ex * 100.0, ey * 100.0, eyaw

    # ---- 기다림이 끝난 결과 모름이면 먼저 정한다. 늦게 온 결과는 버린다 ----
    def _past_unknown(self, m, now: float | None = None) -> bool:
        now = self._clock() if now is None else now
        if m.result_lost_at is None or now - m.result_lost_at < self.params.robot_timeout_settle_s:
            return False
        self._settle_lost(m, now)
        return True

    # ---- 도착 확인과 구간 완료 ----
    def _succeeded(self, m) -> None:
        if self._gap(m, m.goal.end) <= self.params.arrival_tolerance_m:
            self._segment_done(m)
        else:
            m.arrival_by = self._clock() + self.params.arrival_settle_s
            m.heard_since_success = False

    def _judge_arrival(self, m, now: float) -> None:
        if m.arrival_by is None or m.goal is None:
            return
        if m.heard_since_success and self._gap(m, m.goal.end) <= self.params.arrival_tolerance_m:
            self._segment_done(m)
        elif now >= m.arrival_by:
            self._move_failed(m, NOT_AT_TARGET)

    def _segment_done(self, m) -> None:
        goal = m.goal
        self._clear_goal(m)
        m.move_order = None
        m.arrival_by = None
        if m.off_node is None or goal.mode != NAV_GOAL:
            m.attempts = 0
        m.not_sent_warned = False
        m.node, m.off_node = goal.end, None
        m.avoid.behind_since = None
        if m.route is not None:
            m.route.done = m.route.sent = goal.edges
        if goal.kind == RETREAT:
            m.avoid.retreat_to = ""
            self._event("INFO", "RETREAT_DONE", f"stands at {goal.end}", m.name)

    # ---- goal 잊기와 끝 지점에 서 있는지 판정 ----
    def _forget_goal(self, m) -> None:
        self._clear_goal(m)
        m.move_order = None
        m.arrival_by = None
        if m.route is not None:
            m.route.sent = m.route.done

    def _stands_at_end(self, m, behind: bool = False):
        """로봇이 서 있는 goal의 끝(끝 goal, 지나온 조각들). 여럿이면 가장 가까운 끝. 없으면 None."""
        goal = m.goal
        heard = (m.available and m.report.robot_state != LOCALIZING
                 and self._clock() - m.report.last_seen <= self.params.arrival_settle_s)
        if not (heard and goal is not None and goal.kind in (MOVE, RETREAT)):
            return None
        ends = [(self._gap(m, end.end), index, (end, passed))
                for index, (end, passed) in enumerate(self._goal_ends(m))
                if self._at_end(m, end) and not (behind and self._short_of_end(m, end))]
        return min(ends)[2] if ends else None

    # ---- 닿았다고 볼 수 있는 끝: goal의 끝, order면 끝내지 않은 조각 끝마다 ----
    def _goal_ends(self, m) -> list:
        """[(끝 goal, 그 끝까지 끝내지 않은 조각들)]."""
        goal, mo, route = m.goal, m.move_order, m.route
        if goal.mode != ORDER or mo is None or route is None:
            return [(goal, ())]
        ends, passed = [], []
        for seq, first, end in mo.cuts:
            if mo.done_seq < seq and end <= goal.edges:
                mode, node = route.edges[first].mode, route.edges[end - 1].b
                passed.append((mo.id, seq, mode, node))
                ends.append((Goal(MOVE, goal.id, end, node, mode), tuple(passed)))
        return ends or [(goal, ())]

    def _end_mode(self, m, end) -> str:
        return self._mode(m) if end.mode == ORDER else end.mode

    def _at_end(self, m, end) -> bool:
        limit = self.params.arrival_tolerance_m
        if self._end_mode(m, end) == NAV_GOAL:
            return self._gap(m, end.end) <= limit / 2
        if m.route is None:
            return self._gap(m, end.end) <= limit
        return self._edge_reached(m, m.route.edges[end.edges - 1], limit)

    # ---- 뒤가 막혀 선 로봇이 선을 따라 끝에 못 미쳤는지 ----
    def _short_of_end(self, m, end) -> bool:
        if m.route is None or self._end_mode(m, end) == NAV_GOAL:
            return False
        edge = m.route.edges[end.edges - 1]
        return edge.length - along(edge, m.x, m.y) > self.params.arrival_tolerance_m / 2

    def _arrived_anyway(self, m, reason: str, end, goal) -> None:
        target, passed = end
        self._event("WARN", "GOAL_ENDED_AT_TARGET",
                    f"{goal.kind} goal {goal.id} ended {reason}, but the robot stands at "
                    f"{target.end}", m.name)
        # 건너뛴 order 조각도 끝낸 것으로 정리한다
        for order_id, seq, mode, node in passed:
            m.node = node
            self._event("INFO", "SEGMENT_DONE", f"{order_id} seq {seq} ({mode}) to {node}", m.name)
            if mode == RETREAT:
                m.avoid.retreat_to = ""
                self._event("INFO", "RETREAT_DONE", f"stands at {node}", m.name)
        self._set_goal(m, target, reset_watch=False)
        self._segment_done(m)

    # ---- 이동 실패 사유별 처리 ----
    def _move_failed(self, m, reason: str) -> None:
        behind = reason == "OBSTACLE_BEHIND"
        end = None if reason in (HEARTBEAT_LOST, NOT_AT_TARGET) else self._stands_at_end(m, behind)
        if end is not None:
            self._arrived_anyway(m, reason, end, m.goal)
            return
        self._forget_goal(m)
        if behind and self._waits_behind(m):
            return
        if reason == HEARTBEAT_LOST:
            m.retry_at = self._clock() + self.params.move_retry_wait_s
            return
        if reason == NAV_UNAVAILABLE:
            # 로봇의 Nav2 서버가 아직 없어 보내지 못한 goal: 세지 않고 기다렸다 다시 보낸다
            m.retry_at = self._clock() + self.params.move_retry_wait_s
            self._event("WARN", "GOAL_NAV_UNAVAILABLE", "Nav2 is not up on the robot, not an "
                        "attempt, sends it again", m.name)
            return
        if reason == NAV_ABORTED and self._lost_pose_lately(m):
            # 위치를 잃어 끝낸 goal은 세지 않고 위치가 돌아온 뒤 다시 보낸다
            m.retry_at = self._clock() + self.params.move_retry_wait_s
            self._event("WARN", "GOAL_ABORTED_LOCALIZING", "not an attempt, sends it again "
                        "once it reports a pose", m.name)
            return
        if reason == PATH_NOT_CONNECTED:
            if self._in_yard(m):
                m.off_node = (m.x, m.y)
                self._drop_route(m)
        if reason == NAV_ABORTED:
            m.health.aborted_at = self._clock()
        self._goal_failed(m, reason)
