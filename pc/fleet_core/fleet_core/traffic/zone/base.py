# ---- ZoneTraffic 공통 상태와 기본 도구 ----
from __future__ import annotations

import itertools
import math

from fleet_core.common.commands import CancelCmd, DispatchResult, EventOut, RobotIsolated
from fleet_core.fleet_map import SHARED
from fleet_core.traffic.deadlock import priority
from fleet_core.traffic.mission import AT_STATION, ERROR, FILL, NOT_AT_TARGET, RUN, SWAP
from fleet_core.traffic.selector import RobotSelector

from .grants import ZoneGrants
from .lane_policy import LanePolicy
from .robots import ORDER, Robot
from .trips import along

HERE = "@"


# ---- 로봇, 허락, 호출, 막힘 표시 상태 초기화 ----
class ZoneBase:
    def __init__(self, site, params, robots, clock, next_goal_id=None):
        self.site = site
        self.params = params
        self._clock = clock
        self._next_goal_id = next_goal_id or itertools.count(1).__next__
        self.grants = ZoneGrants(site, clock, params.lane_follow_gap_s)
        self.lanes = LanePolicy(params)
        self.selector = RobotSelector(site, params)
        self._robots = {name: Robot(name) for name in robots}
        self._work = next(s.id for s in site.stations.values() if s.action == FILL)
        self._outbound = next(s.id for s in site.stations.values() if s.action == SWAP)
        self._open_calls: list[int] = []
        self._out: list = []
        self._run_mode = RUN
        self._started = None
        self._startup_warned = False
        self._no_robot_warned = False
        self._joining: set[str] = set()
        self._wants: dict = {}
        self._blocked: dict = {}
        self._marks: dict = {}
        self._mark_memory: list = []
        self._mark_seq = itertools.count(1).__next__
        self._cycle: tuple = ()
        self._cycle_since = 0.0
        self._cycle_acted = False
        self._near: dict = {}
        self.order_fault = ""
        self.clear_reason = ""
        self.station_left = None

    # ---- 이벤트 내기와 위치 판정 ----
    def _event(self, level: str, code: str, detail: str, robot: str = "") -> None:
        self._out.append(EventOut(level, code, detail, robot))

    def _gap(self, m, node: str) -> float:
        return math.dist((m.x, m.y), self.site.node_pose(node))

    def _link_reached(self, m, link, limit: float) -> bool:
        gap = self._gap(m, link.b)
        return gap <= limit and (2 * along(link, m.x, m.y) > link.length or gap <= limit / 2)

    def _on_node(self, m) -> bool:
        if not m.node or self.grants.node_holder(m.node) not in ("", m.name):
            return False
        limit = self.params.arrival_tolerance_m
        if m.off_node is not None and self._in_yard(m):
            limit /= 2
        return self._gap(m, m.node) <= limit

    def _in_yard(self, m) -> bool:
        zone = self.site.zone_at(m.x, m.y)
        return zone is not None and self.site.zones[zone].kind == SHARED

    # ---- 현재 주행 방식과 우선순위 ----
    def _via(self, m) -> str:
        goal, trip = m.goal, m.trip
        if goal is None:
            return ""
        if goal.via != ORDER or trip is None or trip.done >= len(trip.links):
            return goal.via
        return trip.links[trip.done].via

    def _priority(self, m) -> int:
        return priority(self.params, m.phase == AT_STATION, m.dispatch is not None)

    def _goal_priority(self, m) -> int:
        m.sent_priority = self._priority(m)
        return m.sent_priority

    # ---- goal 취소와 트립, 대기 정리 ----
    def _cancel_goal(self, m) -> None:
        if m.goal is None:
            return
        self._out.append(CancelCmd(m.name, m.goal.id))
        if m.move_order is not None and m.move_order.before is not None:
            self._out.append(CancelCmd(m.name, m.move_order.before[0]))
        m.goal = None
        m.move_order = None
        m.arrival_by = None
        m.settle_until = self._clock() + self.params.arrival_settle_s
        if m.trip is not None:
            m.trip.sent = m.trip.done

    def _drop_trip(self, m) -> None:
        m.trip = None

    def _stop_wanting(self, m) -> None:
        self._wants.pop(m.name, None)
        self.grants.stop_waiting(m.name)
        m.wait_since = None
        m.wait_for = ()
        m.wait_warned = m.queued = False

    # ---- 로봇 격리와 goal 실패 재시도 ----
    def _fail(self, m, reason: str) -> None:
        first = m.fault != ERROR
        if first:
            m.fault_xy = (m.x, m.y)
        m.pose_doubt = m.pose_doubt or reason == NOT_AT_TARGET
        m.fault = ERROR
        m.fault_reason = reason
        m.called = False
        self._stop_wanting(m)
        if m.dispatch:
            self._out.append(DispatchResult(m.dispatch.goal_id, False, m.name, reason))
            m.dispatch = None
        if first:
            if m.goal:
                self._cancel_goal(m)
            self._out.append(RobotIsolated(m.name, reason))
            held = list(self.grants.held_by(m.name))
            self._event("ERROR", "ROBOT_ISOLATED", f"{reason}; holds {held}", m.name)

    def _goal_failed(self, m, reason: str) -> None:
        m.attempts += 1
        m.retry_at = self._clock() + self.params.move_retry_wait_s
        if m.attempts > self.params.move_retry_limit:
            self._event("ERROR", "GOAL_FAILED", f"{reason} after {m.attempts} attempts", m.name)
            self._fail(m, reason)
        else:
            self._event("WARN", "GOAL_RETRY", f"{reason}, attempt {m.attempts}", m.name)
