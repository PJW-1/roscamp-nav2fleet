# ---- ZoneTraffic 공통 상태와 기본 도구 ----
from __future__ import annotations

import itertools
import math

from fleet_core.common.commands import DispatchResult, EventOut, RobotIsolated
from fleet_core.fleet_map import SHARED
from fleet_core.traffic.deadlock import priority
from fleet_core.traffic.mission import AT_STATION, ERROR, FILL, LOCALIZING, NOT_AT_TARGET, RUN
from fleet_core.traffic.selector import RobotSelector

from .block_marks import MarkBook
from .capacity import CapacityCheck
from .deadlock_watch import DeadlockWatch
from .detours import DetourNotes
from .grants import ZoneGrants
from .lane_policy import LanePolicy
from .localization import PoseJumpWatch
from .nostart import StartWatch
from .robots import ORDER, Robot
from .routes import along
from .silent import SilentWatch
from .views import ZoneViews

HERE = "@"
# LOCALIZING 보고 앞뒤 이 시간 안의 NAV_ABORTED는 위치를 잃어 끝난 것으로 본다
LOCALIZING_GRACE_S = 1.0


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
        self._near: dict = {}
        self.order_fault = ""
        self.clear_reason = ""
        self._views = ZoneViews(site, self.grants, self._robots, self._joining, self._wants,
                                self._blocked, self._marks)
        self._capacity = CapacityCheck(site, self._event)
        self._pose_watch = PoseJumpWatch(params, self._event)
        self._start_watch = StartWatch(params, site, self._event)
        self._silent = SilentWatch(params, self._robots, self._event, self._event_once)
        self._detours = DetourNotes(site, self._event)
        self._mark_book = MarkBook(params, self._blocked, self._marks, self._robots, clock)
        self._deadlock = DeadlockWatch(params, self.grants, self._wants, self._robots,
                                       self._event)

    # ---- 로봇 목록, 운행 상태, 가용 여부, 쌓인 출력 꺼내기 ----
    @property
    def robots(self) -> list[str]:
        return list(self._robots)

    @property
    def running(self) -> bool:
        return self._run_mode == RUN

    @property
    def zones(self):
        return self.grants

    def available(self, robot) -> bool:
        m = self._robots.get(robot)
        return m is not None and m.available

    def take_output(self) -> list:
        out, self._out = self._out, []
        return out

    # ---- 이벤트 내기와 위치 판정 ----
    def _event(self, level: str, code: str, detail: str, robot: str = "") -> None:
        self._out.append(EventOut(level, code, detail, robot))
        self._silent.note(robot, code)

    def _event_once(self, ep, level: str, code: str, detail: str, robot: str = "") -> None:
        if ep.noted:
            return
        ep.noted = True
        self._event(level, code, detail, robot)

    def _gap(self, m, node: str) -> float:
        return math.dist((m.x, m.y), self.site.node_pose(node))

    def _edge_reached(self, m, edge, limit: float) -> bool:
        gap = self._gap(m, edge.b)
        return gap <= limit and (2 * along(edge, m.x, m.y) > edge.length or gap <= limit / 2)

    def _on_node(self, m) -> bool:
        if not m.node or self.grants.node_holder(m.node) not in ("", m.name):
            return False
        limit = self.params.arrival_tolerance_m
        if m.off_node is not None and self._in_yard(m):
            limit /= 2
        return self._gap(m, m.node) <= limit

    # ---- 위치를 잃은 로봇(LOCALIZING): 새 goal 없이 기다린다 ----
    @staticmethod
    def _localizing(m) -> bool:
        return m.report.robot_state == LOCALIZING

    def _lost_pose_lately(self, m) -> bool:
        at = m.health.localizing_at
        return self._localizing(m) or (at is not None
                                       and self._clock() - at <= LOCALIZING_GRACE_S)

    def _wait_localizing(self, m) -> None:
        if m.health.localizing_since is not None:
            return
        m.health.localizing_since = self._clock()
        self._event("WARN", "WAIT_LOCALIZING", f"{m.phase}: no new goal until it reports a "
                    f"pose, keeps {list(self.grants.held_by(m.name))}", m.name)

    # ---- LOCALIZING 보고 바로 앞에 센 NAV_ABORTED는 되돌린다 ----
    def _note_localizing(self, m, now: float) -> None:
        m.health.localizing_at = now
        at, m.health.aborted_at = m.health.aborted_at, None
        if (at is not None and now - at <= LOCALIZING_GRACE_S and m.attempts > 0
                and not m.health.fault):
            m.attempts -= 1
            self._event("WARN", "GOAL_ABORTED_LOCALIZING", "the last NAV_ABORTED came just "
                        f"before LOCALIZING, not an attempt (now {m.attempts})", m.name)

    def _localized(self, m) -> None:
        since, m.health.localizing_since = m.health.localizing_since, None
        if since is not None:
            self._event("INFO", "LOCALIZED", f"reports a pose again after "
                        f"{self._clock() - since:.1f} s", m.name)

    def _in_yard(self, m) -> bool:
        zone = self.site.zone_at(m.x, m.y)
        return zone is not None and self.site.zones[zone].kind == SHARED

    # ---- 현재 주행 방식과 우선순위 ----
    def _mode(self, m) -> str:
        goal, route = m.goal, m.route
        if goal is None:
            return ""
        if goal.mode != ORDER or route is None or route.done >= len(route.edges):
            return goal.mode
        return route.edges[route.done].mode

    def _priority(self, m) -> int:
        return priority(self.params, m.phase == AT_STATION, m.dispatch is not None)

    def _goal_priority(self, m) -> int:
        m.sent_priority = self._priority(m)
        return m.sent_priority

    # ---- route와 대기 정리 ----
    def _drop_route(self, m) -> None:
        m.route = None

    def _stop_wanting(self, m) -> None:
        self._wants.pop(m.name, None)
        self.grants.stop_waiting(m.name)
        m.watch.wait.clear()

    # ---- 로봇 격리와 goal 실패 재시도 ----
    def _fail(self, m, reason: str) -> None:
        first = m.health.fault != ERROR
        if first:
            m.health.fault_xy = (m.x, m.y)
        m.health.pose_doubt = m.health.pose_doubt or reason == NOT_AT_TARGET
        m.health.fault = ERROR
        m.health.fault_reason = reason
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
