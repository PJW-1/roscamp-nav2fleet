"""구역 교통 제어기의 바탕: 가진 상태와, 여러 조각이 함께 쓰는 작은 동작."""
from __future__ import annotations

import itertools
import math

from fleet_core.common.commands import CancelCmd, DispatchResult, EventOut, RobotIsolated
from fleet_core.traffic.deadlock import priority
from fleet_core.traffic.mission import AT_STATION, ERROR, FILL, NOT_AT_TARGET, RUN, SWAP
from fleet_core.traffic.selector import RobotSelector

from .grants import ZoneGrants
from .lane_policy import LanePolicy
from .robots import Robot

HERE = "@"  # 막힘 표시에서 "이 로봇이 지금 선 자리"를 가리키는 머리말


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
        self._started = clock()
        self._startup_warned = False
        self._no_robot_warned = False
        self._joining: set[str] = set()  # 다시 투입되어 첫 보고를 기다리는 로봇
        self._wants: dict = {}  # 로봇 -> 잡으려는 구간(lane_policy.Want)
        # 막힘으로 표시한 연결 (a, b) -> 다시 써 볼 시각. 넓은 구역 한가운데에서 막혔으면
        # a는 HERE + 로봇 이름이다: 그 로봇이 선 자리에서 b로 곧장 가는 길만 막는다.
        self._blocked: dict = {}
        self._cycle: tuple = ()  # 지금 보이는 대기 순환
        self._cycle_since = 0.0
        self._cycle_acted = False
        self._near: dict = {}  # (로봇, 로봇) -> (가까워진 시각, 그때의 두 자리)
        self.clear_reason = ""
        self.station_left = None  # FleetManager가 station_left_s(robot)를 넣어 준다

    # ---- 작은 동작 ----

    def _event(self, level: str, code: str, detail: str, robot: str = "") -> None:
        self._out.append(EventOut(level, code, detail, robot))

    def _gap(self, m, node: str) -> float:
        return math.dist((m.x, m.y), self.site.node_pose(node))

    def _on_node(self, m) -> bool:
        """마지막으로 닿은 지점에 아직 서 있는가(보고 자세로 본다)."""
        return bool(m.node) and self._gap(m, m.node) <= self.params.arrival_tolerance_m

    def _via(self, m) -> str:
        """지금 달리는 방식(goal이 없으면 빈 문자열)."""
        return m.goal.via if m.goal is not None else ""

    def _priority(self, m) -> int:
        return priority(self.params, m.phase == AT_STATION, m.dispatch is not None)

    def _cancel_goal(self, m) -> None:
        if m.goal is None:
            return
        self._out.append(CancelCmd(m.name, m.goal.id))
        m.goal = None
        m.arrival_by = None
        m.settle_until = self._clock() + self.params.arrival_settle_s  # 취소가 닿을 때까지
        if m.trip is not None:
            m.trip.sent = m.trip.done  # 남은 조각은 다시 보낸다

    def _drop_trip(self, m) -> None:
        """가던 길을 버린다. 잡아 둔 구역과 지점은 여기서 놓지 않는다: 보고 위치가 벗어난 것을
        확인한 뒤에 놓는다(progress._release_behind). 취소가 로봇에 닿기 전일 수 있어서다."""
        m.trip = None

    def _stop_wanting(self, m) -> None:
        self._wants.pop(m.name, None)
        self.grants.stop_waiting(m.name)
        m.wait_since = None
        m.wait_for = ()
        m.wait_warned = m.queued = False

    def _fail(self, m, reason: str) -> None:
        """로봇을 ERROR로 세운다. 구역은 쥔 채 두고 운영자가 푼다(confirm_clear)."""
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
        """실패한 시도 하나. 한도를 넘으면 ERROR, 아니면 잠시 뒤 다시 낸다."""
        m.attempts += 1
        m.retry_at = self._clock() + self.params.move_retry_wait_s
        if m.attempts > self.params.move_retry_limit:
            self._event("ERROR", "GOAL_FAILED", f"{reason} after {m.attempts} attempts", m.name)
            self._fail(m, reason)
        else:
            self._event("WARN", "GOAL_RETRY", f"{reason}, attempt {m.attempts}", m.name)
