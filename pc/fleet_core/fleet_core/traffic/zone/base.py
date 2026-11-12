"""구역 교통 제어기의 바탕: 가진 상태와, 여러 조각이 함께 쓰는 작은 동작."""
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
        self._started = None  # 첫 tick의 시각. 만들 때의 시계는 읽지 않는다(재생은 그때 0이다)
        self._startup_warned = False
        self._no_robot_warned = False
        self._joining: set[str] = set()  # 다시 투입되어 첫 보고를 기다리는 로봇
        self._wants: dict = {}  # 로봇 -> 잡으려는 구간(lane_policy.Want)
        # 막힘으로 표시한 연결 (a, b) -> 다시 써 볼 시각. 넓은 구역 한가운데에서 막혔으면
        # a는 HERE + 로봇 이름이다: 그 로봇이 선 자리에서 b로 곧장 가는 길만 막는다.
        self._blocked: dict = {}
        # 장애물 위치로 한 막힘 표시(block_marks.py): 번호 -> Mark. 지운 표시의 (x, y, 지우기 시간,
        # 지운 시각)은 같은 점에서 다시 막히면 시간을 두 배로 늘리려고 남긴다
        self._marks: dict = {}
        self._mark_memory: list = []
        self._mark_seq = itertools.count(1).__next__  # 표시와 CheckClear 질문의 번호
        self._cycle: tuple = ()  # 지금 보이는 대기 순환
        self._cycle_since = 0.0
        self._cycle_acted = False
        self._near: dict = {}  # (로봇, 로봇) -> (가까워진 시각, 그때의 두 자리)
        self.order_fault = ""  # 시험용: "stitch"면 다음 update의 이음 지점을 틀리게 보낸다
        self.clear_reason = ""
        self.station_left = None  # FleetManager가 station_left_s(robot)를 넣어 준다

    # ---- 작은 동작 ----

    def _event(self, level: str, code: str, detail: str, robot: str = "") -> None:
        self._out.append(EventOut(level, code, detail, robot))

    def _gap(self, m, node: str) -> float:
        return math.dist((m.x, m.y), self.site.node_pose(node))

    def _on_node(self, m) -> bool:
        """마지막으로 닿은 지점에 아직 서 있는가(보고 자세로 본다)."""
        if not m.node or self.grants.node_holder(m.node) not in ("", m.name):
            return False  # 남이 쥔 지점이다. 그 옆에 지점 없이 오른 로봇이 이름만 달고 있다
        # 고정 경로에 붙지 못한 로봇(off_node)은 지점 바로 위(허용 거리의 절반 안)로 보고해야
        # 지점 위다. 서 있는 로봇의 보고도 흔들린다: 가장자리의 보고를 다시 믿으면 같은 경로를
        # 또 보내고 로봇은 또 거절한다. 넓은 구역 밖에서는 좁히지 않는다: 거기서는 Nav2 goal로
        # 지점에 돌아갈 수 없어, 지점 위가 아니면 길을 못 잡는다
        limit = self.params.arrival_tolerance_m
        if m.off_node is not None and self._in_yard(m):
            limit /= 2
        return self._gap(m, m.node) <= limit

    def _in_yard(self, m) -> bool:
        """보고 위치가 넓은 구역 안인가."""
        zone = self.site.zone_at(m.x, m.y)
        return zone is not None and self.site.zones[zone].kind == SHARED

    def _via(self, m) -> str:
        """지금 달리는 방식. order goal이면 지금 조각의 방식이다."""
        goal, trip = m.goal, m.trip
        if goal is None:
            return ""
        if goal.via != ORDER or trip is None or trip.done >= len(trip.links):
            return goal.via
        return trip.links[trip.done].via

    def _priority(self, m) -> int:
        return priority(self.params, m.phase == AT_STATION, m.dispatch is not None)

    def _cancel_goal(self, m) -> None:
        if m.goal is None:
            return
        self._out.append(CancelCmd(m.name, m.goal.id))
        if m.move_order is not None and m.move_order.before is not None:
            # update의 답이 오기 전이다. 로봇에서는 이전 goal이 아직 달리고 있을 수 있다
            self._out.append(CancelCmd(m.name, m.move_order.before[0]))
        m.goal = None
        m.move_order = None
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
