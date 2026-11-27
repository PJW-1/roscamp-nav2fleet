"""ScenarioJudge: 시나리오 실행이 통과했는지 판정한다. ROS를 쓰지 않는다.

구역 보유자는 가짜 로봇의 잡음 없는 위치로 물리적으로 검사하고,
주문은 중앙의 최종 보고와 플릿 매니저의 ORDER_BOUND, ORDER_REQUEUED로 판정한다.
"""
from __future__ import annotations

import math
import re

from fleet_core.fleet_map import DOCK, SHARED

from fake_robots.laps import Laps
from fake_robots.order_checks import OrderChecks
from fake_robots.outcome import Outcome
from fake_robots.scenario_parts import _arms_with
from fake_robots.zone_checks import EPS, ZoneChecks

RATE_PERIOD_S = 0.5  # robots.csv는 2 Hz로 줄여 쓴다
OFF_LANE_GRACE_S = 1.0  # drift 시작부터 FMS가 연속 보고로 알아챌 때까지의 여유
# ORDER_HELD는 끝이 아니다(관리자의 강제 출고나 central의 취소를 기다린다)
FINAL_CODES = {"ORDER_SHIPPED": "SHIPPED", "ORDER_PARTIAL": "PARTIAL",
               "ORDER_CANCELED": "CANCELED"}
OVER_CAPACITY = "OVER_CAPACITY"  # 보고 없이 관제가 접수만 거절한 주문의 결과
ORDER_ID = re.compile(r"[^\s:,;]+")
HOLD_REASON = re.compile(r"hold_reason=(\w+)")
HELD_ROBOT = re.compile(r"robot=(\S+)")


def order_id_of(detail: str) -> str:
    """주문 이벤트의 detail은 주문 ID로 시작한다."""
    match = ORDER_ID.match(detail.strip())
    return match.group(0) if match else ""


def is_pseudo_holder(holder: str) -> bool:
    """'#blocked' 같은 것은 로봇이 아니다: 자세도 검사할 것도 없다."""
    return holder.startswith("#")


def holder_set(text: str) -> frozenset[str]:
    """/fleet/zones의 holder("pode1,pode3", 빈 값이면 아무도)에서 로봇 이름들. '#'는 뺀다."""
    return frozenset(name.strip() for name in (text or "").split(",")
                     if name.strip() and not is_pseudo_holder(name.strip()))



ERROR_EDGE_S = 0.5  # 배차 · 팔 상태 토픽 도착 순서 차이를 덮는 유예(시나리오 시각)

class ScenarioJudge(ZoneChecks, Laps, OrderChecks, Outcome):
    def __init__(self, site, scenario, margin_m: float = 0.13, grace_s: float = 2.0,
                 radius_m: float = 0.08):
        self.site = site
        self.scenario = scenario
        self.margin_m = margin_m
        self.radius_m = radius_m
        self.grace_s = grace_s
        self.poses: dict[str, tuple[float, float, str]] = {}
        self.truth: dict[str, tuple[float, float]] = {}
        self.holders: dict[str, frozenset[str]] = {}
        self.events_seen: set[str] = set()
        self.event_counts: dict[str, int] = {}
        self.violations: list[str] = []
        self.max_occupancy = 0
        self.release_violations = 0
        self.unheld_violations = 0
        self.off_lane_releases = 0
        # 로봇 -> 보고가 레인 밖으로 밀린 시간 구간들
        self._drifts: dict[str, list[tuple[float, float]]] = {}
        for f in scenario.faults:
            if f.type == "drift":
                self._drifts.setdefault(f.robot, []).append(
                    (f.at_s + OFF_LANE_GRACE_S, f.at_s + f.duration_s))
        self.robot_rows: list[tuple] = []
        self.zone_rows: list[tuple] = []
        self.event_rows: list[tuple] = []
        self.release_rows: list[tuple] = []
        self._last_logged: dict[str, float] = {}
        self._unheld_since: dict[tuple[str, str], float] = {}
        self._unheld_counted: set[tuple[str, str]] = set()
        self._over: set[tuple[str, ...]] = set()
        self._left_start: set[str] = set()
        self._put_back: dict[str, tuple] = {}  # 다시 투입한 로봇 -> (놓인 지점, 면제 끝 시각)
        self._reinstated: set[str] = set()
        self.bound_after_reinstate = 0
        self._bound_by_reinstated: set[str] = set()  # 재투입 뒤 그 로봇이 묶어 아직 쥔 주문
        self.shipped_after_reinstate = 0
        self._back_now: set[str] = set()  # 재투입 뒤 다시 치우지 않은 로봇
        # 주문
        self.registered: list[str] = list(scenario.order_ids)
        self.final: dict[str, str] = {}
        self.final_counts = {"SHIPPED": 0, "PARTIAL": 0, "CANCELED": 0}
        self.held: dict[str, str] = {}  # 지금 보류 중인 주문 -> 사유
        self.hold_reasons: dict[str, str] = {}  # 주문 -> 처음 보류된 사유(끝난 뒤에도 남김)
        self.order_report_problems: list[str] = []  # 끝난 주문에 또 온 최종, 보류 이벤트
        self.robots_isolated = 0
        self.bound_log: list[str] = []
        self.ship_log: list[str] = []
        self.order_log: list[tuple[str, str]] = []  # BOUND와 REQUEUED, 도착 순
        self.error_codes: list[str] = []
        self.board: list[str] = []
        # 작업대 팔 ERROR 동안 배차 막힘, [장애 처리 완료] 뒤 다시 열림 (끝에 dispatch_checks로 셈)
        self.fill_arms = set(_arms_with(site, "fill"))
        self._error_spans: list[list] = []  # [팔, 시작, 끝(None이면 아직 ERROR)]
        self._to_work: list[float] = []  # 새 TO_WORK 배차를 받은 시각
        self._resolved_at: float | None = None
        self._bound_now: set[str] = set()
        # 바퀴
        self._work_zones = self._station_zones("fill")
        self._out_zones = self._station_zones("swap")
        self._parking = [site.node_pose(n) for n in site.parking]
        self._last_zone: dict[str, str] = {}
        self._from_outbound: set[str] = set()
        self.direct_returns = 0
        self.parked_while_backlog = 0
        self._tasks: dict[str, str] = {}  # 로봇 -> /fleet/assignments의 task_type
        # 첫 ROBOT_ISOLATED나 ARM_FAULT 보류부터 그 로봇의 첫 ZONES_CLEARED 전까지 출고한 수
        self.shipped_before_operator = 0
        self._isolated: set[str] = set()  # source가 로봇이다
        self._arm_held: set[str] = set()  # ARM_FAULT 보류로 작업대에 묶여 사람을 기다리는 로봇
        self._operator_came = False
        # 격리된 로봇(치우기 전)이 서 있는 동안 다른 로봇의 REROUTE 수(계약 4절: 코드와 로봇만 본다)
        self.detours_around_isolated = 0
        self._standing_isolated: set[str] = set()
        # 관제가 처리한 이 이유의 실패. 실패 한 번은 GOAL_RETRY나 GOAL_FAILED 하나다.
        # 관제가 버린 결과(이미 바뀐 목표의 결과)는 이벤트가 없어 세지 않는다
        self.path_not_connected = 0
        self.robot_idle = 0  # 목적지가 있는데 goal도 구역 대기표도 없이 섰다

    def _station_zones(self, action: str) -> set[str]:
        return {self.site.nodes[s.dock].zone for s in self.site.stations.values()
                if s.action == action}

    # ---- 입력 ----

    def on_robot_state(self, t: float, robot: str, x: float, y: float, state: str) -> None:
        self.poses[robot] = (x, y, state)
        if t - self._last_logged.get(robot, -1e9) >= RATE_PERIOD_S - EPS:
            self._last_logged[robot] = t
            self.robot_rows.append((t, robot, x, y, state))

    def on_true_pose(self, robot: str, x: float, y: float) -> None:
        self.truth[robot] = (x, y)

    def on_zones(self, t: float, holders: dict[str, str]) -> None:
        """holders: 구역 -> /fleet/zones의 holder 글(쉼표로 이은 이름들)."""
        for zone, text in holders.items():
            new = holder_set(text)
            old = self.holders.get(zone, frozenset())
            if new == old:
                continue
            self.holders[zone] = new
            self.zone_rows.append((t, zone, ",".join(sorted(old)), ",".join(sorted(new))))
            for robot in sorted(old - new):
                self._check_release(t, zone, robot)

    def on_event(self, t: float, level: str, source: str, code: str, detail: str) -> None:
        self.events_seen.add(code)
        self.event_counts[code] = self.event_counts.get(code, 0) + 1
        self.event_rows.append((t, level, source, code, detail))
        if level == "ERROR":
            self.error_codes.append(code)
        if code in ("GOAL_RETRY", "GOAL_FAILED") and "PATH_NOT_CONNECTED" in detail:
            self.path_not_connected += 1
        if code == "ROBOT_IDLE":
            self.robot_idle += 1
        if code == "ROBOT_ISOLATED":
            self._isolated.add(source)
            self._standing_isolated.add(source)
        elif code == "ZONES_CLEARED" and source in self._isolated | self._arm_held:
            self._operator_came = True
            self._standing_isolated.discard(source)
        elif code == "REROUTE" and self._standing_isolated - {source}:
            # 계약 4절: detail은 자유 글이라 코드와 로봇만 본다. 격리된 로봇이 아직 서 있을 때
            # 다른 로봇이 길을 바꾼 수
            self.detours_around_isolated += 1
        order = order_id_of(detail)
        if code == "ROBOT_ISOLATED":
            self.robots_isolated += 1
        if code in ("ORDER_HELD", "ORDER_OVER_CAPACITY", *FINAL_CODES) and order in self.final:
            # 최종은 한 번이다. 끝난 주문(용량 초과 포함)의 보고는 덮어쓰지 않고 위반으로 센다
            self._bad_report(t, f"{order} {code} after {self.final[order]}")
            return
        if code == "ORDER_HELD":
            match = HOLD_REASON.search(detail)
            reason = match.group(1) if match else ""
            self.held[order] = reason
            self.hold_reasons.setdefault(order, reason)
            robot = HELD_ROBOT.search(detail)
            if reason == "ARM_FAULT" and robot:
                self._arm_held.add(robot.group(1))
        elif code == "ORDER_OVER_CAPACITY":
            self.final[order] = OVER_CAPACITY
        if code == "ORDER_ACCEPTED" and order not in self.registered:
            self.registered.append(order)  # 시나리오 주문은 처음부터 등록돼 있다
        elif code == "ROBOT_REINSTATED":
            self._reinstated.add(source)
            self._back_now.add(source)
        elif code == "ZONES_CLEARED":
            self._back_now.discard(source)
        elif code == "ORDER_BOUND":
            if source in self._reinstated:
                self.bound_after_reinstate += 1
            if source in self._back_now:
                self._bound_by_reinstated.add(order)
            else:
                self._bound_by_reinstated.discard(order)
            self.bound_log.append(order)
            self.order_log.append(("BOUND", order))
            self._bound_now.add(order)
        elif code == "ORDER_REQUEUED":
            self.order_log.append(("REQUEUED", order))
            self._bound_now.discard(order)
            self._bound_by_reinstated.discard(order)
        elif code in FINAL_CODES:
            self.final[order] = FINAL_CODES[code]
            self.held.pop(order, None)
            mine = order in self._bound_by_reinstated
            self._bound_by_reinstated.discard(order)  # 최종 이벤트는 한 번만 센다
            self.final_counts[FINAL_CODES[code]] += 1
            if code != "ORDER_CANCELED":
                self.ship_log.append(order)
                if mine:
                    self.shipped_after_reinstate += 1
                if (self._isolated or self._arm_held) and not self._operator_came:
                    self.shipped_before_operator += 1

    def _bad_report(self, t: float, text: str) -> None:
        self.order_report_problems.append(text)
        self._violate(t, text)

    def note(self, t: float, source: str, code: str, detail: str) -> None:
        """events.csv에 남기되 플릿 이벤트로 세면 안 되는 것을 기록한다."""
        self.event_rows.append((t, "NOTE", source, code, detail))

    def put_back(self, t: float, robot: str, node) -> None:
        """관리자가 로봇을 이 지점에 다시 놓았다. 매니저가 잡을 grace_s 동안만 면제한다.

        시작 칸 면제는 더 쓰지 않는다. 못 잡으면 유예가 끝나는 시각에 바로 센다.
        """
        self._put_back[robot] = (node, t, t + self.grace_s)
        self._left_start.add(robot)

    def on_assignments(self, tasks: dict[str, str], t: float | None = None) -> None:
        """로봇 -> task_type. 목록에 없는 로봇은 할 일이 없다. 새 TO_WORK 배차 시각을 남긴다."""
        if t is not None:
            self._to_work += [t for robot, task in tasks.items()
                              if task == "TO_WORK" and self._tasks.get(robot) != "TO_WORK"]
        self._tasks = dict(tasks)

    def on_arm_state(self, arm: str, state: str, t: float) -> None:
        """팔 상태 토픽. 작업대 팔의 ERROR 구간을 남긴다."""
        if arm not in self.fill_arms:
            return
        open_span = next((s for s in self._error_spans if s[0] == arm and s[2] is None), None)
        if state == "ERROR" and open_span is None:
            self._error_spans.append([arm, t, None])
        elif state != "ERROR" and open_span is not None:
            open_span[2] = t

    def dispatch_checks(self) -> tuple[int, int]:
        """(ERROR 동안 새 배차 수, [장애 처리 완료] 뒤 팔이 ERROR를 벗어나고 나서 배차 수).

        배차와 팔 상태는 다른 토픽이라 도착 순서가 바뀔 수 있어 구간 양끝에 유예를 둔다.
        """
        g = ERROR_EDGE_S
        spans = [(s, math.inf if e is None else e) for _, s, e in self._error_spans]
        during = [t for t in self._to_work if any(s + g <= t <= e - g for s, e in spans)]
        cleared = [e for s, e in spans
                   if self._resolved_at is not None and self._resolved_at <= e < math.inf]
        if not cleared:
            return len(during), 0
        first = min(cleared)
        return len(during), sum(1 for t in self._to_work if t >= first - g and t not in during)

    def on_fault_resolved(self, t: float) -> None:
        if self._resolved_at is None:
            self._resolved_at = t

    def on_board(self, order_ids: list[str]) -> None:
        self.board = list(order_ids)

    # ---- 검사 ----

    def tick(self, t: float) -> None:
        by_zone: dict[str, list[str]] = {}
        for robot, (x, y) in self._positions().items():
            zone = self.site.zone_at(x, y)
            if zone is not None:
                by_zone.setdefault(zone, []).append(robot)
            self._track_lap(t, robot, self._place(x, y, zone))
        for zone, robots in by_zone.items():
            spec = self.site.zones[zone]
            # zone_holder_max는 한 대 구역(도크, 갈림목)에서 잰다. 여러 대가 들어가는 구역에
            # 있는 로봇은 1로 센다(어딘가에 있었다는 것만).
            seen = len(robots) if spec.kind == DOCK else 1
            self.max_occupancy = max(self.max_occupancy, seen)
            capacity = 1 if spec.kind == DOCK else spec.capacity
            key = (zone, *sorted(robots))
            if spec.kind != SHARED and len(robots) > capacity and key not in self._over:
                self._over.add(key)
                self._violate(t, f"{', '.join(sorted(robots))} are in {zone} at the same time")
        self._check_unheld(t)
