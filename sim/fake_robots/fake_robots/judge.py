"""ScenarioJudge: 시나리오 실행이 통과했는지 판정한다. ROS를 쓰지 않는다.

구역 메시지의 holder는 그 구역을 쥔 로봇들의 이름을 쉼표로 이은 것이다(넓은 구역과 한 줄
구역은 여러 대가 쥔다). 이중 예약은 그것만으로 드러나지 않으므로
물리적으로 검사한다: 로봇이 실제로 어디에 있는지, 보유자가 누구였는지. 가짜 로봇은 이를 위해
잡음 없는 위치를 내보낸다. 보고 한 번은 표준편차 몇 배씩 틀려 정상 실행을 떨어뜨릴 수 있고
통신 두절 중에는 끊긴다. 실제 로봇은 보고로 판정한다.

주문은 공통 설계 문서의 이벤트 코드로 판정한다: 중앙의 최종 보고와
플릿 매니저의 ORDER_BOUND, ORDER_REQUEUED.
"""
from __future__ import annotations

import re

from fleet_core.fleet_map import DOCK, SHARED

from fake_robots.laps import Laps
from fake_robots.order_checks import OrderChecks
from fake_robots.outcome import Outcome
from fake_robots.zone_checks import EPS, ZoneChecks

RATE_PERIOD_S = 0.5  # robots.csv는 2 Hz로 줄여 쓴다
OFF_LANE_GRACE_S = 1.0  # drift 시작부터 FMS가 연속 보고로 알아챌 때까지의 여유
PLACING = "PLACING"
FINAL_CODES = {"ORDER_SHIPPED": "SHIPPED", "ORDER_PARTIAL": "PARTIAL", "ORDER_HELD": "HELD"}
ORDER_ID = re.compile(r"[^\s:,;]+")


def order_id_of(detail: str) -> str:
    """주문 이벤트의 detail은 주문 ID로 시작한다."""
    match = ORDER_ID.match(detail.strip())
    return match.group(0) if match else ""


def is_pseudo_holder(holder: str) -> bool:
    """'#blocked' and the like are not robots: no pose, nothing to check."""
    return holder.startswith("#")


def holder_set(text: str) -> frozenset[str]:
    """/fleet/zones의 holder("pinky1,pinky3", 빈 값이면 아무도)에서 로봇 이름들. '#'는 뺀다."""
    return frozenset(name.strip() for name in (text or "").split(",")
                     if name.strip() and not is_pseudo_holder(name.strip()))



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
        self.final_counts = {"SHIPPED": 0, "PARTIAL": 0, "HELD": 0}
        self.bound_log: list[str] = []
        self.ship_log: list[str] = []
        self.order_log: list[tuple[str, str]] = []  # BOUND와 REQUEUED, 도착 순
        self.error_codes: list[str] = []
        self.board: list[str] = []
        self._bound_now: set[str] = set()
        # 팔
        fill_arms = [a for s in site.stations.values() if s.action == "fill" for a in s.arms]
        self._placing: dict[str, list[tuple[float, float]]] = {a: [] for a in fill_arms}
        self._placing_open: dict[str, float | None] = dict.fromkeys(fill_arms)
        self._placing_seen: set[str] = set()
        self._arm_stamp: dict[str, float] = {}
        # 바퀴
        self._work_zones = self._station_zones("fill")
        self._out_zones = self._station_zones("swap")
        self._parking = [site.node_pose(n) for n in site.parking]
        self._last_zone: dict[str, str] = {}
        self._from_outbound: set[str] = set()
        self.direct_returns = 0
        self.parked_while_backlog = 0
        self._tasks: dict[str, str] = {}  # 로봇 -> /fleet/assignments의 task_type
        # 자동 격리: 첫 ROBOT_ISOLATED부터 격리된 로봇의 첫 ZONES_CLEARED 전까지 출고한 수
        self.shipped_before_operator = 0
        self._isolated: set[str] = set()  # source가 로봇이다
        self._operator_came = False
        # 격리된 로봇(치우기 전)이 서 있는 동안 다른 로봇의 REROUTE 수(계약 4절: 코드와 로봇만 본다)
        self.detours_around_isolated = 0
        self._standing_isolated: set[str] = set()
        # 지시서 7.5: 관제가 처리한 이 이유의 실패. 실패 한 번은 GOAL_RETRY나 GOAL_FAILED 하나다.
        # 관제가 버린 결과(이미 바뀐 목표의 결과)는 이벤트가 없어 세지 않는다
        self.path_not_connected = 0
        self.robot_idle = 0  # 1.1 지시서 H2: 목적지가 있는데 goal도 구역 대기표도 없이 섰다

    def _station_zones(self, action: str) -> set[str]:
        return {self.site.nodes[s.dock].zone for s in self.site.stations.values()
                if s.action == action}

    # -- inputs -------------------------------------------------------------------------

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
        elif code == "ZONES_CLEARED" and source in self._isolated:
            self._operator_came = True
            self._standing_isolated.discard(source)
        elif code == "REROUTE" and self._standing_isolated - {source}:
            # 계약 4절: detail은 자유 글이라 코드와 로봇만 본다. 격리된 로봇이 아직 서 있을 때
            # 다른 로봇이 길을 바꾼 수
            self.detours_around_isolated += 1
        order = order_id_of(detail)
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
            mine = order in self._bound_by_reinstated
            self._bound_by_reinstated.discard(order)  # 최종 이벤트는 한 번만 센다
            self.final_counts[FINAL_CODES[code]] += 1
            if code != "ORDER_HELD":
                self.ship_log.append(order)
                if mine:
                    self.shipped_after_reinstate += 1
                if self._isolated and not self._operator_came:
                    self.shipped_before_operator += 1

    def note(self, t: float, source: str, code: str, detail: str) -> None:
        """events.csv에 남기되 플릿 이벤트로 세면 안 되는 것을 기록한다."""
        self.event_rows.append((t, "NOTE", source, code, detail))

    def put_back(self, t: float, robot: str, node) -> None:
        """관리자가 로봇을 이 지점에 다시 놓았다. 매니저가 잡을 grace_s 동안만 면제한다.

        시작 칸 면제는 더 쓰지 않는다. 못 잡으면 유예가 끝나는 시각에 바로 센다.
        """
        self._put_back[robot] = (node, t, t + self.grace_s)
        self._left_start.add(robot)

    def on_assignments(self, tasks: dict[str, str]) -> None:
        """로봇 -> task_type. 목록에 없는 로봇은 할 일이 없다."""
        self._tasks = dict(tasks)

    def on_board(self, order_ids: list[str]) -> None:
        self.board = list(order_ids)

    def on_arm_state(self, stamp: float, arm: str, state: str) -> None:
        """팔 자신의 스탬프를 쓴다. 두 토픽은 서로 순서가 뒤바뀌어 올 수 있다."""
        if arm not in self._placing or stamp < self._arm_stamp.get(arm, -1e18):
            return
        self._arm_stamp[arm] = stamp
        if state == PLACING:
            self._placing_seen.add(arm)
        since = self._placing_open[arm]
        if state == PLACING and since is None:
            self._placing_open[arm] = stamp
        elif state != PLACING and since is not None:
            self._placing[arm].append((since, stamp))
            self._placing_open[arm] = None

    # -- checks -------------------------------------------------------------------------

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
