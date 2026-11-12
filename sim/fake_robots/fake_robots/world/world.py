"""머리 없는 구역 세계: 가짜 시계로 FleetManager(traffic_mode zones)를 돌린다. ROS를 쓰지 않는다.

SimWorld(fleet_core/test/sim_world.py)가 옛 관제를 돌리던 방식을 따른다. 한 걸음(DT = 0.1 s):
시각 진행, 미리 넣은 일, 지난 걸음의 결과, 로봇 움직임, 팔, (결과 일부) 위치 보고 (결과 나머지),
주문판, manager.tick(), 나온 명령 옮기기, 불변식과 숫자.

관제는 manager_factory(site, params, robots, clock)로 만든다. 기본은 FleetManager다. 시험은
recording.RecordingManager를 넣어 명령 실행만 볼 수 있다.
"""
from __future__ import annotations

import math
import random
from dataclasses import replace

from fleet_core.common.params import load_params
from fleet_core.fleet_map import PARKING, WAIT, load_fleet_map

from ..battery import Battery
from .apply import Commands
from .avoid import PERFECT, MixedSettings, check_mode
from .free_space import FreeSpace
from .geom import segment_gap
from .injections import LowBattery, OrderFault, Reinstate, Removal, Schedule
from .invariants import Invariants
from .link import Link
from .metrics import Metrics
from .order_metrics import OrderMetrics
from .order_run import OrderRun, stops_for_lost_contact
from .reaction import ReactionWatch
from .robot import Imperfections, SimRobot, profile_for
from .runs import DOCK, HEARTBEAT_LOST, Result
from .stations import Arms, Central, six_orders

DT = 0.1
BLOCKED_AFTER_S = 5.0  # nav_goal: 이만큼 못 나아가면 BLOCKED(지시서 4절)
STUCK_TIMEOUT_S = 10.0  # 멈춘 로봇의 이동은 이만큼 뒤 TIMEOUT
DEFAULT_STOCK = {"A": 20, "B": 20, "C": 20, "D": 20}


def default_manager(site, params, robots, clock):
    from fleet_core.manager import FleetManager
    return FleetManager(site, params, robots, clock)


def default_robots(site, count=3) -> dict[str, str]:
    """pinky1..은 주차 자리 순서대로."""
    return {f"pinky{i + 1}": node for i, node in enumerate(site.parking[:count])}


class ZoneWorld(Commands, Link):
    def __init__(self, layout="d2", seed=1, params=None, overrides=None, robots=None,
                 orders=None, injections=(), imperfect=None, manager_factory=default_manager,
                 stock=None, arm_s_per_item=2.0, swap_step_s=2.0, dock_s=2.0, run_mode="RUN",
                 avoid_mode=PERFECT, avoid_settings=None):
        self.site = load_fleet_map(layout) if isinstance(layout, str) else layout
        params = params or load_params()
        self.params = replace(params, traffic_mode="zones", **(overrides or {}))
        self.seed = seed
        self.rng = random.Random(seed)
        self.imperfect = Imperfections() if imperfect is None else imperfect
        self.t = 0.0
        self.dt = DT
        self.radius = self.params.robot_radius_m
        self.blocked_after_s = BLOCKED_AFTER_S
        self.avoid_mode = check_mode(avoid_mode)  # 회피 흉내(avoid.py)
        self.avoid_settings = avoid_settings or MixedSettings()
        self.dock_s = dock_s
        self.run_mode = run_mode
        self.schedule = Schedule(injections)
        starts = robots or default_robots(self.site)
        self.robots: dict[str, SimRobot] = {}
        for name, node in starts.items():
            x, y = self.site.node_pose(node)
            yaw = self.site.nodes[node].yaw or 0.0
            self.robots[name] = SimRobot(name, x, y, yaw,
                                         profile=profile_for(self.rng, self.imperfect))
        self.space = FreeSpace(self.site, self.radius)
        self.events: list = []  # (시각, EventOut)
        self.commands: list = []  # (시각, 명령)
        self.results: list = []  # (시각, 로봇, Result)
        self.reports_sent = 0
        self.guard_stops = 0  # 몸이 닿기 직전에 세계가 세운 수(충돌 검사가 놓친 것)
        self.invariants = Invariants(self.site, self.params)
        self.reaction = ReactionWatch(self.params, self.invariants.violations)
        self.metrics = Metrics(self.site, self.robots)
        self.order_metrics = OrderMetrics(self.site, self.robots)
        self.central = Central(six_orders() if orders is None else orders, self._violation)
        self.arms = Arms(self.site, stock or DEFAULT_STOCK, arm_s_per_item, swap_step_s,
                         self._violation)
        spots = [self.site.node_pose(n) for n in self.site.parking]
        low = self.schedule.of(LowBattery)
        self.batteries = {name: Battery([f for f in low if f.robot == name], spots)
                          for name in sorted({f.robot for f in low})}
        self._setup_link()
        self.manager = manager_factory(self.site, self.params, list(self.robots), lambda: self.t)

    # ---- 시험이 읽는 것 ----

    @property
    def traffic(self):
        return getattr(self.manager, "traffic", None)

    def grants(self):
        return getattr(self.traffic, "grants", None)

    def views(self) -> dict:
        view = getattr(self.traffic, "robot_view", None)
        return {name: (view(name) if view else {}) for name in self.robots}

    @property
    def violations(self):
        return self.invariants.violations

    def event_codes(self, robot=None) -> list[str]:
        return [e.code for _, e in self.events if robot is None or e.robot == robot]

    def commands_of(self, kind, robot=None) -> list:
        return [c for _, c in self.commands
                if isinstance(c, kind) and (robot is None or getattr(c, "robot", "") == robot)]

    def orders_done(self) -> bool:
        return self.central.all_final()

    def present(self) -> dict[str, SimRobot]:
        return {n: r for n, r in self.robots.items() if not r.removed}

    def discs(self, robot) -> list[tuple[float, float, float]]:
        """robot이 피해야 할 원: 바닥 위 다른 로봇과 지금 있는 물체."""
        others = [(r.x, r.y, self.radius) for r in self.robots.values()
                  if r is not robot and not r.removed]
        return others + self.schedule.objects_now(self.t)

    # ---- 걸음 ----

    def step(self) -> None:
        self.t = round(self.t + DT, 6)
        self._fire()
        self._deliver_queued()
        self._move()
        self.arms.run(self.t, self.robots, self._slow, self.manager)
        self._flush(early=True)
        self._report()
        self._flush(early=False)
        self.manager.on_order_board(self.run_mode, self.central.board(self.t))
        for item in self.manager.tick():
            self.send(item)
        present, grants = self.present(), self.grants()
        # 5번에서 뺀다: BLOCKED, 끊김, 그리고 고장 주입으로 멈춘 로봇(교통 층이 세운 것이 아니다)
        quiet = {n: (r.run is not None and r.run.blocked) or not r.comm_up or r.stuck
                 for n, r in present.items()}
        views = self.views()
        self.invariants.check(self.t, grants, present, views, quiet)
        self.reaction.check(self.t, present)
        self.metrics.step(self.t, DT, grants, present, self.orders_done(), views)
        self.order_metrics.step(self.t, DT, present, views)

    def run(self, seconds: float) -> None:
        for _ in range(round(seconds / DT)):
            self.step()

    def run_until(self, predicate, timeout_s: float = 1500.0) -> bool:
        for _ in range(round(timeout_s / DT)):
            self.step()
            if predicate():
                return True
        return False

    def _fire(self) -> None:
        for name, r in self.robots.items():
            r.comm_up = not self.schedule.comm_down(name, self.t)
            if r.comm_up:
                r.comm_down_since = None
            elif r.comm_down_since is None:
                r.comm_down_since = self.t
            r.stuck = self.schedule.stuck(name, self.t)
        for item in self.schedule.due(self.t):
            if isinstance(item, OrderFault):
                self.traffic.order_fault = item.kind
                continue
            r = self.robots[item.robot]
            if isinstance(item, Removal):
                r.removed, r.run = True, None
                if item.clear:
                    grants = self.grants()
                    held = list(grants.held_by(item.robot)) if grants is not None else []
                    self.manager.confirm_clear(item.robot, held, "removed from the floor")
            elif isinstance(item, Reinstate):
                node = self._free_spot(item.node, item.robot)
                r.x, r.y = self.site.node_pose(node)
                r.yaw = self.site.nodes[node].yaw or 0.0
                r.removed, r.run, r.docked_at = False, None, ""
                r.has_basket, r.basket = item.basket == "EMPTY", {}
                self.manager.reinstate(item.robot, item.basket, "put back")

    def _free_spot(self, node: str, robot: str) -> str:
        """node에 다른 로봇이 있으면 가장 가까운 빈 대기 지점(운영자는 겹쳐 놓지 않는다).
        도크와 관제가 다른 로봇에 잡아 준 지점에는 놓지 않는다."""
        others = [o.pose for n, o in self.present().items() if n != robot]
        gap = 2 * self.params.robot_radius_m
        grants = self.grants()

        def free(n):
            holder = grants.node_holder(n) if grants is not None else ""
            return (holder in ("", robot)
                    and all(math.dist(self.site.node_pose(n), p) >= gap for p in others))
        if free(node):
            return node
        spots = [n for n in self.site.nodes
                 if self.site.nodes[n].kind in (WAIT, PARKING) and free(n)]
        here = self.site.node_pose(node)
        return min(spots, key=lambda n: math.dist(here, self.site.node_pose(n)), default=node)

    def _move(self) -> None:
        for name, r in self.robots.items():
            run = r.run
            if r.removed or run is None:
                continue
            if r.stuck and run.kind != DOCK:
                run.stuck_s = getattr(run, "stuck_s", 0.0) + DT
                if run.stuck_s >= STUCK_TIMEOUT_S - 1e-9:
                    r.run = None
                    self._finished(name, Result(run.kind, run.goal_id, False, "TIMEOUT"))
                continue
            if isinstance(run, OrderRun) and stops_for_lost_contact(self.params, r, self.t):
                result = run.done(False, HEARTBEAT_LOST)
            else:
                result = run.step(self, r)
            if result is not None:
                r.run = None
                view = self.traffic.robot_view(name) if self.traffic is not None else {}
                self.order_metrics.run_ended(self.t, name, run, result, r, view)
                self._finished(name, result)
            elif isinstance(run, OrderRun):
                self._feedback(name, run)

    def clear_to(self, r, new) -> bool:
        """r이 지금 자리에서 new까지 곧게 가는 동안 다른 로봇 몸에 닿지 않는다
        (2r 밖이거나 멀어진다).
        닿으면 세계가 그 걸음을 막고 센다: 충돌 검사(blocked_ahead)가 놓친 것이다."""
        least = 2 * self.radius
        for o in self.robots.values():
            if o is r or o.removed:
                continue
            gap = segment_gap(o.pose, r.pose, new)
            if gap < least - 1e-9 and gap < math.dist(r.pose, o.pose) - 1e-12:
                self.guard_stops += 1
                return False
        return True
