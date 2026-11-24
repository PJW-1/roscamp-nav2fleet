"""구역 교통 층만 돌리는 작은 세계(시험 전용). 로봇은 명령의 점을 곧게 일정 속도로 가고 장애물 앞에서는 서서 BLOCKED를 보고한다.
매 걸음 구역 점유와 한 줄 구역의 방향을 검사한다."""
from __future__ import annotations

import math
import os
from typing import NamedTuple

from fleet_core.common.commands import (
    CancelCmd,
    DispatchResult,
    DockCmd,
    EventOut,
    MoveCmd,
    MoveOrderCmd,
    RetreatCmd,
)
from fleet_core.common.params import load_params
from fleet_core.fleet_map import load_fleet_map
from fleet_core.manager import FleetManager
from fleet_core.traffic.zone.controller import ZoneTraffic
from zone_order_run import OrderRun

DT = 0.1
DOCK_S = 2.0
STARTS = {"pode1": "P-1", "pode2": "P-2", "pode3": "P-3"}
# 틱 순서: "zone"은 ZoneTraffic.tick, "manager"는 운영의 FleetManager.tick(주문 층과 팔 포함)
TICK_ORDERS = ("zone", "manager")
TICK_ORDER = os.environ.get("ZONE_TICK_ORDER", "manager")


class TappedManager(FleetManager):
    """운영 FleetManager 그대로. 코어 사이로 오간 항목을 tap에 남기기만 한다."""

    def __init__(self, *args):
        self.tap: list = []
        self.seen: list = []  # (단계, 틱 번호, 항목). 단계는 "tick" 또는 "input"
        self.ticks = 0
        self._in_tick = False
        super().__init__(*args)

    def tick(self) -> list:
        self.ticks += 1
        self._in_tick = True
        try:
            return super().tick()
        finally:
            self._in_tick = False

    # 하네스는 입력을 교통에 바로 넣으므로 매 입력의 _input 대신 시계를 그때그때 읽는다
    def _frozen(self) -> float:
        return self._clock()

    def _route(self, item) -> None:
        self.tap.append(item)
        self.seen.append(("tick" if self._in_tick else "input", self.ticks, item))
        super()._route(item)


class Leg(NamedTuple):
    """보낸 구간 하나: MoveCmd 하나, RetreatCmd 하나, 또는 order의 base 조각 하나."""
    robot: str
    goal_id: int
    mode: str
    target_id: str
    x: float
    y: float
    yaw: float
    path: tuple


class Bot:
    def __init__(self, name: str, x: float, y: float):
        self.name, self.x, self.y, self.yaw = name, x, y, 0.0
        self.cmd = None  # 달리는 MoveCmd나 RetreatCmd
        self.run = None  # 달리는 order(OrderRun)
        self.points: list = []
        self.dock = None  # (DockCmd, 끝나는 시각)
        self.heading = (0.0, 0.0)  # 마지막으로 움직인 방향
        self.stuck = False  # 장애물 앞에 서 있다


class Harness:
    def __init__(self, layout="d2", starts=None, params=None, speed=0.12, site=None,
                 tick_order=None, manager_inputs=False):
        self.site = site or load_fleet_map(layout)  # site: 시험에서 고친 지도
        self.params = params or load_params()
        self.now = 0.0
        starts = dict(STARTS if starts is None else starts)
        self.tick_order = tick_order or TICK_ORDER
        if self.tick_order not in TICK_ORDERS:
            raise ValueError(f"tick_order must be one of {TICK_ORDERS}, not {self.tick_order!r}")
        self.manager = None
        if self.tick_order == "manager":
            self.manager = TappedManager(self.site, self.params, list(starts), lambda: self.now)
            self.traffic = self.manager.traffic
        else:
            self.traffic = ZoneTraffic(self.site, self.params, list(starts), lambda: self.now)
        if manager_inputs and self.manager is None:
            raise ValueError("manager_inputs needs tick_order 'manager'")
        # 로봇의 보고와 답을 받는 곳: 기본은 교통에 바로, manager_inputs면 운영 입력 함수(+_pump)
        # call()은 늘 교통에 바로, 주문판은 시험이 self.manager.on_order_board로 넣는다
        self.inputs = self.manager if manager_inputs else self.traffic
        self.bots = {name: Bot(name, *self.site.node_pose(node)) for name, node in starts.items()}
        self.speed = speed
        self.events: list[EventOut] = []
        self.commands: list = []
        self.results: list[DispatchResult] = []
        self.silent: set[str] = set()  # 보고하지 않는 로봇(통신 끊김)
        self.obstacles: list[tuple[float, float, float]] = []  # (x, y, 반지름)
        self.dock_fails: set[str] = set()
        self.problems: list[str] = []
        self.log: list[str] = []  # 시각을 붙인 사건과 명령. 실패한 시험을 읽을 때 쓴다
        self._late: list = []  # 다음 걸음에 관제에 닿을 로봇의 답 (로봇, 메서드 이름, 인자)
        self.hold_answers = False  # 켜면 로봇의 답이 관제에 닿지 않고 쌓인다
        self.answers: list = []  # 관제에 닿은 로봇의 답 (시각, 로봇, 메서드 이름, 인자) 차례대로
        self.goal_fault: dict[str, str] = {}  # 로봇 -> 다음 update goal에 일어날 일(on_goal_*)
        # 로봇 -> 이 방식의 order 조각 끝에서 말없이 선다. 선 채로 남는다(answer_at으로만 풀린다)
        self.hold_after: dict[str, str] = {}
        self._ids = iter(range(1000, 100000))

    # ---- 시험이 부르는 것 ----

    def call(self, station="WORK", robot="") -> int:
        # 주문 층을 거치지 않는 호출. manager_inputs와 상관없이 교통에 바로 넣는다
        goal_id = next(self._ids)
        self.traffic.on_dispatch(goal_id, station, robot)
        return goal_id

    def report(self) -> None:
        """로봇들이 상태를 한 번 보고한다. 관제는 돌지 않는다."""
        for bot in self.bots.values():
            if bot.name not in self.silent:
                self.inputs.on_robot_state(bot.name, bot.x, bot.y, bot.yaw, self._state(bot))

    def step(self) -> None:
        self.now = round(self.now + DT, 6)
        late = [] if self.hold_answers else self._late
        if late:
            self._late = []
        for name, method, args in late:
            if name not in self.silent:
                self.answers.append((self.now, name, method, args))
                getattr(self.inputs, method)(name, *args)
        for bot in self.bots.values():
            if bot.name not in self.silent or bot.run is not None:
                self._move(bot)  # order를 받은 로봇은 말이 끊겨도 base 끝까지 간다
        self.report()
        for item in self._tick():
            self._apply(item)
        self._check()

    def _tick(self) -> list:
        if self.manager is None:
            return self.traffic.tick()
        out = self.manager.tick()
        tapped, self.manager.tap = self.manager.tap, []
        seen = {id(item) for item in tapped}
        # 코어 사이 항목(DispatchResult 등)까지 오간 차례대로 본다
        return tapped + [item for item in out if id(item) not in seen]

    def run(self, seconds: float) -> None:
        for _ in range(round(seconds / DT)):
            self.step()

    def run_until(self, done, timeout_s: float = 300.0) -> bool:
        for _ in range(round(timeout_s / DT)):
            if done():
                return True
            self.step()
        return done()

    def view(self, robot: str) -> dict:
        return self.traffic.robot_view(robot)

    def phase(self, robot: str) -> str:
        return self.view(robot)["state"]

    def codes(self, robot: str = "") -> list[str]:
        return [e.code for e in self.events if not robot or e.robot == robot]

    def sent(self, kind, robot: str = "") -> list:
        return [c for c in self.commands if isinstance(c, kind) and (not robot or c.robot == robot)]

    def moves(self, robot: str = "") -> list[Leg]:
        """새로 낸 이동 goal마다 첫 구간(MoveCmd 하나, 또는 새 order의 첫 base 조각). 끈 길의 RetreatCmd는 세지 않는다."""
        return [self._legs(c)[0] for c in self.sent((MoveCmd, MoveOrderCmd), robot)
                if (not isinstance(c, MoveOrderCmd) or c.update_id == 0) and self._legs(c)]

    def legs(self, robot: str = "") -> list[Leg]:
        """보낸 구간을 차례로: MoveCmd, RetreatCmd는 하나씩, order는 base 조각마다 하나씩."""
        return [leg for c in self.sent((MoveCmd, RetreatCmd, MoveOrderCmd), robot)
                for leg in self._legs(c)]

    def retreats(self, robot: str = "") -> list[Leg]:
        """보낸 물러나기: RetreatCmd, 또는 order 모델이면 order의 retreat 조각."""
        return [leg for leg in self.legs(robot) if leg.mode == "retreat"]

    def answer_at(self, robot: str, node: str, move: bool = True) -> None:
        """로봇이 node에 닿았다고 손으로 답하고 move면 로봇을 그 자리에 옮긴다. order면 node에서 끝나는 다음 조각까지 끝냈다는 피드백을 낸다."""
        bot, run = self.bots[robot], self.bots[robot].run
        if run is None:
            if not isinstance(bot.cmd, MoveCmd):
                raise AssertionError(f"{robot} runs no MoveCmd or accepted order")
            x, y = self.site.node_pose(node)
            if move:
                bot.x, bot.y = x, y
            cmd, bot.cmd, bot.points = bot.cmd, None, []
            self.inputs.on_move_result(robot, cmd.goal_id, True, "")
            return
        ahead = [s.seq_id for s in run.taken if s.seq_id > run.done_seq and s.end_node == node]
        if not ahead:
            raise AssertionError(f"{robot} has no unfinished accepted segment ending at {node}")
        seq = min(ahead)
        if move:
            bot.x, bot.y = self.site.node_pose(node)
        bot.points = []
        run.done_seq, run.held = seq, False
        if run.current is not None and run.current.seq_id <= seq:
            run.current = None
        run.queue = [s for s in run.queue if s.seq_id > seq]
        run.waiting = run.current is None and not run.queue and bool(run.horizon)
        run.told = (run.done_seq, run.waiting, run.goal_id)
        if run.finished:
            bot.run = None
            self.inputs.on_move_result(robot, run.goal_id, True, "", seq)
        else:
            self.inputs.on_move_feedback(robot, run.goal_id, seq, run.waiting)

    def _legs(self, c) -> list[Leg]:
        if isinstance(c, MoveCmd):
            return [Leg(c.robot, c.goal_id, c.mode, c.target_id, c.x, c.y, c.yaw, c.path)]
        if isinstance(c, RetreatCmd):
            x, y = c.path[-1][:2] if c.path else (0.0, 0.0)
            end = self.site.nearest_node(x, y) if c.path else ""
            return [Leg(c.robot, c.goal_id, "retreat", end, x, y, 0.0, c.path)]
        return [Leg(c.robot, c.goal_id, s.mode, s.end_node, s.x, s.y, s.yaw, s.path)
                for s in c.base]

    def result_of(self, goal_id: int):
        return next((r for r in self.results if r.goal_id == goal_id), None)

    # ---- 세계 ----

    def _state(self, bot: Bot) -> str:
        moving = bot.cmd is not None or bot.run is not None
        if bot.stuck and moving:
            return "BLOCKED"
        if bot.dock is not None:
            return "DOCKING"
        return "MOVING" if moving else "IDLE"

    def _apply(self, item) -> None:
        if isinstance(item, EventOut):
            self.events.append(item)
            self.log.append(f"{self.now:6.1f} {item.robot:7} {item.code}: {item.detail}")
            return
        self.log.append(f"{self.now:6.1f} {getattr(item, 'robot', ''):7} {type(item).__name__} "
                        f"{self._detail(item)}")
        if isinstance(item, DispatchResult):
            self.results.append(item)
            return
        self.commands.append(item)
        bot = self.bots.get(getattr(item, "robot", ""))
        if bot is None or bot.name in self.silent:
            return
        if isinstance(item, MoveCmd):
            bot.cmd = item
            bot.points = [p[:2] for p in item.path[1:]] if item.path else [(item.x, item.y)]
        elif isinstance(item, RetreatCmd):
            bot.cmd, bot.points = item, [p[:2] for p in item.path[1:]]
        elif isinstance(item, MoveOrderCmd):
            self._take_order(bot, item)
        elif isinstance(item, DockCmd):
            bot.dock = (item, self.now + DOCK_S)
        elif isinstance(item, CancelCmd):
            if bot.cmd is not None and bot.cmd.goal_id == item.goal_id:
                bot.cmd, bot.points = None, []
            if bot.run is not None and bot.run.goal_id == item.goal_id:
                bot.run, bot.points = None, []
            if bot.dock is not None and bot.dock[0].goal_id == item.goal_id:
                bot.dock = None

    def _move(self, bot: Bot) -> None:
        if bot.dock is not None:
            cmd, until = bot.dock
            if self.now >= until:
                bot.dock = None
                self.inputs.on_dock_result(bot.name, cmd.goal_id, bot.name not in self.dock_fails,
                                           0.4, 0.3, 0.5)
            return
        bot.stuck = False
        if bot.run is not None:
            self._run_order(bot)
            return
        if bot.cmd is None:
            return
        speed = bot.cmd.speed_mps if isinstance(bot.cmd, RetreatCmd) else self.speed
        self._advance(bot, speed)
        if not bot.points and not bot.stuck:
            cmd, bot.cmd = bot.cmd, None
            if isinstance(cmd, RetreatCmd):
                self.inputs.on_retreat_result(bot.name, cmd.goal_id, True, "")
            else:
                bot.yaw = cmd.yaw
                self.inputs.on_move_result(bot.name, cmd.goal_id, True, "")

    @staticmethod
    def _detail(item) -> str:
        if not isinstance(item, MoveOrderCmd):
            return getattr(item, "target_id", "")
        ends = [[segment.end_node for segment in part] for part in (item.base, item.horizon)]
        return f"{item.order_id} update {item.update_id}: base {ends[0]} horizon {ends[1]}"

    def _take_order(self, bot: Bot, cmd) -> None:
        run = bot.run
        if cmd.update_id == 0:
            bot.run, bot.cmd, bot.points = OrderRun(cmd), None, []
            return
        fault = self.goal_fault.pop(bot.name, "")
        if fault:  # 이 goal은 로봇에 닿지 못했다
            self._late.append((bot.name, fault, (cmd.goal_id,)))
            return
        why = "no running order" if run is None else run.refusal(cmd)
        done = run.done_seq if run is not None else 0
        if why:
            self._late.append((bot.name, "on_move_result",
                               (cmd.goal_id, False, f"ORDER_UPDATE_REJECTED: {why}", done)))
        else:
            self._late.append((bot.name, "on_move_result",
                               (run.stitch(cmd), False, "SUPERSEDED", done)))

    def _run_order(self, bot: Bot) -> None:
        """order의 base 조각을 차례로 달린다. 조각 사이에서 관제를 기다리지 않는다."""
        run = bot.run
        if run.held:
            return
        segment = run.next_segment()
        if segment is not None:
            if not bot.points:
                bot.points = ([p[:2] for p in segment.path[1:]] if segment.path
                              else [(segment.x, segment.y)])
            retreat = segment.mode == "retreat"
            self._advance(bot, self.params.retreat_speed_mps if retreat else self.speed)
            if not bot.points and not bot.stuck:
                bot.yaw = segment.yaw
                if self.hold_after.get(bot.name) == segment.mode:
                    run.held = True  # 조각 끝에 닿았지만 끝냈다고 하지 않고 선다
                    return
                run.finish_segment()
        if run.finished:
            bot.run = None
            self._late.append((bot.name, "on_move_result", (run.goal_id, True, "", run.done_seq)))
        elif run.told != (run.done_seq, run.waiting, run.goal_id):
            run.told = (run.done_seq, run.waiting, run.goal_id)
            self._late.append((bot.name, "on_move_feedback",
                               (run.goal_id, run.done_seq, run.waiting, "")))

    def _advance(self, bot: Bot, speed: float) -> None:
        """bot.points를 따라 한 걸음. 장애물에 막히면 bot.stuck을 켠다."""
        left = speed * DT
        while bot.points and left > 0.0:
            px, py = bot.points[0]
            gap = math.hypot(px - bot.x, py - bot.y)
            if gap > 1e-9:
                bot.heading = ((px - bot.x) / gap, (py - bot.y) / gap)
            go = min(gap, left)
            x, y = bot.x + bot.heading[0] * go, bot.y + bot.heading[1] * go
            if self._hits(bot, x, y):
                bot.stuck = True
                return
            bot.x, bot.y, left = x, y, left - go
            if gap <= go + 1e-12:
                bot.x, bot.y = px, py
                bot.points.pop(0)

    def _hits(self, bot: Bot, x: float, y: float) -> bool:
        """(x, y)로 가는 걸음이 장애물 안으로 더 들어가는가. 멀어지는 걸음은 막지 않는다."""
        for ox, oy, radius in self.obstacles:
            after = math.hypot(x - ox, y - oy)
            if after < radius and after < math.hypot(bot.x - ox, bot.y - oy):
                return True
        return False

    def _check(self) -> None:
        grants, site = self.traffic.grants, self.site
        for bot in self.bots.values():
            view = self.view(bot.name)
            if not view["on_map"] or view["fault"] or view["comm"]:
                continue
            held = grants.held_by(bot.name)
            gap = min((site.distance_to_zone(bot.x, bot.y, zone) for zone in held), default=1.0)
            if gap > 0.01:
                self.problems.append(f"{self.now:.1f} {bot.name} at ({bot.x:.2f}, {bot.y:.2f}) "
                                     f"is outside its zones {list(held)}")
        for zone, spec in site.zones.items():
            inside = [b for b in self.bots.values() if site.zone_at(b.x, b.y) == zone]
            if spec.kind == "lane" and len(inside) > 1:
                ways = {round(b.heading[0]) for b in inside
                        if b.cmd is not None or b.run is not None}
                if len(ways) > 1:
                    self.problems.append(f"{self.now:.1f} two ways inside {zone}")
            if spec.kind == "dock" and len(inside) > 1:
                self.problems.append(f"{self.now:.1f} {[b.name for b in inside]} share {zone}")
