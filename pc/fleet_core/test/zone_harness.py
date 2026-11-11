"""구역 교통 층만 돌리는 작은 세계. 시험 전용이다.

로봇은 받은 명령의 점들을 곧게, 일정한 속도로 간다. 서로 피하지 않고, 잡음도 없다. 장애물
(원)에 다가가는 걸음은 떼지 않고 서서 BLOCKED를 보고한다. 도킹은 정해진 시간이 걸린다.
길 찾기와 회피까지 흉내 내는 세계는 sim/fake_robots의 world에 있다.
매 걸음마다 "로봇이 쥔 구역 안에 있는가"와 "한 줄 구역 안의 로봇이 같은 방향인가"를 본다.
"""
from __future__ import annotations

import math

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
from fleet_core.traffic.zone.controller import ZoneTraffic
from zone_order_run import OrderRun

DT = 0.1
DOCK_S = 2.0
STARTS = {"pinky1": "P-1", "pinky2": "P-2", "pinky3": "P-3"}


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
    def __init__(self, layout="d2", starts=None, params=None, speed=0.12):
        self.site = load_fleet_map(layout)
        self.params = params or load_params()
        self.now = 0.0
        starts = dict(STARTS if starts is None else starts)
        self.traffic = ZoneTraffic(self.site, self.params, list(starts), lambda: self.now)
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
        self.goal_fault: dict[str, str] = {}  # 로봇 -> 다음 update goal에 일어날 일(on_goal_*)
        self._ids = iter(range(1000, 100000))

    # ---- 시험이 부르는 것 ----

    def call(self, station="WORK", robot="") -> int:
        goal_id = next(self._ids)
        self.traffic.on_dispatch(goal_id, station, robot)
        return goal_id

    def report(self) -> None:
        """로봇들이 상태를 한 번 보고한다. 관제는 돌지 않는다."""
        for bot in self.bots.values():
            if bot.name not in self.silent:
                self.traffic.on_robot_state(bot.name, bot.x, bot.y, bot.yaw, self._state(bot))

    def step(self) -> None:
        self.now = round(self.now + DT, 6)
        late = [] if self.hold_answers else self._late
        if late:
            self._late = []
        for name, method, args in late:
            if name not in self.silent:
                getattr(self.traffic, method)(name, *args)
        for bot in self.bots.values():
            if bot.name not in self.silent or bot.run is not None:
                self._move(bot)  # order를 받은 로봇은 말이 끊겨도 base 끝까지 간다
        self.report()
        for item in self.traffic.tick():
            self._apply(item)
        self._check()

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
                self.traffic.on_dock_result(bot.name, cmd.goal_id, bot.name not in self.dock_fails,
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
                self.traffic.on_retreat_result(bot.name, cmd.goal_id, True, "")
            else:
                bot.yaw = cmd.yaw
                self.traffic.on_move_result(bot.name, cmd.goal_id, True, "")

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
        segment = run.next_segment()
        if segment is not None:
            if not bot.points:
                bot.points = ([p[:2] for p in segment.path[1:]] if segment.path
                              else [(segment.x, segment.y)])
            retreat = segment.mode == "retreat"
            self._advance(bot, self.params.retreat_speed_mps if retreat else self.speed)
            if not bot.points and not bot.stuck:
                bot.yaw = segment.yaw
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
