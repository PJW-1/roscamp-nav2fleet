# ---- 믹스인들을 모아 ZoneTraffic 조립 ----
from __future__ import annotations

from .backoff import Backoff
from .base import ZoneBase
from .blocked import Blocked
from .calls import Calls
from .drive import Drive
from .goal_life import GoalLife
from .goals import Goals
from .health import Health
from .move_orders import MoveOrders
from .order_answers import OrderAnswers
from .placement import Placement
from .progress import Progress
from .redock import Redock
from .results import Results
from .turns import Turns
from .watch import Watch
from .yard_retreat import YardRetreat


# ---- ZoneTraffic 클래스와 틱 순서 ----
class ZoneTraffic(Calls, Goals, Turns, Drive, MoveOrders, OrderAnswers, Results, Redock, Progress,
                  Placement, Blocked, YardRetreat, Health, Watch, Backoff, GoalLife, ZoneBase):
    # ---- 조회 위임 ----
    @property
    def everyone_placed(self) -> bool:
        return self._views.everyone_placed

    def live_block(self, keys, point, radius: float, now: float) -> bool:
        return self._views.live_block(keys, point, radius, now)

    def zones_view(self) -> list:
        return self._views.zones_view()

    def robot_view(self, robot) -> dict:
        return self._views.robot_view(robot)

    def status_view(self, robot) -> dict:
        return self._views.status_view(robot)

    # ---- 막힘 표시 확인 응답 ----
    def on_clear_check(self, robot, check_id, answer) -> None:
        said = self._mark_book.answer(robot, check_id, answer)
        if said is not None:
            self._event(*said, robot)

    # ---- 틱 한 번 ----
    def tick(self) -> list:
        now = self._clock()
        everyone_here = self.tick_begin(now)
        stopped = self.tick_progress(now)
        if self.running and everyone_here:
            self.tick_pick(stopped)
            for robot in self.robots:
                if self.available(robot) and robot not in stopped:
                    self.step(robot)
        return self.take_output()

    # ---- 틱 시작 점검 ----
    def tick_begin(self, now: float) -> bool:
        self._capacity.check(len(self._robots))
        everyone_here = self._check_startup(now)
        self._check_comm(now)
        self._finish_recoveries(now)
        return everyone_here

    # ---- 만료 처리와 로봇별 감시 ----
    def tick_progress(self, now: float) -> frozenset:
        self._expire_blocked(now)
        self._mark_book.expire(now)
        self._out.extend(self._mark_book.ask(now))
        for m in self._robots.values():
            if m.available:
                self._judge_arrival(m, now)
                self._watch_stall(m, now)
                self._silent.watch_idle(m, now, self.running)
                self._watch_retreat_wait(m, now)
            self._start_watch.watch(m, now, self.running)
            self._silent.watch_silent(m, now, self.running)
        self._watch_near(now)
        return self._watch_deadlock(now)
