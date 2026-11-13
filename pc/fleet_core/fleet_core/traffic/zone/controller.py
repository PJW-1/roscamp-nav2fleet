# ---- 믹스인들을 모아 ZoneTraffic 조립 ----
from __future__ import annotations

from .backoff import Backoff
from .base import ZoneBase
from .block_marks import BlockMarks
from .blocked import Blocked
from .calls import Calls
from .capacity import Capacity
from .clear_checks import ClearChecks
from .detours import Detours
from .drive import Drive
from .goals import Goals
from .health import Health
from .localization import Localization
from .move_orders import MoveOrders
from .nostart import NoStart
from .order_answers import OrderAnswers
from .placement import Placement
from .progress import Progress
from .redock import Redock
from .results import Results
from .silent import Silent
from .turns import Turns
from .views import Views
from .watch import Watch
from .yard_retreat import YardRetreat


# ---- ZoneTraffic 클래스와 틱 순서 ----
class ZoneTraffic(Calls, Goals, Turns, Drive, MoveOrders, OrderAnswers, Results, Redock, Progress,
                  Placement, Blocked, BlockMarks, YardRetreat, Detours, ClearChecks, Health, Watch,
                  Localization, Backoff, Silent, NoStart, Capacity, Views, ZoneBase):
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
        self._check_capacity()
        everyone_here = self._check_startup(now)
        self._check_comm(now)
        self._finish_recoveries(now)
        return everyone_here

    # ---- 만료 처리와 로봇별 감시 ----
    def tick_progress(self, now: float) -> frozenset:
        self._expire_blocked(now)
        self._expire_marks(now)
        self._ask_clear(now)
        for m in self._robots.values():
            if m.available:
                self._judge_arrival(m, now)
                self._watch_stall(m, now)
                self._watch_idle(m, now)
                self._watch_retreat_wait(m, now)
            self._watch_start(m, now)
            self._watch_silent(m, now)
        self._watch_near(now)
        return self._watch_deadlock(now)
