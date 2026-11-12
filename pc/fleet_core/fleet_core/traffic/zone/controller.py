"""구역 교통 제어기: 지점과 구역의 그래프 위에서 "어느 구역에 들어가도 되나"를 정한다.

넓은 구역(shared)은 Nav2 goal 하나로, 고정 경로 구간(lane, dock)은 FollowPath 하나로 보낸다.
바깥 API는 traffic/api.py의 TrafficApi다. 조각마다 하는 일은 아래 모듈에 있다.

| 모듈 | 하는 일 |
|---|---|
| grants | 구역 정원, 지점, 한 줄 구역의 방향. 구간을 한꺼번에 잡는다 |
| lane_policy | 한 줄 구역의 방향: ETA, 히스테리시스, 넘겨주기 |
| trips | 가는 길을 구간과 조각(명령)으로 나눈다 |
| goals | 길 고르기, 구간 잡기 |
| turns | 목적지 후보(도크, 대기 자리, 주차), 도크 차례 |
| drive | 로봇 한 대의 한 걸음, 명령 내기 |
| results | 명령의 결과 |
| progress | 상태 보고: 뒤 구역 놓기 |
| placement | 로봇을 지도에 올리기: 선 지점 고르기, 겹친 보고, 거절 |
| blocked | 막힘: 기다림, 표시, 다른 길, 후진 |
| block_marks | 장애물 위치로 한 막힘 표시: 든 연결, 다른 로봇이면 비키기, 다시 막히면 두 배 |
| health | 통신 끊김과 복구, 치움, 다시 투입 |
| watch | 멈춤, 구역 밖, 서로 막음, 대기 순환 |
| silent | 할 일 없이 섬, 조용히 멈춤(갈 곳이 있는데 관제가 말이 없음) |
| nostart | goal을 받고 움직이기 시작하지 않음 |
| capacity | 시작 때 넓은 구역 정원의 합과 로봇 수 |
| backoff | 서로 막은 로봇을 뒤로 조금 물리기(기본 꺼짐) |
| calls | 배차와 호출 |
| move_orders | order 모델(use_order_model): 조각 목록, base와 horizon, update |
| order_answers | order 모델에서 로봇이 알려 온 것: 피드백, 결과, 붙지 못한 update |
| views | 밖에서 읽는 것 |
"""
from __future__ import annotations

from .backoff import Backoff
from .base import ZoneBase
from .block_marks import BlockMarks
from .blocked import Blocked
from .calls import Calls
from .capacity import Capacity
from .drive import Drive
from .goals import Goals
from .health import Health
from .move_orders import MoveOrders
from .nostart import NoStart
from .order_answers import OrderAnswers
from .placement import Placement
from .progress import Progress
from .results import Results
from .silent import Silent
from .turns import Turns
from .views import Views
from .watch import Watch


class ZoneTraffic(Calls, Goals, Turns, Drive, MoveOrders, OrderAnswers, Results, Progress,
                  Placement, Blocked, BlockMarks, Health, Watch, Backoff, Silent, NoStart,
                  Capacity, Views, ZoneBase):
    def tick(self) -> list:
        """한 주기 전체. FleetManager 없이 교통 층만 돌릴 때 쓴다."""
        now = self._clock()
        everyone_here = self.tick_begin(now)
        stopped = self.tick_progress(now)
        if self.running and everyone_here:
            self.tick_pick(stopped)
            for robot in self.robots:
                if self.available(robot) and robot not in stopped:
                    self.step(robot)
        return self.take_output()

    def tick_begin(self, now: float) -> bool:
        self._check_capacity()
        everyone_here = self._check_startup(now)
        self._check_comm(now)
        self._finish_recoveries(now)
        return everyone_here

    def tick_progress(self, now: float) -> frozenset:
        self._expire_blocked(now)
        self._expire_marks(now)
        for m in self._robots.values():
            if m.available:
                self._judge_arrival(m, now)
                self._watch_stall(m, now)
                self._watch_idle(m, now)
            self._watch_start(m, now)
            self._watch_silent(m, now)
        self._watch_near(now)
        return self._watch_deadlock(now)
