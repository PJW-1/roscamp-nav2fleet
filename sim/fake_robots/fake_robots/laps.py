"""바퀴와 주차 추적: 출고 도크에서 작업 도크나 주차 자리까지, 주차 자리에 선 로봇."""
from __future__ import annotations

import math

PARK_NEAR_M = 0.10  # 주차 지점에서 이 안이면 주차 자리에 있다
ON_THE_WAY = ("TO_WORK", "TO_OUTBOUND")  # 스테이션으로 가는 중이거나 그 줄에 선 로봇의 task_type


class Laps:
    def _parked_at(self, x: float, y: float) -> bool:
        return any(math.dist((x, y), spot) <= PARK_NEAR_M for spot in self._parking)


    def _place(self, x: float, y: float, zone) -> str:
        """한 바퀴 추적의 자리: out, work, park 또는 빈 값."""
        if zone in self._out_zones:
            return "out"
        if zone in self._work_zones:
            return "work"
        return "park" if self._parked_at(x, y) else ""

    def _unbound_on_board(self) -> int:
        return sum(order not in self._bound_now for order in self.board)

    def _track_lap(self, t: float, robot: str, place: str) -> None:
        """출고 도크 구역에서 작업 도크 구역과 주차 자리 중 먼저 닿는 곳까지 로봇을 따라간다.

        미배정 주문이 로봇 수 이상이면 모든 로봇이 호출되므로
        주차 자리로 갈 까닭이 없다.
        """
        if not place or self._last_zone.get(robot) == place:
            return
        self._last_zone[robot] = place
        zone = place
        if place == "out":
            self._from_outbound.add(robot)
        elif robot not in self._from_outbound:
            return
        elif place == "park" and self._tasks.get(robot) in ON_THE_WAY:
            # 구역 관제는 스테이션으로 가는 로봇을 주차 자리에서 기다리게 한다(도크 대기 줄, 구역
            # 기다림. docs/관제_구역교통.md 규칙 5): 바퀴가 끊기지 않는다. WAIT(주차시킴)만 센다
            self._last_zone.pop(robot, None)
            return
        elif place == "work":
            self._from_outbound.discard(robot)
            self.direct_returns += 1
        elif place == "park":
            self._from_outbound.discard(robot)
            unbound = self._unbound_on_board()
            if unbound >= len(self.scenario.robots):
                self.parked_while_backlog += 1
                self._violate(t, f"{robot} entered {zone} with {unbound} unbound orders waiting")


    def _all_parked(self) -> bool:
        for robot in self.scenario.robots:
            if robot in self.scenario.gone_robots:
                continue
            pose = self.poses.get(robot)
            if pose is None or pose[2] != "IDLE":
                return False
            if not self._parked_at(*self._position(robot)):
                return False
        return True
