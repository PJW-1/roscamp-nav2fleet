"""바퀴와 주차 추적: 출고 도크에서 작업 도크나 주차 자리까지, 주차 자리에 선 로봇."""
from __future__ import annotations

import math

# 경기 종료 때 이 안에 선 로봇은 주차한 것이다. 관제의 arrival_tolerance_m와 같다(더 좁으면 경기가 안 끝난다)
PARK_NEAR_M = 0.15
# 바퀴 추적에서 주차 자리에 들어섰다고 보는 거리. 지나가기만 한 로봇을 세지 않게 더 좁다
LAP_PARK_M = 0.10
ON_THE_WAY = ("TO_WORK", "TO_OUTBOUND")  # 스테이션으로 가는 중이거나 그 줄에 선 로봇의 task_type


class Laps:
    def _parked_at(self, x: float, y: float, near: float = PARK_NEAR_M) -> bool:
        return any(math.dist((x, y), spot) <= near for spot in self._parking)


    def _place(self, x: float, y: float, zone) -> str:
        """한 바퀴 추적의 자리: out, work, park 또는 빈 값."""
        if zone in self._out_zones:
            return "out"
        if zone in self._work_zones:
            return "work"
        return "park" if self._parked_at(x, y, LAP_PARK_M) else ""

    def _unbound_on_board(self) -> int:
        return sum(order not in self._bound_now for order in self.board)

    def _track_lap(self, t: float, robot: str, place: str) -> None:
        """출고 도크 구역에서 작업 도크나 주차 자리 중 먼저 닿는 곳까지 로봇을 따라간다."""
        if not place or self._last_zone.get(robot) == place:
            return
        self._last_zone[robot] = place
        zone = place
        if place == "out":
            self._from_outbound.add(robot)
        elif robot not in self._from_outbound:
            return
        elif place == "park" and self._tasks.get(robot) in ON_THE_WAY:
            # 구역 관제가 스테이션행 로봇을 주차 자리에서 기다리게 해도 바퀴는 안 끊긴다. WAIT(주차시킴)만 센다
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
