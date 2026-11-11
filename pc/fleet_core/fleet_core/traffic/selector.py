"""채우기 스테이션으로 부를 로봇을 고른다.

순수 함수다. 교통 층이 로봇마다 RobotInfo를 주고 결과를 적용한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from fleet_core.fleet_map import route


@dataclass(frozen=True)
class RobotInfo:
    robot_id: str
    node: str  # 거리를 재는 출발 지점
    called: bool = False
    eligible: bool = True  # 쓸 수 있고, 스테이션에 잡혀 있지 않고, 지정 배차도 없다


class RobotSelector:
    def __init__(self, site, params):
        self._site = site
        self._params = params
        self._dock = next(s.dock for s in site.stations.values() if s.action == "fill")

    def pick(self, calls: int, robots: list[RobotInfo]) -> set[str]:
        """부를 로봇들, 최대 `calls`대. 점수가 같으면 이름 순서다."""
        pool = sorted((r for r in robots if r.eligible),
                      key=lambda r: (self._score(r), r.robot_id))
        return {r.robot_id for r in pool[:max(calls, 0)]}

    def _score(self, robot: RobotInfo) -> float:
        way = route(self._site, robot.node, self._dock)
        metres = math.inf if way is None else sum(link.length for link in way)
        score = self._params.score_distance_weight * metres
        return score + (self._params.score_same_call if robot.called else 0.0)
