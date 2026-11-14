# ---- 호출에 보낼 로봇 고르기 ----
from __future__ import annotations

from dataclasses import dataclass

from fleet_core.fleet_map import distance


# ---- 후보 로봇 정보 ----
@dataclass(frozen=True)
class RobotInfo:
    robot_id: str
    node: str
    called: bool = False
    eligible: bool = True


# ---- 도크까지 거리와 호출 여부로 점수 매겨 선택 ----
class RobotSelector:
    def __init__(self, site, params):
        self._site = site
        self._params = params
        self._dock = next(s.dock for s in site.stations.values() if s.action == "fill")

    def pick(self, calls: int, robots: list[RobotInfo]) -> set[str]:
        pool = sorted((r for r in robots if r.eligible),
                      key=lambda r: (self._score(r), r.robot_id))
        return {r.robot_id for r in pool[:max(calls, 0)]}

    def _score(self, robot: RobotInfo) -> float:
        metres = distance(self._site, robot.node, self._dock)
        score = self._params.score_distance_weight * metres
        return score + (self._params.score_same_call if robot.called else 0.0)
