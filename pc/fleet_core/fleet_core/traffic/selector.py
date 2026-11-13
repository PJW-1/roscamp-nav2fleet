from __future__ import annotations

import math
from dataclasses import dataclass

from fleet_core.fleet_map import route


@dataclass(frozen=True)
class RobotInfo:
    robot_id: str
    node: str
    called: bool = False
    eligible: bool = True


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
        way = route(self._site, robot.node, self._dock)
        metres = math.inf if way is None else sum(link.length for link in way)
        score = self._params.score_distance_weight * metres
        return score + (self._params.score_same_call if robot.called else 0.0)
