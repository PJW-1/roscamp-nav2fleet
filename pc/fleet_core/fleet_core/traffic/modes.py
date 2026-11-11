"""traffic_mode로 교통 층을 고른다. 주문 코어와 FleetManager의 바깥 API는 그대로다."""
from __future__ import annotations

from fleet_core.traffic.controller import TrafficController

LANES = "lanes"
MODES = (LANES,)


def build_traffic(site, params, robots, clock, next_goal_id):
    """lanes는 레인 제어기 그대로."""
    mode = params.traffic_mode
    if mode == LANES:
        return TrafficController(site, params, robots, clock, next_goal_id)
    raise ValueError(f"traffic_mode must be one of {', '.join(MODES)}, not {mode!r}")


def zone_space(site, params):
    """Something that measures /fleet/zones names, for nodes outside the controller."""
    mode = params.traffic_mode
    if mode == LANES:
        return site
    raise ValueError(f"traffic_mode must be one of {', '.join(MODES)}, not {mode!r}")
