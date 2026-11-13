from __future__ import annotations

from pinky_fsm.costmap import Grid
from pinky_fsm.geometry import Polyline
from pinky_fsm.nav_goal import MODE_NAV_GOAL
from pinky_fsm.order_queue import Seg

KEEPOUT_COST = 100
SAMPLE_M = 0.05


def segment_problem(seg: Seg, map_grid: Grid | None, keepout: Grid | None) -> str:
    if seg.mode == MODE_NAV_GOAL:
        points = [seg.target[:2]] if seg.target is not None else []
    else:
        keepout = None
        if len(seg.path) > 1:
            line = Polyline([p[:2] for p in seg.path])
            points = line.sample(0.0, line.length, SAMPLE_M)
        else:
            points = [p[:2] for p in seg.path]
    for x, y in points:
        if map_grid is not None and map_grid.cost_at(x, y) is None:
            return f"({x:.2f}, {y:.2f}) is outside the map"
        cost = keepout.cost_at(x, y) if keepout is not None else None
        if cost is not None and cost >= KEEPOUT_COST:
            return f"({x:.2f}, {y:.2f}) is in keepout"
    return ""
