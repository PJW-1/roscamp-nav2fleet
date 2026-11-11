"""horizon 조각 미리 검사(order 추가지시서 3절 5번, 계약 0.11-draft 제안). ROS를 모른다.

지도 밖으로 나가거나 keepout 칸을 지나는 조각을 찾는다. nav_goal은 길을 Nav2가 짜므로 target_pose만
본다. 문제가 있어도 경고(HORIZON_INVALID)일 뿐이고 이동은 그대로 간다.
"""
from __future__ import annotations

from pinky_fsm.costmap import Grid
from pinky_fsm.geometry import Polyline
from pinky_fsm.nav_goal import MODE_NAV_GOAL
from pinky_fsm.order_queue import Seg

KEEPOUT_COST = 100  # 마스크의 막힌 칸(map_server trinary의 점유 칸). keepout 필터에서 LETHAL이다
SAMPLE_M = 0.05  # 경로 점 사이를 이 간격으로 본다(5 cm 지도 칸)


def segment_problem(seg: Seg, map_grid: Grid | None, keepout: Grid | None) -> str:
    """문제가 없으면 빈 문자열. 지도나 마스크를 아직 못 받았으면 그 검사는 건너뛴다."""
    if seg.mode == MODE_NAV_GOAL:
        points = [seg.target[:2]] if seg.target is not None else []
    elif len(seg.path) > 1:
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
