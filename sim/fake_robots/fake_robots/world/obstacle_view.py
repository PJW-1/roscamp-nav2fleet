"""머리 없는 세계의 local costmap 흉내(추가 지시서 3의 4-1, 4-3). 순수 함수다.

- seen_ahead: BLOCKED로 선 로봇이 함께 보고할 장애물 위치. 가던 쪽(보는 방향) 띠 안, 로봇 중심에서
  look_m 안의 가장 가까운 원 둘레의 점이다. 원은 다른 로봇과 시나리오의 물체다(costmap은 둘 다
  본다).
- clear_answer: CheckClear의 답. 로봇이 그 점에서 check_range_m 안이고 사이에 진열대가 없으면
  시나리오의 물체로 답하고, 아니면 unknown이다.
"""
from __future__ import annotations

import math

from fleet_core.common.commands import BLOCKED_AT, CLEAR, UNKNOWN
from fleet_core.fleet_map.geometry import segment_rect_distance

HALF_WIDTH_M = 0.12  # 가던 쪽 띠의 반폭(pinky_fsm.obstacle_report와 같다)


def seen_ahead(x: float, y: float, yaw: float, discs, look_m: float,
               half_width_m: float = HALF_WIDTH_M) -> tuple[float, float] | None:
    ux, uy = math.cos(yaw), math.sin(yaw)
    best = None
    for ox, oy, reach in discs:
        gap = math.hypot(ox - x, oy - y)
        if gap <= reach:
            continue  # 몸 안에 든 원은 보지 않는다
        px, py = ox - (ox - x) / gap * reach, oy - (oy - y) / gap * reach
        ahead = (px - x) * ux + (py - y) * uy
        side = abs(-(px - x) * uy + (py - y) * ux)
        near = math.hypot(px - x, py - y)
        if ahead > 0.0 and side <= half_width_m and near <= look_m and (
                best is None or near < best[0]):
            best = (near, (px, py))
    return best[1] if best else None


def clear_answer(robot_xy, point, radius_m: float, objects, shelves, range_m: float) -> str:
    """objects는 지금 바닥에 있는 물체 (x, y, 반지름)이다."""
    if math.dist(robot_xy, point) > range_m:
        return UNKNOWN
    if any(segment_rect_distance(robot_xy, point, shelf) <= 0.0 for shelf in shelves):
        return UNKNOWN  # 진열대 너머는 라이다가 못 본다
    for ox, oy, reach in objects:
        if math.dist((ox, oy), point) - reach <= radius_m:
            return BLOCKED_AT
    return CLEAR
