"""BLOCKED와 함께 보고할 장애물 위치, 관제의 CheckClear에 줄 답. ROS 없이 돈다.

계약 0.12-draft(추가 지시서 3의 4-1, 4-3, 기준(사용자)). 점과 자세는 모두 격자(local costmap)의
틀이다. map 틀과 오가는 것은 node.py가 한다.
"""
from __future__ import annotations

import math

from pinky_fsm.costmap import Grid

LETHAL = 100  # 장애물 칸 자체. 99(INSCRIBED) 아래는 부풀린 둘레다
AHEAD_HALF_WIDTH_M = 0.12  # 가던 길 앞의 띠 반폭: 발자국 반 대각선 0.085 m + 칸 하나 남짓
CLEAR, BLOCKED, UNKNOWN = "clear", "blocked", "unknown"  # CheckClear.srv의 ANSWER_*
MAX_AGE_S = 2.0  # 이보다 오래된 costmap으로는 답하지 않는다(local costmap은 보통 5 Hz 안팎)
TOUCH_M = 1e-6  # 원이 칸에 꼭 닿으면 걸친 것으로 본다(부동소수점 오차보다 크고 칸보다 작다)


def _cells_near(grid: Grid, x: float, y: float, reach: float):
    """(x, y)에서 reach 안에 중심이 있는 칸들의 (중심 x, 중심 y, 값). 격자 밖 칸은 없다."""
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    dx, dy = x - grid.origin_x, y - grid.origin_y
    gx, gy = c * dx + s * dy, -s * dx + c * dy  # 격자 안 좌표
    res = grid.resolution
    span = int(math.ceil(reach / res)) + 1
    ci, cj = math.floor(gx / res), math.floor(gy / res)
    for j in range(max(0, cj - span), min(grid.height, cj + span + 1)):
        for i in range(max(0, ci - span), min(grid.width, ci + span + 1)):
            lx, ly = (i + 0.5) * res, (j + 0.5) * res
            px, py = grid.origin_x + c * lx - s * ly, grid.origin_y + s * lx + c * ly
            if math.hypot(px - x, py - y) <= reach:
                yield px, py, int(grid.data[j * grid.width + i])


def nearest_ahead(grid: Grid, x: float, y: float, yaw: float, look_m: float,
                  half_width_m: float = AHEAD_HALF_WIDTH_M) -> tuple[float, float] | None:
    """(x, y, yaw)에 선 로봇의 앞 look_m 안, 가던 쪽 띠(반폭 half_width_m) 안의 가장 가까운
    장애물 칸의 중심. 없으면 None."""
    ux, uy = math.cos(yaw), math.sin(yaw)
    best = None
    for px, py, cost in _cells_near(grid, x, y, look_m):
        if cost < LETHAL:
            continue
        ahead = (px - x) * ux + (py - y) * uy
        side = abs(-(px - x) * uy + (py - y) * ux)
        if ahead <= 0.0 or side > half_width_m:
            continue
        gap = math.hypot(px - x, py - y)
        if best is None or gap < best[0]:
            best = (gap, (px, py))
    return best[1] if best else None


def clear_answer(grid: Grid, x: float, y: float, r: float, age_s: float = 0.0) -> str:
    """CheckClear의 답.

    격자가 MAX_AGE_S보다 오래됐으면(갱신이 멈춘 costmap) unknown. r 안에 중심이 있는 장애물 칸이
    있으면 blocked. 원 전체를 본 경우에만 clear다: 원이 격자의 변 밖으로 나가거나(격자 좌표로
    잰다, 격자가 돌아가 있어도 같다) 원에 걸친 칸(중심이 원 밖이어도) 가운데 모르는 칸(-1)이
    하나라도 있으면 unknown.
    """
    if age_s > MAX_AGE_S or grid.cost_at(x, y) is None:
        return UNKNOWN
    reach = max(r, grid.resolution)
    if any(cost >= LETHAL for _, _, cost in _cells_near(grid, x, y, reach)):
        return BLOCKED
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    dx, dy = x - grid.origin_x, y - grid.origin_y
    gx, gy = c * dx + s * dy, -s * dx + c * dy  # 격자 안 좌표(m)
    if min(gx, gy, grid.width * grid.resolution - gx, grid.height * grid.resolution - gy) < reach:
        return UNKNOWN
    res = grid.resolution
    span = int(math.ceil(reach / res)) + 1
    ci, cj = math.floor(gx / res), math.floor(gy / res)
    for j in range(max(0, cj - span), min(grid.height, cj + span + 1)):
        for i in range(max(0, ci - span), min(grid.width, ci + span + 1)):
            # 칸 사각형에서 원 중심에 가장 가까운 점까지가 reach 이하면 원에 걸친 칸이다
            dx = max(i * res - gx, 0.0, gx - (i + 1) * res)
            dy = max(j * res - gy, 0.0, gy - (j + 1) * res)
            if math.hypot(dx, dy) <= reach + TOUCH_M and grid.data[j * grid.width + i] < 0:
                return UNKNOWN
    return CLEAR
