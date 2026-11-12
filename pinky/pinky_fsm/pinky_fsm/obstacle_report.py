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


def clear_answer(grid: Grid, x: float, y: float, r: float) -> str:
    """CheckClear의 답. 그 점이 격자 밖이거나 r 안의 칸을 모두 모르면(-1) unknown, 장애물 칸이
    있으면 blocked, 없으면 clear."""
    if grid.cost_at(x, y) is None:
        return UNKNOWN
    costs = [cost for _, _, cost in _cells_near(grid, x, y, max(r, grid.resolution))]
    if any(cost >= LETHAL for cost in costs):
        return BLOCKED
    if not costs or all(cost < 0 for cost in costs):
        return UNKNOWN
    return CLEAR
