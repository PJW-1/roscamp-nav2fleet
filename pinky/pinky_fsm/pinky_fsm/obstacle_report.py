from __future__ import annotations

import math

from pinky_fsm.costmap import Grid

LETHAL = 100
AHEAD_HALF_WIDTH_M = 0.12
CLEAR, BLOCKED, UNKNOWN = "clear", "blocked", "unknown"
MAX_AGE_S = 2.0
TOUCH_M = 1e-6


def _cells_near(grid: Grid, x: float, y: float, reach: float):
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    dx, dy = x - grid.origin_x, y - grid.origin_y
    gx, gy = c * dx + s * dy, -s * dx + c * dy
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
    if age_s > MAX_AGE_S or grid.cost_at(x, y) is None:
        return UNKNOWN
    reach = max(r, grid.resolution)
    if any(cost >= LETHAL for _, _, cost in _cells_near(grid, x, y, reach)):
        return BLOCKED
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    dx, dy = x - grid.origin_x, y - grid.origin_y
    gx, gy = c * dx + s * dy, -s * dx + c * dy
    if min(gx, gy, grid.width * grid.resolution - gx, grid.height * grid.resolution - gy) < reach:
        return UNKNOWN
    res = grid.resolution
    span = int(math.ceil(reach / res)) + 1
    ci, cj = math.floor(gx / res), math.floor(gy / res)
    for j in range(max(0, cj - span), min(grid.height, cj + span + 1)):
        for i in range(max(0, ci - span), min(grid.width, ci + span + 1)):
            dx = max(i * res - gx, 0.0, gx - (i + 1) * res)
            dy = max(j * res - gy, 0.0, gy - (j + 1) * res)
            if math.hypot(dx, dy) <= reach + TOUCH_M and grid.data[j * grid.width + i] < 0:
                return UNKNOWN
    return CLEAR
