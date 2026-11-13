from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

INSCRIBED = 99


@dataclass(frozen=True)
class Grid:
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    width: int
    height: int
    data: Sequence[int]

    def cost_at(self, x: float, y: float) -> int | None:
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(-self.origin_yaw), math.sin(-self.origin_yaw)
        gx, gy = c * dx - s * dy, s * dx + c * dy
        i, j = math.floor(gx / self.resolution), math.floor(gy / self.resolution)
        if not (0 <= i < self.width and 0 <= j < self.height):
            return None
        return int(self.data[j * self.width + i])


def to_frame(points: Sequence[Sequence[float]],
             tf: tuple[float, float, float]) -> list[tuple[float, float]]:
    tx, ty, yaw = tf
    c, s = math.cos(yaw), math.sin(yaw)
    return [(tx + c * px - s * py, ty + s * px + c * py) for px, py in points]


def obstacle_on(grid: Grid, points: Sequence[Sequence[float]],
                threshold: int = INSCRIBED) -> bool:
    for x, y in points:
        cost = grid.cost_at(x, y)
        if cost is not None and cost >= threshold:
            return True
    return False
