"""Nav2 goal이 다닐 수 있는 바닥(격자). ROS를 쓰지 않는다.

넓은 구역(shared)의 합만 바닥이다. 한 줄 구역과 도크 구역은 keepout처럼 막고, 벽과 진열대와
keepout 구역은 로봇 반지름만큼 키운다(지시서 3절 keepout, 6-1). 다른 로봇과 물체는 매번
원으로 찍는다(로봇 반지름 + 그 반지름 + 여유).
"""
from __future__ import annotations

import math

from fleet_core.fleet_map import SHARED
from fleet_core.fleet_map.geometry import inside, point_rect_distance, wall_gap

GRID_M = 0.02


class FreeSpace:
    def __init__(self, site, radius_m: float, resolution: float = GRID_M):
        self.res = resolution
        self.radius = radius_m
        x0, y0, x1, y1 = site.bounds()
        self.x0, self.y0 = x0, y0
        self.w = max(1, round((x1 - x0) / resolution))
        self.h = max(1, round((y1 - y0) / resolution))
        shared = [rect for z in site.zones.values() if z.kind == SHARED for rect in z.rects]
        keepout = [rect for z in site.zones.values() if z.kind != SHARED for rect in z.rects]
        blocked = list(site.shelves) + keepout
        bounds = site.bounds()
        cells = bytearray(self.w * self.h)
        for iy in range(self.h):
            for ix in range(self.w):
                x, y = self.centre(ix, iy)
                ok = (any(inside(rect, x, y) for rect in shared)
                      and wall_gap(bounds, x, y) >= radius_m
                      and all(point_rect_distance(rect, x, y) >= radius_m for rect in blocked))
                cells[iy * self.w + ix] = 0 if ok else 1
        self.static = bytes(cells)

    def centre(self, ix: int, iy: int) -> tuple[float, float]:
        return (self.x0 + (ix + 0.5) * self.res, self.y0 + (iy + 0.5) * self.res)

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return (min(self.w - 1, max(0, int((x - self.x0) / self.res))),
                min(self.h - 1, max(0, int((y - self.y0) / self.res))))

    def with_discs(self, discs) -> bytearray:
        """정적 바닥에 원 (x, y, 막는 반지름)들을 찍은 격자."""
        cells = bytearray(self.static)
        for cx, cy, reach in discs:
            ix0, iy0 = self.cell(cx - reach, cy - reach)
            ix1, iy1 = self.cell(cx + reach, cy + reach)
            for iy in range(iy0, iy1 + 1):
                for ix in range(ix0, ix1 + 1):
                    x, y = self.centre(ix, iy)
                    if math.hypot(x - cx, y - cy) < reach:
                        cells[iy * self.w + ix] = 1
        return cells

    def free(self, cells, x: float, y: float) -> bool:
        ix, iy = self.cell(x, y)
        return cells[iy * self.w + ix] == 0

    def line_free(self, cells, a, b) -> bool:
        """a에서 b까지 곧은 선이 빈 칸만 지난다(반 칸 간격으로 본다)."""
        count = max(1, math.ceil(math.dist(a, b) / (self.res * 0.5)))
        return all(self.free(cells, a[0] + (b[0] - a[0]) * i / count,
                             a[1] + (b[1] - a[1]) * i / count) for i in range(count + 1))

    def nearest_free(self, cells, x: float, y: float, within: float):
        """within 안에서 가장 가까운 빈 칸의 중심. 없으면 None."""
        ix, iy = self.cell(x, y)
        reach = math.ceil(within / self.res)
        best, best_gap = None, math.inf
        for jy in range(max(0, iy - reach), min(self.h, iy + reach + 1)):
            for jx in range(max(0, ix - reach), min(self.w, ix + reach + 1)):
                if cells[jy * self.w + jx]:
                    continue
                c = self.centre(jx, jy)
                gap = math.dist(c, (x, y))
                if gap <= within and gap < best_gap:
                    best, best_gap = c, gap
        return best
