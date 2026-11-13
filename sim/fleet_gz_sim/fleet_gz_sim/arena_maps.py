"""경기장의 Nav2 지도(PGM, YAML)와 global costmap keepout 마스크. ROS를 쓰지 않는다.

지도는 벽 안쪽에서 MAP_PAD_M씩 넓다(벽 0.05 m가 들어간다). 막힌 칸은 0(검정), 빈칸은 254다.
상자가 칸에 조금이라도 걸치면 막힌 칸이다. 칸보다 얇은 상자(1.5 cm 진열대, 2 cm 칸막이)는 그 축에서
가운데가 든 칸 하나만 칠한다: 사라지지 않고, 칸 경계에 걸쳐 두 칸으로 두꺼워지지도 않는다.

keepout(지시서 3절, pinky_nav_config/params/keepout.yaml): shared가 아닌 구역(급행 lane, 갈림목과
도크 dock)을 막아 Nav2 goal 경로(NavFn)가 그 구역으로 들어가지 않게 한다. keepout 필터는 inflation
뒤에 얹혀 부풀지 않으므로 구역을 몸 반폭(KEEPOUT_PAD_M)만큼 넓혀 막는다: 경로가 구역 경계에 붙어
지나가도 몸이 구역에 걸치지 않는다. shared 구역의 지점(대기 자리, 주차, 갈림 지점)은 늘 빈칸이다
(test_arena가 본다).
"""
from __future__ import annotations

import math

from fleet_core.fleet_map.keepout import (
    BODY_HALF_WIDTH_M,
    filled,
    keepout_rects,  # noqa: F401  test_arena와 arena.py의 설명이 이 이름으로 쓴다
    mask,
)
from fleet_core.fleet_map.keepout import pgm as _pgm

from fleet_gz_sim.arena import obstacles

COARSE_M = 0.05  # costmap용 지도
FINE_M = 0.01  # AMCL용 지도(R15)와 keepout 마스크
MAP_PAD_M = 0.15
KEEPOUT_PAD_M = BODY_HALF_WIDTH_M  # Nav2 발자국의 반 폭(pinky_nav_config footprint 0.06)


def map_origin(site) -> tuple[float, float]:
    x0, y0, _, _ = site.bounds()
    return round(x0 - MAP_PAD_M, 4), round(y0 - MAP_PAD_M, 4)


def map_cells(resolution: float, site) -> tuple[int, int]:
    """(가로 칸 수, 세로 칸 수). 벽 바깥 MAP_PAD_M까지 덮도록 올림한다."""
    x0, y0, x1, y1 = site.bounds()
    return (math.ceil(round((x1 - x0 + 2 * MAP_PAD_M) / resolution, 6)),
            math.ceil(round((y1 - y0 + 2 * MAP_PAD_M) / resolution, 6)))


def occupancy(resolution: float, site) -> list[list[bool]]:
    """벽·진열대가 걸친 칸은 참. 줄은 아래(y 작은 쪽)부터."""
    nx, ny = map_cells(resolution, site)
    ox, oy = map_origin(site)

    def thin(lo: float, hi: float, start: float) -> tuple[float, float]:
        # 칸보다 얇으면 가운데가 든 칸 하나만: 칸 경계에 걸쳐 두 칸으로 두꺼워지지 않게
        if hi - lo >= resolution - 1e-9:
            return lo, hi
        cell = math.floor(((lo + hi) / 2 - start) / resolution)
        return start + (cell + 0.25) * resolution, start + (cell + 0.75) * resolution
    rects = []
    for b in obstacles(site):
        (x0, x1), (y0, y1) = thin(b.x0, b.x1, ox), thin(b.y0, b.y1, oy)
        rects.append((x0, y0, x1, y1))
    return filled(rects, nx, ny, resolution, (ox, oy))


def keepout(resolution: float, site) -> list[list[bool]]:
    """keepout 칸은 참. 줄은 아래부터."""
    nx, ny = map_cells(resolution, site)
    return mask(site, nx, ny, resolution, map_origin(site))


def cell_of(x: float, y: float, resolution: float, site) -> tuple[int, int]:
    """(i, j): occupancy/keepout 격자에서 점이 든 칸."""
    ox, oy = map_origin(site)
    return math.floor((x - ox) / resolution), math.floor((y - oy) / resolution)


def pgm(resolution: float, site) -> bytes:
    return _pgm(occupancy(resolution, site))


def keepout_pgm(resolution: float, site) -> bytes:
    return _pgm(keepout(resolution, site))


def map_yaml(name: str, resolution: float, site) -> str:
    """map_server 형식. 마스크도 같은 형식이고 막힌 칸(100)이 keepout이다."""
    ox, oy = map_origin(site)
    layout = site.name.removeprefix("arena_")
    return (f"# 생성 파일: python3 -m fleet_gz_sim.arena --layout {layout}\n"
            f"image: {name}.pgm\nmode: trinary\nresolution: {resolution}\n"
            f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
