"""경기장의 Nav2 지도(PGM, YAML)와 global costmap keepout 마스크. ROS를 쓰지 않는다.

지도는 벽 안쪽에서 MAP_PAD_M씩 넓다(벽 0.05 m가 들어간다). 막힌 칸은 0(검정), 빈칸은 254다.
상자가 칸에 조금이라도 걸치면 막힌 칸이라 1.5 cm 진열대도 5 cm 지도에서 사라지지 않는다(TOUCH, 기본).

가운데 통로 갈래(2026-11-17, Gazebo 비교용): 칸 중심이 상자 안일 때만 막는 규칙(CENTRE)과
2.5 cm 지도. NAV_VARIANTS의 지도는 v7에만 만든다(arena.generate_nav_variants). 기본 지도와
keepout 마스크, 1 cm 지도는 그대로 TOUCH다.

keepout(pode_nav_config/params/keepout.yaml): shared가 아닌 구역(급행 lane, 갈림목과
도크 dock)을 막아 Nav2 goal 경로(NavFn)가 그 구역으로 들어가지 않게 한다. keepout 필터는 inflation
뒤에 얹혀 부풀지 않으므로 구역을 몸 반폭(KEEPOUT_PAD_M)만큼 넓혀 막는다: 경로가 구역 경계에 붙어
지나가도 몸이 구역에 걸치지 않는다. shared 구역의 지점(대기 자리, 주차, 갈림 지점)은 늘 빈칸이다
(test_arena가 본다).
"""
from __future__ import annotations

import math

from fleet_core.fleet_map.keepout import (
    BODY_HALF_WIDTH_M,
    HAIR,
    filled,
    keepout_rects,  # noqa: F401  test_arena와 arena.py의 설명이 이 이름으로 쓴다
    mask,
)
from fleet_core.fleet_map.keepout import pgm as _pgm

from fleet_gz_sim.arena import obstacles

COARSE_M = 0.05  # costmap용 지도
FINE_M = 0.01  # AMCL용 지도(R15)와 keepout 마스크
MAP_PAD_M = 0.15
KEEPOUT_PAD_M = BODY_HALF_WIDTH_M  # Nav2 발자국의 반 폭(pode_nav_config footprint 0.06)
FINER_M = 0.025  # 갈래 A의 costmap용 지도
TOUCH = "touch"  # 상자가 걸친 칸은 막힘(기본)
CENTRE = "centre"  # 칸 중심이 상자 안(경계 포함)이면 막힘
PAINTS = (TOUCH, CENTRE)
# 갈래 이름: (해상도, 칠하기). 파일은 maps/<경기장>_<이름>.{yaml,pgm}
NAV_VARIANTS = {"a": (FINER_M, TOUCH), "b": (COARSE_M, CENTRE), "ab": (FINER_M, CENTRE)}


def map_origin(site) -> tuple[float, float]:
    x0, y0, _, _ = site.bounds()
    return round(x0 - MAP_PAD_M, 4), round(y0 - MAP_PAD_M, 4)


def map_cells(resolution: float, site) -> tuple[int, int]:
    """(가로 칸 수, 세로 칸 수). 벽 바깥 MAP_PAD_M까지 덮도록 올림한다."""
    x0, y0, x1, y1 = site.bounds()
    return (math.ceil(round((x1 - x0 + 2 * MAP_PAD_M) / resolution, 6)),
            math.ceil(round((y1 - y0 + 2 * MAP_PAD_M) / resolution, 6)))


def centre_filled(rects, width: int, height: int, resolution: float, origin) -> list[list[bool]]:
    """칸 중심이 사각형 안(경계 포함, 소수 오차로 HAIR 칸까지 허용)인 칸은 참. 한 축에서 중심을
    하나도 덮지 않는 얇은 상자는 그 축에서 상자 가운데가 든 칸 하나를 칠한다(2 cm 칸막이가 지도에서
    사라지지 않게). 상자마다 따로 칠하므로 격자에 어긋나게 맞닿은 상자 사이에는 대각 틈이 날 수
    있다(v7은 벽 밖 빈칸이 안쪽에 닿지 않고 칸막이가 벽과 이어짐만 test_arena_v7가 본다)."""
    grid = [[False] * width for _ in range(height)]

    def cells(lo: float, hi: float, start: float, count: int) -> range:
        first = math.ceil((lo - start) / resolution - 0.5 - HAIR)
        last = math.floor((hi - start) / resolution - 0.5 + HAIR)
        if first > last:
            first = last = math.floor(((lo + hi) / 2 - start) / resolution)
        return range(max(0, first), min(count, last + 1))

    for x0, y0, x1, y1 in rects:
        for j in cells(y0, y1, origin[1], height):
            for i in cells(x0, x1, origin[0], width):
                grid[j][i] = True
    return grid


def occupancy(resolution: float, site, paint: str = TOUCH) -> list[list[bool]]:
    """벽·진열대 칸은 참. 줄은 아래(y 작은 쪽)부터. paint: TOUCH(걸친 칸) 또는 CENTRE."""
    if paint not in PAINTS:
        raise ValueError(f"paint must be one of {PAINTS}, got {paint!r}")
    nx, ny = map_cells(resolution, site)
    paint_cells = filled if paint == TOUCH else centre_filled
    return paint_cells([(b.x0, b.y0, b.x1, b.y1) for b in obstacles(site)], nx, ny, resolution,
                       map_origin(site))


def keepout(resolution: float, site) -> list[list[bool]]:
    """keepout 칸은 참. 줄은 아래부터."""
    nx, ny = map_cells(resolution, site)
    return mask(site, nx, ny, resolution, map_origin(site))


def cell_of(x: float, y: float, resolution: float, site) -> tuple[int, int]:
    """(i, j): occupancy/keepout 격자에서 점이 든 칸."""
    ox, oy = map_origin(site)
    return math.floor((x - ox) / resolution), math.floor((y - oy) / resolution)


def pgm(resolution: float, site, paint: str = TOUCH) -> bytes:
    return _pgm(occupancy(resolution, site, paint))


def keepout_pgm(resolution: float, site) -> bytes:
    return _pgm(keepout(resolution, site))


def map_yaml(name: str, resolution: float, site, note: str = "") -> str:
    """map_server 형식. 마스크도 같은 형식이고 막힌 칸(100)이 keepout이다. note: 둘째 줄 설명."""
    ox, oy = map_origin(site)
    layout = site.name.removeprefix("arena_")
    head = f"# 생성 파일: python3 -m fleet_gz_sim.arena --layout {layout}\n"
    if note:
        head += f"# {note}\n"
    return (head + f"image: {name}.pgm\nmode: trinary\nresolution: {resolution}\n"
            f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
