"""경기장의 Nav2 지도(PGM, YAML)와 global costmap keepout 마스크. ROS를 쓰지 않는다.
막힌 칸은 0(검정), 빈칸은 254. keepout은 shared가 아닌 구역을 몸 반폭만큼 넓혀 막는다.
갈래 a1은 a와 같은 칠하기 규칙의 1 cm costmap 지도다.
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
NAV_VARIANTS = {"a": (FINER_M, TOUCH), "b": (COARSE_M, CENTRE), "ab": (FINER_M, CENTRE),
                "a1": (FINE_M, TOUCH)}


def map_origin(site) -> tuple[float, float]:
    x0, y0, _, _ = site.bounds()
    return round(x0 - MAP_PAD_M, 4), round(y0 - MAP_PAD_M, 4)


def map_cells(resolution: float, site) -> tuple[int, int]:
    """(가로 칸 수, 세로 칸 수). 벽 바깥 MAP_PAD_M까지 덮도록 올림한다."""
    x0, y0, x1, y1 = site.bounds()
    return (math.ceil(round((x1 - x0 + 2 * MAP_PAD_M) / resolution, 6)),
            math.ceil(round((y1 - y0 + 2 * MAP_PAD_M) / resolution, 6)))


def centre_filled(rects, width: int, height: int, resolution: float, origin) -> list[list[bool]]:
    """칸 중심이 사각형 안(경계 포함)이면 참. 얇은 상자는 축마다 가운데 칸 하나를 칠한다."""
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


REAL_EXAMPLE_OUT = "/tmp/real_v7"  # 실물 지도 머리 줄의 예시 --out(패키지 밖)


def map_yaml(name: str, resolution: float, site, note: str = "", real: bool = False) -> str:
    """map_server 형식으로 쓴다. 마스크도 같은 형식이고 막힌 칸(100)이 keepout이다(real: 실물 키트 지도)."""
    ox, oy = map_origin(site)
    layout = site.name.removeprefix("arena_")
    head = f"# 생성 파일: python3 -m fleet_gz_sim.arena --layout {layout}"
    if real:
        # 셸에 그대로 쳐도 도는 명령 + 복사 안내는 따로 한 줄
        head += (f" --real --out {REAL_EXAMPLE_OUT}\n"
                 f"# 실물 키트: {REAL_EXAMPLE_OUT}/maps/의 지도를 tools/real_pode/maps로 복사한다")
    head += "\n"
    if note:
        head += f"# {note}\n"
    return (head + f"image: {name}.pgm\nmode: trinary\nresolution: {resolution}\n"
            f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
