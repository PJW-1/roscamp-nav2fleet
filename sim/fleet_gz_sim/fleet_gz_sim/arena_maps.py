"""경기장의 Nav2 지도(PGM, YAML)와 global costmap keepout 마스크. ROS를 쓰지 않는다.

지도는 벽 안쪽에서 MAP_PAD_M씩 넓다(벽 0.05 m가 들어간다). 막힌 칸은 0(검정), 빈칸은 254다.
상자가 칸에 조금이라도 걸치면 막힌 칸이라 1.5 cm 진열대도 5 cm 지도에서 사라지지 않는다.

keepout(지시서 3절, pinky_nav_config/params/keepout.yaml): shared가 아닌 구역(급행 lane, 갈림목과
도크 dock)을 막아 Nav2 goal 경로(NavFn)가 그 구역으로 들어가지 않게 한다. keepout 필터는 inflation
뒤에 얹혀 부풀지 않으므로 구역을 몸 반폭(KEEPOUT_PAD_M)만큼 넓혀 막는다: 경로가 구역 경계에 붙어
지나가도 몸이 구역에 걸치지 않는다. shared 구역의 지점(대기 자리, 주차, 갈림 지점)은 늘 빈칸이다
(test_arena가 본다).
"""
from __future__ import annotations

import math

from fleet_core.fleet_map import SHARED

from fleet_gz_sim.arena import obstacles

COARSE_M = 0.05  # costmap용 지도
FINE_M = 0.01  # AMCL용 지도(R15)와 keepout 마스크
MAP_PAD_M = 0.15
KEEPOUT_PAD_M = 0.06  # Nav2 발자국의 반 폭(pinky_nav_config footprint 0.06)
HAIR = 1e-6  # 칸 단위: 경계에 꼭 맞는 상자가 옆 칸을 채우지 않게


def map_origin(site) -> tuple[float, float]:
    x0, y0, _, _ = site.bounds()
    return round(x0 - MAP_PAD_M, 4), round(y0 - MAP_PAD_M, 4)


def map_cells(resolution: float, site) -> tuple[int, int]:
    """(가로 칸 수, 세로 칸 수). 벽 바깥 MAP_PAD_M까지 덮도록 올림한다."""
    x0, y0, x1, y1 = site.bounds()
    return (math.ceil(round((x1 - x0 + 2 * MAP_PAD_M) / resolution, 6)),
            math.ceil(round((y1 - y0 + 2 * MAP_PAD_M) / resolution, 6)))


def _fill(grid, rects, resolution: float, site) -> None:
    ny, nx = len(grid), len(grid[0])
    ox, oy = map_origin(site)

    def cells(lo: float, hi: float, origin: float, n: int) -> range:
        return range(max(0, math.floor((lo - origin) / resolution + HAIR)),
                     min(n, math.ceil((hi - origin) / resolution - HAIR)))

    for x0, y0, x1, y1 in rects:
        for j in cells(y0, y1, oy, ny):
            for i in cells(x0, x1, ox, nx):
                grid[j][i] = True


def occupancy(resolution: float, site) -> list[list[bool]]:
    """벽·진열대가 걸친 칸은 참. 줄은 아래(y 작은 쪽)부터."""
    nx, ny = map_cells(resolution, site)
    grid = [[False] * nx for _ in range(ny)]
    _fill(grid, [(b.x0, b.y0, b.x1, b.y1) for b in obstacles(site)], resolution, site)
    return grid


def keepout_rects(site, pad: float = KEEPOUT_PAD_M) -> list[tuple[float, float, float, float]]:
    """shared가 아닌 구역의 사각형을 pad만큼 넓힌 것. 구역 이름 순서."""
    return [(x0 - pad, y0 - pad, x1 + pad, y1 + pad)
            for zone in sorted(site.zones.values(), key=lambda z: z.id) if zone.kind != SHARED
            for x0, y0, x1, y1 in zone.rects]


def keepout(resolution: float, site) -> list[list[bool]]:
    """keepout 칸은 참. 줄은 아래부터."""
    nx, ny = map_cells(resolution, site)
    grid = [[False] * nx for _ in range(ny)]
    _fill(grid, keepout_rects(site), resolution, site)
    return grid


def cell_of(x: float, y: float, resolution: float, site) -> tuple[int, int]:
    """(i, j): occupancy/keepout 격자에서 점이 든 칸."""
    ox, oy = map_origin(site)
    return math.floor((x - ox) / resolution), math.floor((y - oy) / resolution)


def _pgm(grid) -> bytes:
    header = f"P5\n{len(grid[0])} {len(grid)}\n255\n".encode()
    # PGM 첫 줄은 지도 위쪽(y 큰 쪽). 막힌 칸 0(검정), 빈칸 254
    return header + b"".join(bytes(0 if c else 254 for c in row) for row in reversed(grid))


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
