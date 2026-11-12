"""keepout 마스크: 넓은 구역이 아닌 구역(급행, 도크, 갈림목)을 Nav2 goal 경로에서 막는 그림.

Nav2의 keepout 필터는 map_server 형식(yaml과 그림)의 마스크를 읽고, 막힌 칸이 keepout이다.
필터는 벽 여유(inflation) 뒤에 얹혀 둘레가 부풀지 않으므로 몸 반폭만큼 넓혀 만든다. 반폭은 곧게
선 몸의 것이다: 45도로 선 몸의 모서리는 0.085 m까지 나간다.
어떤 지도든 그 지도의 크기, 원점, 칸 크기에 맞춰 만든다. ROS를 쓰지 않는다.
"""
from __future__ import annotations

import math

from .model import SHARED

BODY_HALF_WIDTH_M = 0.06  # Nav2 발자국의 반 폭(pinky_nav_config footprint)
HAIR = 1e-6  # 칸 단위: 경계에 꼭 맞는 사각형이 옆 칸을 채우지 않게


def keepout_rects(site, pad: float = BODY_HALF_WIDTH_M) -> list[tuple]:
    """넓은 구역이 아닌 구역의 사각형 (x0, y0, x1, y1)을 pad만큼 넓힌 것. 구역 이름 순서."""
    return [(x0 - pad, y0 - pad, x1 + pad, y1 + pad)
            for zone in sorted(site.zones.values(), key=lambda z: z.id) if zone.kind != SHARED
            for x0, y0, x1, y1 in zone.rects]


def filled(rects, width: int, height: int, resolution: float, origin) -> list[list[bool]]:
    """사각형 (x0, y0, x1, y1)이 조금이라도 걸친 칸이 참인 격자.

    줄은 아래(y 작은 쪽)부터이고, origin은 왼쪽 아래 구석이다. 격자는 축에 맞춘 것이다(지도
    origin의 yaw는 0이어야 한다).
    """
    grid = [[False] * width for _ in range(height)]

    def cells(lo: float, hi: float, start: float, count: int) -> range:
        return range(max(0, math.floor((lo - start) / resolution + HAIR)),
                     min(count, math.ceil((hi - start) / resolution - HAIR)))

    for x0, y0, x1, y1 in rects:
        for j in cells(y0, y1, origin[1], height):
            for i in cells(x0, x1, origin[0], width):
                grid[j][i] = True
    return grid


def mask(site, width: int, height: int, resolution: float, origin,
         pad: float = BODY_HALF_WIDTH_M) -> list[list[bool]]:
    """keepout 칸이 참인 격자(filled와 같은 꼴)."""
    return filled(keepout_rects(site, pad), width, height, resolution, origin)


def pgm(grid) -> bytes:
    """격자를 PGM(P5)으로. 첫 줄이 지도 위쪽이고 막힌 칸은 0(검정), 빈칸은 254다."""
    header = f"P5\n{len(grid[0])} {len(grid)}\n255\n".encode()
    return header + b"".join(bytes(0 if closed else 254 for closed in row)
                             for row in reversed(grid))


def pgm_size(data: bytes) -> tuple[int, int]:
    """PGM(P5) 머리말에서 (가로, 세로) 칸 수를 읽는다. 주석 줄(#)은 건너뛴다."""
    words = []
    for line in data[:512].split(b"\n"):
        words += line.split(b"#")[0].split()
        if len(words) >= 3:
            break
    if len(words) < 3 or words[0] != b"P5":
        raise ValueError("not a binary PGM (P5) picture")
    return int(words[1]), int(words[2])


def mask_yaml(image: str, resolution: float, origin) -> str:
    """마스크의 map_server yaml 본문."""
    return (f"image: {image}\nmode: trinary\nresolution: {resolution}\n"
            f"origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.25\n")
