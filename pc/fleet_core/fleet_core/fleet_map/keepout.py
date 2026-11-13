from __future__ import annotations

import math

from .model import SHARED

BODY_HALF_WIDTH_M = 0.06
HAIR = 1e-6


def keepout_rects(site, pad: float = BODY_HALF_WIDTH_M) -> list[tuple]:
    return [(x0 - pad, y0 - pad, x1 + pad, y1 + pad)
            for zone in sorted(site.zones.values(), key=lambda z: z.id) if zone.kind != SHARED
            for x0, y0, x1, y1 in zone.rects]


def filled(rects, width: int, height: int, resolution: float, origin) -> list[list[bool]]:
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
    return filled(keepout_rects(site, pad), width, height, resolution, origin)


def pgm(grid) -> bytes:
    header = f"P5\n{len(grid[0])} {len(grid)}\n255\n".encode()
    return header + b"".join(bytes(0 if closed else 254 for closed in row)
                             for row in reversed(grid))


def pgm_size(data: bytes) -> tuple[int, int]:
    words = []
    for line in data[:512].split(b"\n"):
        words += line.split(b"#")[0].split()
        if len(words) >= 3:
            break
    if len(words) < 3 or words[0] != b"P5":
        raise ValueError("not a binary PGM (P5) picture")
    return int(words[1]), int(words[2])


def mask_yaml(image: str, resolution: float, origin) -> str:
    return (f"image: {image}\nmode: trinary\nresolution: {resolution}\n"
            f"origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.25\n")
