from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from pinky_fsm.geometry import Point, Polyline
from pinky_fsm.nav_goal import MODE_NAV_GOAL
from pinky_fsm.order_queue import Seg


@dataclass(frozen=True)
class Line:
    seq_id: int
    points: tuple[Point, ...]
    horizon: bool


def order_lines(start: Point | None, base: Sequence[Seg], horizon: Sequence[Seg]) -> list[Line]:
    out: list[Line] = []
    prev = start
    for seg, is_horizon in [(s, False) for s in base] + [(s, True) for s in horizon]:
        if seg.mode == MODE_NAV_GOAL:
            if seg.target is None:
                continue
            end = (float(seg.target[0]), float(seg.target[1]))
            points = (prev, end) if prev is not None else (end,)
        else:
            points = tuple((float(p[0]), float(p[1])) for p in seg.path)
        if points:
            out.append(Line(seg.seq_id, points, is_horizon))
            prev = points[-1]
    return out


def dashes(points: Sequence[Point], dash_m: float = 0.06,
           gap_m: float = 0.04) -> list[tuple[Point, Point]]:
    if len(points) < 2:
        return []
    line = Polyline(points)
    period = dash_m + gap_m
    out = []
    for k in range(math.ceil(line.length / period - 1e-9)):
        s = k * period
        out.append((line.point_at(s), line.point_at(min(s + dash_m, line.length))))
    return out
