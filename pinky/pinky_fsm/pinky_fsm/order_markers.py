"""order 표시용 선(order 추가지시서 3절 7번, 계약 0.11-draft 제안). ROS를 모른다.

노드가 /<robot>/order_markers(MarkerArray)로 옮긴다. base는 실선, horizon은 점선이다. nav_goal
조각은 길을 Nav2가 짜므로 앞 조각의 끝(처음이면 로봇)에서 목표까지 곧은 선으로 그린다.
"""
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
    """점선의 조각들(LINE_LIST의 점 쌍)."""
    if len(points) < 2:
        return []
    line = Polyline(points)
    period = dash_m + gap_m
    out = []
    for k in range(math.ceil(line.length / period - 1e-9)):
        s = k * period
        out.append((line.point_at(s), line.point_at(min(s + dash_m, line.length))))
    return out
