"""Nav2 planner 흉내: 격자 위 A*(8방향) 뒤에 보이는 점끼리 이어 꺾은선을 줄인다.

ROS를 쓰지 않는다.
"""
from __future__ import annotations

import heapq
import math

START_SNAP_M = 0.12  # 출발 칸이 막혔으면(잡음, 벽 가까이) 이 안의 빈 칸에서 시작한다
GOAL_SNAP_M = 0.05  # 목표 칸이 막혔으면 이 안의 빈 칸까지(Nav2 planner tolerance)
_STEPS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
          (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
          (-1, -1, math.sqrt(2))]


def plan(space, cells, start, goal):
    """start에서 goal까지의 꺾은선, 길이 없으면 None. 첫 점은 start, 끝 점은 goal이나
    (goal 칸이 막혔으면) GOAL_SNAP_M 안의 빈 칸이다."""
    entry = start if space.free(cells, *start) else space.nearest_free(cells, *start,
                                                                       START_SNAP_M)
    if entry is None:
        return None
    end = goal if space.free(cells, *goal) else space.nearest_free(cells, *goal, GOAL_SNAP_M)
    if end is None:
        return None
    if space.line_free(cells, entry, end):
        return _dedupe([start, entry, end])
    cells_path = _astar(space, cells, space.cell(*entry), space.cell(*end))
    if cells_path is None:
        return None
    points = [entry, *(space.centre(ix, iy) for ix, iy in cells_path[1:-1]), end]
    return _dedupe([start, *_shortcut(space, cells, points)])


def _astar(space, cells, a, b):
    w = space.w
    if cells[a[1] * w + a[0]] or cells[b[1] * w + b[0]]:
        return None

    def guess(ix, iy):
        dx, dy = abs(ix - b[0]), abs(iy - b[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    best = {a: 0.0}
    came = {}
    heap = [(guess(*a), 0.0, a)]
    while heap:
        _, cost, here = heapq.heappop(heap)
        if here == b:
            out = [here]
            while here in came:
                here = came[here]
                out.append(here)
            return out[::-1]
        if cost > best.get(here, math.inf):
            continue
        for dx, dy, step in _STEPS:
            nx, ny = here[0] + dx, here[1] + dy
            if not (0 <= nx < w and 0 <= ny < space.h) or cells[ny * w + nx]:
                continue
            if dx and dy and (cells[here[1] * w + nx] or cells[ny * w + here[0]]):
                continue  # 모서리를 깎지 않는다
            new = cost + step
            if new < best.get((nx, ny), math.inf):
                best[(nx, ny)] = new
                came[(nx, ny)] = here
                heapq.heappush(heap, (new + guess(nx, ny), new, (nx, ny)))
    return None


def _shortcut(space, cells, points):
    """앞 점에서 곧게 보이는 가장 먼 점으로 건너뛴다."""
    out, i = [points[0]], 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not space.line_free(cells, points[i], points[j]):
            j -= 1
        out.append(points[j])
        i = j
    return out


def _dedupe(points):
    out = [points[0]]
    for p in points[1:]:
        if math.dist(p, out[-1]) > 1e-9:
            out.append(p)
    return out
