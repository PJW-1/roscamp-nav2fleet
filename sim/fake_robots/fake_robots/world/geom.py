"""꺾은선 계산: 길이, 점과 선의 거리, 앞쪽 일부. ROS를 쓰지 않는다."""
from __future__ import annotations

import math

Point = tuple[float, float]


def segment_gap(p: Point, a: Point, b: Point) -> float:
    vx, vy = b[0] - a[0], b[1] - a[1]
    span = vx * vx + vy * vy
    t = max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / span)) if span else 0.0
    return math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)


def path_gap(p: Point, points) -> float:
    """점에서 꺾은선까지. 점이 하나면 그 점까지."""
    if len(points) == 1:
        return math.dist(p, points[0])
    return min(segment_gap(p, a, b) for a, b in zip(points, points[1:], strict=False))


def length(points) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))


def ahead(points, distance: float) -> list[Point]:
    """꺾은선의 앞쪽 distance만큼(첫 점부터)."""
    out, left = [points[0]], distance
    for a, b in zip(points, points[1:], strict=False):
        span = math.dist(a, b)
        if span >= left:
            f = left / span if span else 0.0
            out.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
            return out
        out.append(b)
        left -= span
    return out


def advance(points: list[Point], step: float) -> tuple[Point, list[Point], float]:
    """points[0]에서 꺾은선을 따라 step만큼 간 자리, 남은 꺾은선(그 자리부터), 실제로 간 거리."""
    here, rest, moved = points[0], list(points[1:]), 0.0
    while rest and step > 0.0:
        gap = math.dist(here, rest[0])
        if gap <= step:
            here, step, moved = rest.pop(0), step - gap, moved + gap
            continue
        f = step / gap
        here = (here[0] + (rest[0][0] - here[0]) * f, here[1] + (rest[0][1] - here[1]) * f)
        moved += step
        step = 0.0
    return here, [here, *rest], moved


def heading(a: Point, b: Point, default: float = 0.0) -> float:
    if math.dist(a, b) <= 1e-12:
        return default
    return math.atan2(b[1] - a[1], b[0] - a[0])


def angle_diff(a: float, b: float) -> float:
    """a에서 b로 도는 가장 작은 각(-pi..pi)."""
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def nearest_index(p: Point, points) -> int:
    """p에 가장 가까운 선분의 번호(같으면 뒤 선분). 그 선분 앞의 꼭짓점은 지난 것이다."""
    gaps = [segment_gap(p, a, b) for a, b in zip(points, points[1:], strict=False)]
    best = min(gaps)
    return max(i for i, gap in enumerate(gaps) if gap <= best + 1e-9)
