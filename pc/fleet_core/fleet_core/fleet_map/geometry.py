# ---- 사각형, 점, 선분 기하 계산 ----
from __future__ import annotations

import math

# ---- 타입 별칭과 사각형 판정 ----
Rect = tuple[float, float, float, float]
Point = tuple[float, float]
EPS = 1e-9


def inside(rect: Rect, x: float, y: float, eps: float = EPS) -> bool:
    return rect[0] - eps <= x <= rect[2] + eps and rect[1] - eps <= y <= rect[3] + eps


def within(inner: Rect, outer: Rect, eps: float = EPS) -> bool:
    return (outer[0] - eps <= inner[0] and outer[1] - eps <= inner[1]
            and inner[2] <= outer[2] + eps and inner[3] <= outer[3] + eps)


def overlap_area(a: Rect, b: Rect) -> float:
    width = min(a[2], b[2]) - max(a[0], b[0])
    height = min(a[3], b[3]) - max(a[1], b[1])
    return width * height if width > EPS and height > EPS else 0.0


# ---- 점과 선분의 거리 계산 ----
def point_rect_distance(rect: Rect, x: float, y: float) -> float:
    dx = max(rect[0] - x, 0.0, x - rect[2])
    dy = max(rect[1] - y, 0.0, y - rect[3])
    return math.hypot(dx, dy)


def point_segment_distance(point: Point, a: Point, b: Point) -> float:
    ux, uy = b[0] - a[0], b[1] - a[1]
    span = ux * ux + uy * uy
    if span <= EPS:
        return math.hypot(point[0] - a[0], point[1] - a[1])
    t = max(0.0, min(1.0, ((point[0] - a[0]) * ux + (point[1] - a[1]) * uy) / span))
    return math.hypot(point[0] - a[0] - t * ux, point[1] - a[1] - t * uy)


def _cross(a: Point, b: Point, c: Point, d: Point) -> bool:
    def side(p: Point, q: Point, r: Point) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    d1, d2, d3, d4 = side(c, d, a), side(c, d, b), side(a, b, c), side(a, b, d)
    if ((d1 > EPS and d2 < -EPS) or (d1 < -EPS and d2 > EPS)) and (
            (d3 > EPS and d4 < -EPS) or (d3 < -EPS and d4 > EPS)):
        return True
    touching = ((d1, a, c, d), (d2, b, c, d), (d3, c, a, b), (d4, d, a, b))
    return any(abs(turn) <= EPS and point_segment_distance(point, p, q) <= EPS
               for turn, point, p, q in touching)


# ---- 선분과 사각형, 벽 사이 거리와 표본 추출 ----
def segment_rect_distance(a: Point, b: Point, rect: Rect) -> float:
    if inside(rect, *a) or inside(rect, *b):
        return 0.0
    corners = [(rect[0], rect[1]), (rect[2], rect[1]), (rect[2], rect[3]), (rect[0], rect[3])]
    if any(_cross(a, b, p, q) for p, q in zip(corners, corners[1:] + corners[:1], strict=True)):
        return 0.0
    ends = min(point_rect_distance(rect, *a), point_rect_distance(rect, *b))
    return min(ends, min(point_segment_distance(corner, a, b) for corner in corners))


def wall_gap(bounds: Rect, x: float, y: float) -> float:
    return min(x - bounds[0], bounds[2] - x, y - bounds[1], bounds[3] - y)


def sample(a: Point, b: Point, step: float) -> list[Point]:
    count = max(1, math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    return [(a[0] + (b[0] - a[0]) * i / count, a[1] + (b[1] - a[1]) * i / count)
            for i in range(count + 1)]
