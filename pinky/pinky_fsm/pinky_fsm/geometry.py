from __future__ import annotations

import math
from collections.abc import Sequence
from itertools import pairwise

Point = tuple[float, float]
Pose = tuple[float, float, float]

_SAME_M = 1e-6
ARC_STEP_M = 0.01


def _dedup(points: Sequence[Point]) -> list[Point]:
    out: list[Point] = []
    for p in points:
        p = (float(p[0]), float(p[1]))
        if not out or math.dist(out[-1], p) > _SAME_M:
            out.append(p)
    return out


def poses_along(points: Sequence[Point], final_yaw: float, step: float = 0.05) -> list[Pose]:
    pts = _dedup(points)
    if len(pts) == 1:
        return [(pts[0][0], pts[0][1], float(final_yaw))]
    poses: list[Pose] = []
    for (ax, ay), (bx, by) in pairwise(pts):
        length = math.hypot(bx - ax, by - ay)
        yaw = math.atan2(by - ay, bx - ax)
        n = max(1, math.ceil(length / step - 1e-9))
        for i in range(n):
            f = i / n
            poses.append((ax + f * (bx - ax), ay + f * (by - ay), yaw))
    last = pts[-1]
    poses.append((last[0], last[1], float(final_yaw)))
    return poses


def poses_backing(points: Sequence[Point], step: float = 0.05) -> list[Pose]:
    pts = _dedup(points)
    final = 0.0
    if len(pts) > 1:
        (ax, ay), (bx, by) = pts[-2], pts[-1]
        final = math.atan2(by - ay, bx - ax)
    return [(x, y, math.atan2(-math.sin(yaw), -math.cos(yaw)))
            for x, y, yaw in poses_along(pts, final, step)]


def _corner_arc(a: Point, b: Point, c: Point, room_in: float, room_out: float,
                radius: float, step: float) -> list[Point]:
    h1 = math.atan2(b[1] - a[1], b[0] - a[0])
    h2 = math.atan2(c[1] - b[1], c[0] - b[0])
    turn = math.remainder(h2 - h1, math.tau)
    if abs(turn) < 1e-3 or abs(turn) > math.pi - 1e-3:
        return [b]
    half = math.tan(abs(turn) / 2.0)
    cut = min(radius * half, room_in, room_out)
    r = cut / half
    start = (b[0] - cut * math.cos(h1), b[1] - cut * math.sin(h1))
    side = 1.0 if turn > 0 else -1.0
    centre = (start[0] - side * r * math.sin(h1), start[1] + side * r * math.cos(h1))
    a0 = math.atan2(start[1] - centre[1], start[0] - centre[0])
    n = max(2, math.ceil(r * abs(turn) / step))
    return [(centre[0] + r * math.cos(a0 + turn * i / n),
             centre[1] + r * math.sin(a0 + turn * i / n)) for i in range(n + 1)]


def rounded(points: Sequence[Point], radius: float, step: float = ARC_STEP_M) -> list[Point]:
    return _rounded_with_corners(points, radius, step)[0]


Corner = tuple[float, float, float]


def _rounded_with_corners(points: Sequence[Point], radius: float,
                          step: float) -> tuple[list[Point], list[Corner]]:
    pts = _dedup(points)
    if len(pts) < 3:
        return pts, []
    last = len(pts) - 2
    lengths = [math.dist(p, q) for p, q in pairwise(pts)]
    out, corners, run = [pts[0]], [], 0.0
    for i in range(1, len(pts) - 1):
        room_in = lengths[i - 1] if i - 1 == 0 else lengths[i - 1] / 2.0
        room_out = lengths[i] if i == last else lengths[i] / 2.0
        arc = [pts[i]]
        if radius > 0.0:
            arc = _corner_arc(pts[i - 1], pts[i], pts[i + 1], room_in, room_out, radius, step)
        start = run + math.dist(out[-1], arc[0])
        for p in arc:
            run += math.dist(out[-1], p)
            out.append(p)
        if len(arc) > 1 or not _straight(pts[i - 1], pts[i], pts[i + 1]):
            corners.append((start, run, math.dist(pts[i], arc[0])))
    out.append(pts[-1])
    return _dedup(out), corners


def _straight(a: Point, b: Point, c: Point) -> bool:
    turn = math.remainder(math.atan2(c[1] - b[1], c[0] - b[0])
                          - math.atan2(b[1] - a[1], b[0] - a[0]), math.tau)
    return abs(turn) < 1e-3


def _on_rounded(corners: Sequence[Corner], s: float) -> float:
    lost = 0.0
    for start, end, cut in corners:
        begin = start + lost
        if s <= begin:
            break
        if s < begin + 2.0 * cut:
            return start + (end - start) * (s - begin) / (2.0 * cut)
        lost += 2.0 * cut - (end - start)
    return s - lost


def rounded_at(route: Polyline, s: float, radius: float) -> float:
    _, corners = _rounded_with_corners(route.points, radius, ARC_STEP_M)
    return _on_rounded(corners, min(max(s, 0.0), route.length))


def _piece_end(corners: Sequence[Corner], s: float, length: float) -> float:
    for start, end, _ in corners:
        if s < start - _SAME_M:
            return start
        if s < end - _SAME_M:
            return end
    return length


def rounded_rest(route: Polyline, s: float, radius: float, here: Sequence[float] | None = None,
                 join_min_off: float = 0.0, join_slope: float = 0.0) -> list[Point]:
    return joined_rest(route, s, radius, here, join_min_off, join_slope)[0]


def joined_rest(route: Polyline, s: float, radius: float, here: Sequence[float] | None = None,
                join_min_off: float = 0.0,
                join_slope: float = 0.0) -> tuple[list[Point], float, float]:
    pts, corners = _rounded_with_corners(route.points, radius, ARC_STEP_M)
    line = Polyline(pts)
    at = _on_rounded(corners, min(max(s, 0.0), route.length))
    if here is None or join_slope <= 0.0 or at >= line.length - _SAME_M:
        return line.rest(at), at, 0.0
    spot = (float(here[0]), float(here[1]))
    at, off = line.project(spot, s_min=at - radius, s_max=at + radius)
    if off <= join_min_off or at >= line.length - _SAME_M:
        return line.rest(at), at, 0.0
    join = min(at + join_slope * off, _piece_end(corners, at, line.length))
    return _dedup([spot, *line.rest(join)]), join, math.dist(spot, line.point_at(join))


class Polyline:
    def __init__(self, points: Sequence[Point]):
        self.points = _dedup(points)
        if not self.points:
            raise ValueError("a polyline needs at least one point")
        self._s = [0.0]
        for a, b in pairwise(self.points):
            self._s.append(self._s[-1] + math.dist(a, b))
        self.length = self._s[-1]

    def project(self, p: Sequence[float], s_min: float = 0.0,
                s_max: float = math.inf) -> tuple[float, float]:
        px, py = float(p[0]), float(p[1])
        if len(self.points) == 1:
            return 0.0, math.hypot(px - self.points[0][0], py - self.points[0][1])
        best = (math.inf, 0.0)
        for i, ((ax, ay), (bx, by)) in enumerate(pairwise(self.points)):
            s0, s1 = self._s[i], self._s[i + 1]
            if s1 < s_min or s0 > s_max:
                continue
            seg = s1 - s0
            f = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / (seg * seg)
            lo = max(0.0, (s_min - s0) / seg)
            hi = min(1.0, (s_max - s0) / seg)
            f = min(max(f, lo), hi)
            qx, qy = ax + f * (bx - ax), ay + f * (by - ay)
            d = math.hypot(px - qx, py - qy)
            if d < best[0] - 1e-12:
                best = (d, s0 + f * seg)
        return best[1], best[0]

    def point_at(self, s: float) -> Point:
        if s <= 0.0 or len(self.points) == 1:
            return self.points[0]
        if s >= self.length:
            return self.points[-1]
        for i in range(len(self.points) - 1):
            if self._s[i + 1] >= s:
                (ax, ay), (bx, by) = self.points[i], self.points[i + 1]
                f = (s - self._s[i]) / (self._s[i + 1] - self._s[i])
                return (ax + f * (bx - ax), ay + f * (by - ay))
        return self.points[-1]

    def heading_at(self, s: float) -> float:
        if len(self.points) == 1:
            return 0.0
        i = 0
        while i < len(self.points) - 2 and self._s[i + 1] < s:
            i += 1
        (ax, ay), (bx, by) = self.points[i], self.points[i + 1]
        return math.atan2(by - ay, bx - ax)

    def vertex_before(self, s: float) -> float:
        best = 0.0
        for vs in self._s:
            if vs <= s + _SAME_M:
                best = vs
        return best

    def rest(self, s: float) -> list[Point]:
        if s >= self.length:
            return [self.points[-1]]
        start = self.point_at(s)
        later = [p for p, ps in zip(self.points, self._s, strict=True) if ps > s + _SAME_M]
        return _dedup([start, *later])

    def sample(self, s0: float, s1: float, step: float) -> list[Point]:
        s1 = min(s1, self.length)
        s0 = min(max(s0, 0.0), s1)
        n = max(1, math.ceil((s1 - s0) / step - 1e-9))
        return [self.point_at(s0 + (s1 - s0) * i / n) for i in range(n + 1)]


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
