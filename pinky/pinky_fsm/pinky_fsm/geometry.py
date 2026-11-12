"""경로 기하. 레인 폴리라인을 Nav2 경로 점으로 펼치고, 로봇 위치를 선에 투영한다.

ROS 없이 돈다. 좌표는 지도(map) 틀의 m, 각도는 rad.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from itertools import pairwise

Point = tuple[float, float]
Pose = tuple[float, float, float]

_SAME_M = 1e-6  # 이보다 가까운 이웃 점은 같은 점으로 친다
ARC_STEP_M = 0.01  # 호를 이 간격의 점으로 편다


def _dedup(points: Sequence[Point]) -> list[Point]:
    out: list[Point] = []
    for p in points:
        p = (float(p[0]), float(p[1]))
        if not out or math.dist(out[-1], p) > _SAME_M:
            out.append(p)
    return out


def poses_along(points: Sequence[Point], final_yaw: float, step: float = 0.05) -> list[Pose]:
    """폴리라인을 step 이하 간격의 (x, y, yaw)로 펼친다.

    각 점의 yaw는 그 점이 속한 선분의 방향이고, 마지막 점만 final_yaw다.
    """
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
    """뒤로 따라갈 폴리라인을 poses_along과 같은 점으로 펼친다. yaw는 진행 방향의 반대, 즉
    후진하는 로봇이 보는 방향이다(마지막 점은 마지막 선분의 것). 점이 하나면 yaw는 0이다."""
    pts = _dedup(points)
    final = 0.0
    if len(pts) > 1:
        (ax, ay), (bx, by) = pts[-2], pts[-1]
        final = math.atan2(by - ay, bx - ax)
    return [(x, y, math.atan2(-math.sin(yaw), -math.cos(yaw)))
            for x, y, yaw in poses_along(pts, final, step)]


def _corner_arc(a: Point, b: Point, c: Point, room_in: float, room_out: float,
                radius: float, step: float) -> list[Point]:
    """꺾은선 a-b-c의 꼭짓점 b를 대신할 호의 점들. 돌 수 없으면(곧음, 되돌아감) [b]."""
    h1 = math.atan2(b[1] - a[1], b[0] - a[0])
    h2 = math.atan2(c[1] - b[1], c[0] - b[0])
    turn = math.remainder(h2 - h1, math.tau)
    if abs(turn) < 1e-3 or abs(turn) > math.pi - 1e-3:
        return [b]
    half = math.tan(abs(turn) / 2.0)
    cut = min(radius * half, room_in, room_out)  # 꼭짓점에서 호가 시작하는 거리
    r = cut / half
    start = (b[0] - cut * math.cos(h1), b[1] - cut * math.sin(h1))
    side = 1.0 if turn > 0 else -1.0  # 왼쪽으로 돌면 중심이 왼쪽에 있다
    centre = (start[0] - side * r * math.sin(h1), start[1] + side * r * math.cos(h1))
    a0 = math.atan2(start[1] - centre[1], start[0] - centre[0])
    n = max(2, math.ceil(r * abs(turn) / step))
    return [(centre[0] + r * math.cos(a0 + turn * i / n),
             centre[1] + r * math.sin(a0 + turn * i / n)) for i in range(n + 1)]


def rounded(points: Sequence[Point], radius: float, step: float = ARC_STEP_M) -> list[Point]:
    """꺾은선의 꼭짓점마다 반지름 radius의 호를 넣는다(추가 지시서 3의 2-1).

    호가 짧은 선분을 넘지 않게 줄인다: 가운데 선분은 양 끝의 호가 반씩, 첫 선분과 마지막 선분은
    하나뿐인 호가 다 쓸 수 있다. 되돌아가는 꼭짓점(180°)은 둥글게 할 수 없어 그대로 둔다.
    """
    return _rounded_with_arcs(points, radius, step)[0]


def _rounded_with_arcs(points: Sequence[Point], radius: float,
                       step: float) -> tuple[list[Point], list[tuple[float, float]]]:
    """rounded의 점들과, 그 선 위에서 호가 놓인 구간들 (시작 s, 끝 s)."""
    pts = _dedup(points)
    if len(pts) < 3 or radius <= 0.0:
        return pts, []
    last = len(pts) - 2  # 마지막 선분의 번호
    lengths = [math.dist(p, q) for p, q in pairwise(pts)]
    out, arcs, run = [pts[0]], [], 0.0
    for i in range(1, len(pts) - 1):
        room_in = lengths[i - 1] if i - 1 == 0 else lengths[i - 1] / 2.0
        room_out = lengths[i] if i == last else lengths[i] / 2.0
        arc = _corner_arc(pts[i - 1], pts[i], pts[i + 1], room_in, room_out, radius, step)
        start = run + math.dist(out[-1], arc[0])
        for p in arc:
            run += math.dist(out[-1], p)
            out.append(p)
        if len(arc) > 1:
            arcs.append((start, run))
    out.append(pts[-1])
    return _dedup(out), arcs


def _piece_end(arcs: Sequence[tuple[float, float]], s: float, length: float) -> float:
    """s가 놓인 조각(곧은 구간이나 호 하나)이 끝나는 s."""
    for start, end in arcs:
        if s < start - _SAME_M:
            return start
        if s < end - _SAME_M:
            return end
    return length


def rounded_rest(route: Polyline, s: float, radius: float, here: Sequence[float] | None = None,
                 join_min_off: float = 0.0, join_slope: float = 0.0) -> list[Point]:
    """route의 꼭짓점을 둥글게 한 선에서, route 위 s 자리에 가장 가까운 곳부터 끝까지.

    진행 거리 s는 관제가 보낸 꺾은선(route) 위에서 잰다. 둥근 선은 Nav2에 보낼 때만 쓴다.

    here(로봇의 실제 위치)와 join_slope를 주면 둥근 선에서 로봇에 가장 가까운 곳부터 낸다. 로봇이
    선에서 join_min_off 넘게 벗어나 있으면 here에서 시작해, 벗어난 거리의 join_slope배 앞에서 선에
    붙는다(이음 정밀도, 추가 지시서 3의 2-4). 붙는 곳은 지금 조각(곧은 구간이나 호 하나)의 끝을
    넘지 않는다: 꺾이는 곳을 가로지르는 지름길은 내지 않는다. 선의 끝에 닿았거나 지나친 로봇에게는
    끝 점만 낸다: 되돌아가는 조각을 붙이지 않는다.
    """
    pts, arcs = _rounded_with_arcs(route.points, radius, ARC_STEP_M)
    line = Polyline(pts)
    s = min(max(s, 0.0), route.length)
    # 호는 선을 줄인다. 둥근 선 위의 같은 자리는 s와 s - (줄어든 길이) 사이에 있다. 그 안에서만
    # 찾아야 같은 곳을 두 번 지나는 선에서 이미 지난 쪽을 고르지 않는다
    at, _ = line.project(route.point_at(s), s_min=s - (route.length - line.length), s_max=s)
    if here is None or join_slope <= 0.0 or at >= line.length - _SAME_M:
        return line.rest(at)
    spot = (float(here[0]), float(here[1]))
    at, off = line.project(spot, s_min=at - radius, s_max=at + radius)
    if off <= join_min_off or at >= line.length - _SAME_M:
        return line.rest(at)
    join = min(at + join_slope * off, _piece_end(arcs, at, line.length))
    return _dedup([spot, *line.rest(join)])


class Polyline:
    """누적 거리 s로 점을 찾는 폴리라인."""

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
        """p에 가장 가까운 선 위의 점. (그 점의 s, p까지의 거리)

        s_min, s_max를 주면 그 구간 안에서만 찾는다(같은 곳을 두 번 지나는 선에서
        이미 지난 쪽이나 너무 먼 앞쪽으로 뛰지 않게).
        """
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
        """s가 놓인 선분의 방향(rad). 꼭짓점에서는 지나온(뒤) 선분이다. 점 하나면 0."""
        if len(self.points) == 1:
            return 0.0
        i = 0
        while i < len(self.points) - 2 and self._s[i + 1] < s:
            i += 1
        (ax, ay), (bx, by) = self.points[i], self.points[i + 1]
        return math.atan2(by - ay, bx - ax)

    def vertex_before(self, s: float) -> float:
        """s 지점이나 그 뒤(지나온 쪽)에서 가장 가까운 꼭짓점의 s. 첫 점도 꼭짓점이다."""
        best = 0.0
        for vs in self._s:
            if vs <= s + _SAME_M:
                best = vs
        return best

    def rest(self, s: float) -> list[Point]:
        """s 지점부터 끝까지. s 뒤의 꼭짓점은 그대로 둔다."""
        if s >= self.length:
            return [self.points[-1]]
        start = self.point_at(s)
        later = [p for p, ps in zip(self.points, self._s, strict=True) if ps > s + _SAME_M]
        return _dedup([start, *later])

    def sample(self, s0: float, s1: float, step: float) -> list[Point]:
        """s0부터 s1(선 끝에서 자름)까지 step 이하 간격의 점."""
        s1 = min(s1, self.length)
        s0 = min(max(s0, 0.0), s1)
        n = max(1, math.ceil((s1 - s0) / step - 1e-9))
        return [self.point_at(s0 + (s1 - s0) * i / n) for i in range(n + 1)]


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
