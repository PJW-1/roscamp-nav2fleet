# ---- 판정 이름 ----
import math
from dataclasses import dataclass

FRESH, SAME, OFF, OTHER = "fresh", "same", "off", "other"
KEEP, BLOCKED, SILENT, SHORTER = "keep", "blocked", "silent", "shorter"
OPEN, UNKNOWN = "open", "unknown"


# ---- 규칙 값 ----
@dataclass(frozen=True)
class Rules:
    same_route_m: float = 0.20
    off_route_m: float = 0.30
    shorter_m: float = 0.30
    shorter_ratio: float = 0.2
    same_goal_m: float = 0.05


DEFAULT = Rules()


# ---- 전역 비용 지도 격자 ----
@dataclass(frozen=True)
class Grid:
    origin_x: float
    origin_y: float
    yaw: float
    resolution: float
    width: int
    height: int
    data: object


# ---- 꺾은선의 길이와 거리 ----
def length(line) -> float:
    return sum(math.dist(a, b) for a, b in zip(line, line[1:], strict=False))


def _to_segment(p, a, b) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    span = dx * dx + dy * dy
    t = 0.0 if span == 0.0 else ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / span
    t = min(1.0, max(0.0, t))
    return math.dist(p, (a[0] + t * dx, a[1] + t * dy))


def distance_to(line, p) -> float:
    if len(line) == 1:
        return math.dist(p, line[0])
    return min(_to_segment(p, a, b) for a, b in zip(line, line[1:], strict=False))


def _nearest(line, p) -> int:
    return min(range(len(line)), key=lambda i: math.dist(line[i], p))


# ---- 처음 보기: 지켜 둔 길의 남은 부분과 새 길 ----
def first_look(kept, kept_goal, new, goal, rules: Rules = DEFAULT) -> tuple[str, int]:
    if not kept or not new or kept_goal is None:
        return FRESH, 0
    if math.dist(kept_goal, goal) >= rules.same_goal_m:
        return FRESH, 0
    start = _nearest(kept, new[0])
    rest = kept[start:]
    if len(rest) < 2:
        return FRESH, start
    if distance_to(rest, new[0]) > rules.off_route_m:
        return OFF, start
    if max(distance_to(rest, p) for p in new) < rules.same_route_m:
        return SAME, start
    return OTHER, start


# ---- 다음 보기: 다른 길이면 막힘과 길이를 본다 ----
def second_look(rest, new, valid, rules: Rules = DEFAULT) -> str:
    if valid is None:
        return SILENT
    if not valid:
        return BLOCKED
    old = length(rest)
    if length(new) < old - max(rules.shorter_m, rules.shorter_ratio * old):
        return SHORTER
    return KEEP


# ---- 막힘: 남은 길의 칸 가운데 planner가 지나가지 못하는 칸 ----
def check_route(grid, route, blocked_cost: int = 99):
    if grid is None or abs(grid.yaw) > 1e-9:
        return UNKNOWN
    for x, y in route[:-1]:
        mx = math.floor((x - grid.origin_x) / grid.resolution)
        my = math.floor((y - grid.origin_y) / grid.resolution)
        if not (0 <= mx < grid.width and 0 <= my < grid.height):
            return x, y, None
        cost = grid.data[my * grid.width + mx]
        if cost >= blocked_cost:
            return x, y, cost
    return OPEN


# ---- 묵은 비용 지도 ----
def stale(received_s: float, now_s: float, timeout_s: float) -> bool:
    return now_s - received_s > timeout_s
