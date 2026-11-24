"""앞에 있는 다른 로봇을 보고 서기. ROS 가짜 Pode의 두 이동 모드가 같이 쓴다. ROS를 쓰지 않는다.

늦게 받은 남의 자세로 지금 자리를 어림하고, nav_goal 모드는 방금 움직인 앞선 이름의 로봇에게 양보한다.
"""
from __future__ import annotations

import math

from .world.geom import ahead, path_gap
from .world.runs import FOLLOW_BLOCKED_S, GAP_M, LOOK_M

SEEN_FOR_S = 0.3
LIFTED_MPS = 1.0
YIELD_M = 0.26
YIELD_FOR_S = 1.0


# ---- 늦게 받는 남의 자세로 지금 자리 어림하기 ----
def seen(before, x, y, t):
    if before is None:
        return (x, y, t, 0.0, 0.0, -math.inf)
    dt = t - before[2]
    stood = math.dist((x, y), before[:2]) < 1e-6
    moved_at = t if not stood else before[5] if dt >= 0.0 else -math.inf
    if dt <= 0.0:
        return (x, y, t, 0.0, 0.0, moved_at)
    vx, vy = (x - before[0]) / dt, (y - before[1]) / dt
    if dt > SEEN_FOR_S or math.hypot(vx, vy) > LIFTED_MPS:
        vx = vy = 0.0
    return (x, y, t, vx, vy, moved_at)


def where_now(view, t):
    x, y, at, vx, vy, _ = view
    age = min(max(t - at, 0.0), SEEN_FOR_S)
    return x + vx * age, y + vy * age


# ---- 가까이서 방금 움직인 로봇에게 양보하기 ----
def yields(x, y, views, t) -> bool:
    return any(0.0 <= t - view[5] < YIELD_FOR_S and math.dist((x, y), where_now(view, t)) <= YIELD_M
               for view in views)


def nothing_ahead(motion, radius, step, others) -> bool:
    """남은 길의 앞 LOOK_M + step 안에 몸이 닿을 로봇이 없다(가까워지는 것만 본다)."""
    if not motion.moving:
        return True
    seg = ahead([(motion.x, motion.y), *motion.via, motion.target], LOOK_M + step)
    for x, y, r in others:
        gap = path_gap((x, y), seg)
        if gap <= radius + r + GAP_M and gap < math.dist((x, y), (motion.x, motion.y)) - 1e-9:
            return False
    return True


class PathWatch:
    """path 모드 목표 하나. 앞에 로봇이 있으면 서고, FOLLOW_BLOCKED_S 이어서 서 있으면 BLOCKED다."""

    def __init__(self, radius):
        self.radius = radius
        self._stood_at: float | None = None

    def clear_ahead(self, t, motion, step, others) -> bool:
        if nothing_ahead(motion, self.radius, step, others):
            self._stood_at = None
            return True
        if self._stood_at is None:
            self._stood_at = t
        return False

    def moved(self) -> None:
        """뒤로 물러났다(자기 복구). 서 있던 시간을 처음부터 다시 센다."""
        self._stood_at = None

    def blocked(self, t) -> bool:
        return self._stood_at is not None and t - self._stood_at >= FOLLOW_BLOCKED_S - 1e-9
