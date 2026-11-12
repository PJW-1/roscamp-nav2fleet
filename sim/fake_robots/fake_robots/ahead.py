"""앞에 있는 다른 로봇을 보고 서기. ROS 가짜 핑키의 두 이동 모드가 같이 쓴다. ROS를 쓰지 않는다.

world.runs.blocked_ahead와 같은 규칙을 Motion 위에서 쓴다. nav_goal 모드는 충돌 검사를 켠 Nav2가,
path 모드는 핑키의 앞 물체 검사가 로봇을 세우는 것을 흉내 낸다(world.FollowRun).
"""
from __future__ import annotations

import math

from .world.geom import ahead, path_gap
from .world.runs import FOLLOW_BLOCKED_S, GAP_M, LOOK_M


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

    def blocked(self, t) -> bool:
        return self._stood_at is not None and t - self._stood_at >= FOLLOW_BLOCKED_S - 1e-9
