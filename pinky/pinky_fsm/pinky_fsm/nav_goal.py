# ---- Nav2 NavigateToPose goal 기록과 진행 감시 ----
from __future__ import annotations

import math
from collections.abc import Hashable, Sequence
from dataclasses import dataclass, field

from pinky_fsm.geometry import Point, Pose

# ---- 주행 모드 상수와 NavGoal 명령, 기록 자료형 ----
MODE_FOLLOW_PATH = "follow_path"
MODE_NAV_GOAL = "nav_goal"
MODES = ("", MODE_FOLLOW_PATH, MODE_NAV_GOAL)


@dataclass(frozen=True)
class SendNavGoal:
    seq: int
    pose: Pose


@dataclass(frozen=True)
class CancelNavGoal:
    pass


@dataclass
class NavGoal:
    goal_id: Hashable
    target_id: str
    target: Pose
    since: float
    seq: int = 0
    distance: float = 0.0
    far_successes: int = 0
    anchor: Point | None = None
    anchor_at: float = 0.0
    turn_yaw: float | None = None
    turn_pending: float = 0.0
    turned_at: float = 0.0
    blocked: bool = False
    trail: list[Point] = field(default_factory=list)
    driven: float = 0.0


# ---- 자세에서 방향 꺼내기 ----
def _yaw_of(pose: Sequence[float] | None) -> float | None:
    return float(pose[2]) if pose is not None and len(pose) > 2 else None


# ---- NavGoal 전송, 취소, 진행 중 goal 관리 ----
class NavGoals:
    def __init__(self, blocked_after_s: float, progress_m: float, result_grace_s: float,
                 turn_step_rad: float = 0.0, turn_window_s: float = 0.0):
        self.blocked_after_s = blocked_after_s
        self.progress_m = progress_m
        self.result_grace_s = result_grace_s
        self.turn_step_rad = turn_step_rad
        self.turn_window_s = turn_window_s
        self._seq = 0
        self._live: dict[int, float] = {}
        self._uncancelled: set[int] = set()

    def send(self, g: NavGoal, now: float, pose: Sequence[float] | None) -> list:
        self._seq += 1
        g.seq, g.distance, g.blocked = self._seq, 0.0, False
        g.anchor = (float(pose[0]), float(pose[1])) if pose is not None else None
        g.anchor_at = g.turned_at = now
        g.turn_yaw, g.turn_pending = _yaw_of(pose), 0.0
        self._live[g.seq] = math.inf
        self._uncancelled.add(g.seq)
        return [SendNavGoal(g.seq, g.target)]

    def stop(self) -> list:
        if not self._uncancelled:
            return []
        self._uncancelled.clear()
        return [CancelNavGoal()]

    def forget(self, seq: int) -> None:
        self._live.pop(seq, None)
        self._uncancelled.discard(seq)

    def cancel_accepted(self, seq: int, now: float) -> None:
        if self._live.get(seq) == math.inf:
            self._live[seq] = now + self.result_grace_s

    def busy(self, now: float) -> bool:
        for seq, until in list(self._live.items()):
            if now >= until:
                del self._live[seq]
        return bool(self._live)

    # ---- 제자리 정체 감시와 남은 거리 계산 ----
    def watch(self, g: NavGoal, now: float, pose: Sequence[float] | None) -> None:
        if pose is None or g.seq == 0:
            return
        here = (float(pose[0]), float(pose[1]))
        yaw = _yaw_of(pose)
        if g.anchor is None or math.dist(here, g.anchor) >= self.progress_m:
            g.anchor, g.anchor_at, g.blocked = here, now, False
            g.turned_at, g.turn_yaw, g.turn_pending = now, yaw, 0.0
            return
        self._count_turn(g, now, yaw)
        if now - g.anchor_at >= self.blocked_after_s:
            g.blocked = True

    # ---- 움직인 뒤 잠깐의 제자리 회전을 기록 ----
    def _count_turn(self, g: NavGoal, now: float, yaw: float | None) -> None:
        if self.turn_step_rad <= 0.0 or yaw is None:
            return
        if g.turn_yaw is not None:
            g.turn_pending += abs(math.remainder(yaw - g.turn_yaw, math.tau))
        g.turn_yaw = yaw
        if g.turn_pending >= self.turn_step_rad and now - g.anchor_at <= self.turn_window_s:
            g.turned_at, g.turn_pending = now, 0.0

    @staticmethod
    def distance_remaining(g: NavGoal, pose: Sequence[float] | None) -> float:
        if g.distance > 0.0 or pose is None:
            return g.distance
        return math.dist(pose[:2], g.target[:2])
