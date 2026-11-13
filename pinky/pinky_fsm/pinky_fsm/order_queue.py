# ---- 주문 구간 자료형과 구간 대기열 ----
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from pinky_fsm.geometry import Pose
from pinky_fsm.nav_goal import MODE_FOLLOW_PATH, MODE_NAV_GOAL

# ---- 구간 모드 상수와 구간, 주문 자료형 ----
MODE_RETREAT = "retreat"
SEG_MODES = (MODE_FOLLOW_PATH, MODE_NAV_GOAL, MODE_RETREAT)
__all__ = ["MODE_FOLLOW_PATH", "MODE_NAV_GOAL", "MODE_RETREAT", "OrderGoal", "OrderQueue", "Seg"]


@dataclass(frozen=True)
class Seg:
    seq_id: int
    mode: str
    controller_id: str = ""
    path: tuple[Pose, ...] = ()
    target: Pose | None = None
    end_node_id: str = ""
    zone_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class OrderGoal:
    order_id: str
    update_id: int = 0
    stitch_node_id: str = ""
    base: tuple[Seg, ...] = ()
    horizon: tuple[Seg, ...] = ()


# ---- 구간 순서와 모드 검사, 합칠 수 있는지 판정 ----
def segments_problem(segs: Sequence[Seg]) -> str:
    for a, b in zip(segs, segs[1:], strict=False):
        if b.seq_id != a.seq_id + 1:
            return f"seq_id {b.seq_id} does not follow {a.seq_id}"
    for s in segs:
        if s.mode not in SEG_MODES:
            return f"seq {s.seq_id}: unknown mode '{s.mode}'"
        if s.mode == MODE_NAV_GOAL and s.target is None:
            return f"seq {s.seq_id}: nav_goal without target_pose"
        if s.mode != MODE_NAV_GOAL and len(s.path) < 2:
            return f"seq {s.seq_id}: {s.mode} needs a path of two poses or more"
    return ""


def mergeable(a: Seg, b: Seg, join_m: float) -> bool:
    return (a.mode == b.mode == MODE_FOLLOW_PATH and a.controller_id == b.controller_id
            and math.dist(a.path[-1][:2], b.path[0][:2]) <= join_m)


# ---- 주문 대기열과 갱신 이어 붙이기 검사 ----
class OrderQueue:
    def __init__(self, goal: OrderGoal):
        self.order_id = goal.order_id
        self.update_id = goal.update_id
        self.pending: list[Seg] = list(goal.base)
        self.horizon: list[Seg] = list(goal.horizon)
        self.last_seq = goal.base[-1].seq_id if goal.base else 0
        self.last_node = goal.base[-1].end_node_id if goal.base else ""
        self.done = 0

    def stitch_problem(self, u: OrderGoal) -> str:
        if u.order_id != self.order_id:
            return f"order_id '{u.order_id}' is not the running '{self.order_id}'"
        if u.update_id != self.update_id + 1:
            return f"order_update_id {u.update_id} is not {self.update_id + 1}"
        if u.stitch_node_id != self.last_node:
            return f"stitch_node_id '{u.stitch_node_id}' is not the base end '{self.last_node}'"
        if u.base and u.base[0].seq_id != self.last_seq + 1:
            return f"first seq_id {u.base[0].seq_id} is not {self.last_seq + 1}"
        return segments_problem(u.base)

    def stitch(self, u: OrderGoal) -> str:
        problem = self.stitch_problem(u)
        if problem:
            return problem
        self.update_id = u.update_id
        self.pending += u.base
        self.horizon = list(u.horizon)
        if u.base:
            self.last_seq, self.last_node = u.base[-1].seq_id, u.base[-1].end_node_id
        return ""

    # ---- 다음 구간 묶음 꺼내기 ----
    def take_run(self, join_m: float) -> list[Seg]:
        run = [self.pending.pop(0)]
        return run + self.take_merge(run[-1], join_m)

    def take_merge(self, last: Seg, join_m: float) -> list[Seg]:
        out: list[Seg] = []
        while self.pending and mergeable(out[-1] if out else last, self.pending[0], join_m):
            out.append(self.pending.pop(0))
        return out
