"""order 모델의 조각 줄(계약 0.11-draft 제안, order 추가지시서 3절). ROS를 모른다.

order는 목적지 하나까지의 이동을 조각(Segment) 목록으로 보낸다. base는 달려도 되는 조각, horizon은
아직 허락되지 않은 조각이다(VDA 5050의 released). update는 새 MoveTo goal로 오고, 네 조건이 모두
맞을 때만 줄 뒤에 붙는다(stitch_problem). 줄에서 꺼낼 때 같은 방식으로 이어지는 조각은 하나로
합친다(take_run): follow_path끼리 controller가 같고 끝점과 시작점이 닿으면 한 경로가 된다.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from pinky_fsm.geometry import Pose
from pinky_fsm.nav_goal import MODE_FOLLOW_PATH, MODE_NAV_GOAL

MODE_RETREAT = "retreat"  # Segment.MODE_RETREAT
SEG_MODES = (MODE_FOLLOW_PATH, MODE_NAV_GOAL, MODE_RETREAT)
__all__ = ["MODE_FOLLOW_PATH", "MODE_NAV_GOAL", "MODE_RETREAT", "OrderGoal", "OrderQueue", "Seg"]


@dataclass(frozen=True)
class Seg:
    """fleet_interfaces/msg/Segment. path는 (x, y, yaw), target은 nav_goal의 (x, y, yaw)다."""
    seq_id: int
    mode: str
    controller_id: str = ""
    path: tuple[Pose, ...] = ()
    target: Pose | None = None
    end_node_id: str = ""
    zone_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class OrderGoal:
    """MoveTo goal의 order 칸. order_id가 비면 order가 아니다."""
    order_id: str
    update_id: int = 0
    stitch_node_id: str = ""
    base: tuple[Seg, ...] = ()
    horizon: tuple[Seg, ...] = ()


def segments_problem(segs: Sequence[Seg]) -> str:
    """base 조각을 달릴 수 있는가. 문제가 없으면 빈 문자열."""
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
    """b를 a의 FollowPath에 이어 붙여도 되는가(추가지시서 3절 3번)."""
    return (a.mode == b.mode == MODE_FOLLOW_PATH and a.controller_id == b.controller_id
            and math.dist(a.path[-1][:2], b.path[0][:2]) <= join_m)


class OrderQueue:
    """한 order의 줄. pending은 아직 시작하지 않은 base 조각이다."""

    def __init__(self, goal: OrderGoal):
        self.order_id = goal.order_id
        self.update_id = goal.update_id
        self.pending: list[Seg] = list(goal.base)
        self.horizon: list[Seg] = list(goal.horizon)
        # 지금까지 받은 base의 마지막 조각(이음 조건 3, 4). 받은 base가 없으면 0과 빈 이름이다
        self.last_seq = goal.base[-1].seq_id if goal.base else 0
        self.last_node = goal.base[-1].end_node_id if goal.base else ""
        self.done = 0  # 끝낸 마지막 조각(last_done_seq_id)

    def stitch_problem(self, u: OrderGoal) -> str:
        """update를 붙일 수 없는 이유. 네 조건과 조각 검사 순서다."""
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
        """맞으면 붙이고 빈 문자열, 틀리면 이유를 돌려주고 줄은 그대로 둔다.
        horizon은 통째로 바꾼다."""
        problem = self.stitch_problem(u)
        if problem:
            return problem
        self.update_id = u.update_id
        self.pending += u.base
        self.horizon = list(u.horizon)
        if u.base:
            self.last_seq, self.last_node = u.base[-1].seq_id, u.base[-1].end_node_id
        return ""

    def take_run(self, join_m: float) -> list[Seg]:
        """다음 조각과, 거기에 합칠 수 있는 뒤 조각들을 꺼낸다."""
        run = [self.pending.pop(0)]
        return run + self.take_merge(run[-1], join_m)

    def take_merge(self, last: Seg, join_m: float) -> list[Seg]:
        """last에 차례로 합칠 수 있는 앞쪽 조각을 꺼낸다."""
        out: list[Seg] = []
        while self.pending and mergeable(out[-1] if out else last, self.pending[0], join_m):
            out.append(self.pending.pop(0))
        return out
