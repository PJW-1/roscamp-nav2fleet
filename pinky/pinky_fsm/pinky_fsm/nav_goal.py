"""MoveTo mode nav_goal(계약 0.11-draft, 사용자 기준): Nav2 NavigateToPose로 가는 이동.

로봇이 target_pose까지 길을 스스로 짜고, 막히면 BT가 지도를 지우고 기다린다(추가 지시서 3의 7-4:
Spin과 BackUp을 뺐다). 물러나기는 로봇단 교착 처리(yield_nav.py)만 한다. 여기서는 ROS를 모른 채
아래만 정한다.
- 명령: SendNavGoal(노드가 NavigateToPose goal로 옮긴다), CancelNavGoal(그 취소).
- NavGoal: 실행 중인 nav_goal 이동 하나. seq 0이면 앞 이동이 끝나기를 기다리는 중이다.
- NavGoals: 보낸 NavigateToPose의 기록(아직 돌 수 있는 goal)과 진전 판단.

진전(BLOCKED 기준): 마지막으로 진전을 본 자리에서 로봇 중심이 progress_m(0.05 m, Nav2 progress
checker의 required_movement_radius와 같은 값) 이상 멀어지면 진전이다. 그 자리와 시각을 새로 잡는다.
blocked_after_s(10 s, 0.12까지 5 s) 동안 진전이 없으면 BLOCKED이고 goal은 그대로 둔다(Nav2가 계속
해 본다). 로봇단 교착 처리를 켜면 그 전에 기다리고 낮은 쪽만 물러난다(yield_nav.py).
제자리 회전은 진전이 아니다. 물러나는 것은 진전으로 친다(추정, 단순하게 둔다).
"""
from __future__ import annotations

import math
from collections.abc import Hashable, Sequence
from dataclasses import dataclass, field

from pinky_fsm.geometry import Point, Pose

# MoveTo.Goal.MODE_* 와 같은 값. 비면 follow_path다
MODE_FOLLOW_PATH = "follow_path"
MODE_NAV_GOAL = "nav_goal"
MODES = ("", MODE_FOLLOW_PATH, MODE_NAV_GOAL)


@dataclass(frozen=True)
class SendNavGoal:
    """Nav2 bt_navigator의 navigate_to_pose에 이 자세(map 틀 x, y, yaw)를 보낸다. 도는 goal은
    취소하지 않는다(bt_navigator가 새 goal로 바꾼다)."""
    seq: int
    pose: Pose


@dataclass(frozen=True)
class CancelNavGoal:
    """보낸 NavigateToPose goal을 모두 취소한다. BT가 controller를 세운다."""


@dataclass
class NavGoal:
    goal_id: Hashable
    target_id: str
    target: Pose
    since: float  # 받은 시각. seq 0인 동안 앞 이동을 기다린 시간을 잰다
    seq: int = 0  # 보낸 NavigateToPose 순번. 0이면 아직 보내지 않았다
    distance: float = 0.0  # Nav2 feedback의 distance_remaining. 0이면 아직 모른다
    far_successes: int = 0
    anchor: Point | None = None  # 마지막으로 진전을 본 자리
    anchor_at: float = 0.0
    blocked: bool = False
    # 로봇단 교착 처리(7-4): 이 goal에서 지나온 길의 끝부분과 지나온 거리(yield_nav.py)
    trail: list[Point] = field(default_factory=list)
    driven: float = 0.0


class NavGoals:
    """보낸 NavigateToPose goal들. 결과가 오거나, 취소가 받아들여지고 여유가 지나면 잊는다
    (core의 FollowPath 기록 _nav_live와 같은 규칙)."""

    def __init__(self, blocked_after_s: float, progress_m: float, result_grace_s: float):
        self.blocked_after_s = blocked_after_s
        self.progress_m = progress_m
        self.result_grace_s = result_grace_s
        self._seq = 0
        self._live: dict[int, float] = {}  # 순번 → 이 시각 뒤에는 잊는다(inf: 취소 답 전)
        self._uncancelled: set[int] = set()  # 아직 취소를 보내지 않은 goal

    def send(self, g: NavGoal, now: float, pose: Sequence[float] | None) -> list:
        self._seq += 1
        g.seq, g.distance, g.blocked = self._seq, 0.0, False
        g.anchor = (float(pose[0]), float(pose[1])) if pose is not None else None
        g.anchor_at = now
        self._live[g.seq] = math.inf
        self._uncancelled.add(g.seq)
        return [SendNavGoal(g.seq, g.target)]

    def stop(self) -> list:
        if not self._uncancelled:
            return []
        self._uncancelled.clear()
        return [CancelNavGoal()]

    def forget(self, seq: int) -> None:
        """결과가 왔다. 그 goal은 돌지 않는다."""
        self._live.pop(seq, None)
        self._uncancelled.discard(seq)

    def cancel_accepted(self, seq: int, now: float) -> None:
        if self._live.get(seq) == math.inf:
            self._live[seq] = now + self.result_grace_s

    def busy(self, now: float) -> bool:
        """아직 돌 수 있는 NavigateToPose가 있다."""
        for seq, until in list(self._live.items()):
            if now >= until:
                del self._live[seq]
        return bool(self._live)

    def watch(self, g: NavGoal, now: float, pose: Sequence[float] | None) -> None:
        """진전을 보고 BLOCKED를 켜거나 끈다. 보낸 goal에만 쓴다."""
        if pose is None or g.seq == 0:
            return
        here = (float(pose[0]), float(pose[1]))
        if g.anchor is None or math.dist(here, g.anchor) >= self.progress_m:
            g.anchor, g.anchor_at, g.blocked = here, now, False
            return
        if now - g.anchor_at >= self.blocked_after_s:
            g.blocked = True

    @staticmethod
    def distance_remaining(g: NavGoal, pose: Sequence[float] | None) -> float:
        """Nav2가 알려 준 남은 길. 첫 경로 전(0)에는 목표까지 곧은 거리."""
        if g.distance > 0.0 or pose is None:
            return g.distance
        return math.dist(pose[:2], g.target[:2])
