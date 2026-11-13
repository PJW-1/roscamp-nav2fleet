# ---- LANE 구역 방향 결정 정책 ----
from __future__ import annotations

import math
from dataclasses import dataclass, field


# ---- 로봇이 원하는 구역과 노드, 차선 방향 ----
@dataclass
class Want:
    zones: tuple
    nodes: tuple
    lanes: dict
    since: float
    eta: dict = field(default_factory=dict)


# ---- 대기 시간과 도착 예상 시간으로 차선 방향 거절 사유 판단 ----
class LanePolicy:
    def __init__(self, params):
        self._p = params
        self.last: dict[str, str] = {}

    def refusal(self, robot: str, zone: str, want: Want, now: float, rivals: dict) -> str:
        p = self._p
        direction = want.lanes[zone]
        waited = now - want.since
        for name in sorted(rivals):
            their_wait = now - rivals[name].since
            if their_wait >= p.lane_handover_s and their_wait > waited:
                return f"handover: {name} waited {their_wait:.0f} s"
        if waited >= p.lane_handover_s and all(now - rival.since < waited
                                               for rival in rivals.values()):
            return ""
        mine = want.eta.get(zone, 0.0)
        for name in sorted(rivals):
            theirs = rivals[name].eta.get(zone, math.inf)
            last = self.last.get(zone)
            if last == direction:
                wins = mine < theirs + p.lane_hysteresis_s
            elif last == rivals[name].lanes[zone]:
                wins = mine <= theirs - p.lane_hysteresis_s
            else:
                wins = mine < theirs or (mine == theirs and robot < name)
            if not wins:
                return (f"direction: {name} reaches {zone} in {theirs:.1f} s, "
                        f"this robot in {mine:.1f} s")
        return ""

    # ---- 양보 여부와 마지막 방향 기록 ----
    def handed_over(self, want: Want, now: float) -> bool:
        return now - want.since >= self._p.lane_handover_s

    def note(self, zone: str, direction: str) -> bool:
        changed = self.last.get(zone) not in (None, direction)
        self.last[zone] = direction
        return changed
