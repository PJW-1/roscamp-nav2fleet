"""한 줄 구역의 방향 정하기(지시서 2-2의 4번). 순수 판단이다.

- 비어 있는 한 줄 구역에 양쪽이 함께 들어가려 하면 입구에 먼저 닿는 쪽(ETA)이 들어간다.
- 직전에 달린 방향은 반대쪽이 lane_hysteresis_s 넘게 이를 때만 바뀐다.
- 반대쪽이 lane_handover_s 넘게 기다렸으면 같은 방향으로 뒤따르지 않고 방향을 넘긴다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class Want:
    """로봇이 잡으려는 구간 하나."""
    zones: tuple
    nodes: tuple
    lanes: dict  # 한 줄 구역 -> 들어가는 쪽 지점
    since: float  # 못 잡고 서 있기 시작한 시각
    eta: dict = field(default_factory=dict)  # 한 줄 구역 -> 입구까지 남은 시간(s)


class LanePolicy:
    def __init__(self, params):
        self._p = params
        self.last: dict[str, str] = {}  # 한 줄 구역 -> 마지막으로 달린 방향

    def refusal(self, robot: str, zone: str, want: Want, now: float, rivals: dict) -> str:
        """이 로봇이 지금 그 방향으로 들어가면 안 되는 이유. 들어가도 되면 "".

        rivals는 같은 구역을 반대 방향으로 바라고, 그 구역만 비면 떠날 수 있는 로봇들이다.
        """
        p = self._p
        direction = want.lanes[zone]
        waited = now - want.since
        for name in sorted(rivals):
            their_wait = now - rivals[name].since
            if their_wait >= p.lane_handover_s and their_wait > waited:
                return f"handover: {name} waited {their_wait:.0f} s"
        if waited >= p.lane_handover_s and all(now - rival.since < waited
                                               for rival in rivals.values()):
            return ""  # 가장 오래 기다렸다. 넘겨받는다
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

    def handed_over(self, want: Want, now: float) -> bool:
        """오래 기다려서 방향을 넘겨받았는가."""
        return now - want.since >= self._p.lane_handover_s

    def note(self, zone: str, direction: str) -> bool:
        """그 방향으로 달리기 시작했다. 직전 방향과 다르면 True."""
        changed = self.last.get(zone) not in (None, direction)
        self.last[zone] = direction
        return changed
