"""막힘 처리 보완(추가 지시서 3의 4절)의 숫자: 치운 상자 자리의 막힘 표시가 언제 풀렸나, 우회 종류.

걸음마다 관제의 막힘 표시(traffic._blocked, _marks)를 본다. 상자를 치운 시각에 그 자리를 막은
표시가 살아 있었으면, 그 표시가 모두 풀린 때까지의 시간을 잰다(BLOCK_CLEARED든 시간이 다 된
것이든). 살아 있지 않았으면 잴 것이 없다.
"""
from __future__ import annotations

import math

from fleet_core.traffic.zone.block_marks import keys_at
from fleet_core.traffic.zone.detours import CORRIDOR, DESTINATION, IN_ZONE, KEEPOUT

from .injections import Obstacle

DETOUR_KINDS = (IN_ZONE, KEEPOUT, CORRIDOR, DESTINATION)


class BlockWatch:
    def __init__(self, world, injections):
        self.world = world
        self.boxes = [o for o in injections if isinstance(o, Obstacle) and math.isfinite(o.t1)]
        self.keys = {o: set(keys_at(world.site, world.params, (o.x, o.y))) for o in self.boxes}
        self.live_at_removal: dict = {}  # 상자 -> 치울 때 표시가 살아 있었나
        self.cleared_after: dict = {}  # 상자 -> 치운 뒤 표시가 풀리기까지(s)

    def _live(self, box, t: float) -> bool:
        traffic = self.world.traffic
        blocked = getattr(traffic, "_blocked", {})
        marks = getattr(traffic, "_marks", {})
        near = self.world.params.near_m
        return (any(blocked.get(key, 0.0) > t for key in self.keys[box])
                or any(math.dist((mk.x, mk.y), (box.x, box.y)) <= near + box.radius
                       for mk in marks.values()))

    def step(self) -> None:
        t = self.world.t
        for box in self.boxes:
            if t < box.t1 or box in self.cleared_after:
                continue
            live = self._live(box, t)
            self.live_at_removal.setdefault(box, live)
            if not self.live_at_removal[box]:
                self.cleared_after[box] = None
            elif not live:
                self.cleared_after[box] = t - box.t1

    def cleared_cell(self) -> str:
        """치운 뒤 풀리기까지(s). 상자가 없으면 "-", 치울 때 표시가 없었으면 "표시 없음"."""
        if not self.boxes:
            return "-"
        cells = []
        for box in self.boxes:
            if box not in self.live_at_removal:
                cells.append("치우기 전 끝")
            elif box not in self.cleared_after:
                cells.append(f"> {self.world.t - box.t1:.0f}")
            elif self.cleared_after[box] is None:
                cells.append("표시 없음")
            else:
                cells.append(f"{self.cleared_after[box]:.1f}")
        return " / ".join(cells)


def detour_counts(world) -> dict:
    """DETOUR 사건을 종류마다 센다."""
    counts = dict.fromkeys(DETOUR_KINDS, 0)
    for _, event in world.events:
        if event.code == "DETOUR":
            kind = event.detail.split(":")[0]
            counts[kind] = counts.get(kind, 0) + 1
    return counts
