"""막힘 처리 보완의 숫자: 치운 상자 자리의 막힘 표시가 풀리기까지의 시간, 우회 종류.

걸음마다 관제의 막힘 표시(읽기 전용 관측 API)를 보고, 상자를 치운 시각에 살아 있던 표시가
모두 풀린 때까지를 잰다.
"""
from __future__ import annotations

import math

from fleet_core.traffic.api import CORRIDOR, DESTINATION, IN_ZONE, KEEPOUT, keys_at

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
        if not hasattr(traffic, "live_block"):  # 관제가 없으면 막힘 표시도 없다
            return False
        radius = self.world.params.near_m + box.radius
        return traffic.live_block(self.keys[box], (box.x, box.y), radius, t)

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
