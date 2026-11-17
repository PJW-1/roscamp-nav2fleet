# ---- 감시 일화 기록 자료형 ----
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Episode:
    key: object = None
    since: float | None = None
    xy: tuple | None = None
    noted: object = False
    count: int = 0

    def clear(self) -> None:
        self.key = self.since = self.xy = None
        self.noted = False
        self.count = 0


@dataclass
class WaitEpisode(Episode):
    key: tuple = ()
    queued: bool = False

    def clear(self) -> None:
        super().clear()
        self.key = ()
        self.queued = False
