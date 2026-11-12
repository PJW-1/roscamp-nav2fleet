"""회피 흉내 세 모드(추가 지시서 2의 1절). Nav2 goal 하나(nav_goal.py)마다 하나씩 만든다.

- none(최악): 정적 지도 위 길만 간다. 무엇이 길에 있으면 서고, 돌아가는 길을 짜지 않는다.
- perfect(최선): 격자 A*로 다른 로봇과 물체를 돌아간다(기본, 지금까지의 동작).
- mixed(중간): 막힐 때마다 p_detour 확률로 delay 뒤 돌아가고, 아니면 그 막힘 동안 none처럼 선다.
  goal마다 stray_p 확률로 길에서 옆으로 0~stray_max_m 벗어난다.
뽑기는 세계의 rng를 쓴다(같은 씨드면 같다). perfect는 rng를 쓰지 않는다.
FollowPath, Retreat는 회피와 상관없어 이 모드를 쓰지 않는다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

NONE, PERFECT, MIXED = "none", "perfect", "mixed"
AVOID_MODES = (NONE, PERFECT, MIXED)


@dataclass(frozen=True)
class MixedSettings:
    """mixed의 값(추가 지시서 2의 9절 제안값)."""

    p_detour: float = 0.7  # 막힘마다 돌아갈 확률
    delay_max_s: float = 3.0  # 돌아가기 전 0~이만큼 기다림
    stray_p: float = 0.05  # goal마다 길에서 벗어날 확률
    stray_max_m: float = 0.05  # 벗어나는 옆 거리 0~이만큼


def check_mode(mode: str) -> str:
    if mode not in AVOID_MODES:
        raise ValueError(f"avoid_mode must be one of {AVOID_MODES}, got {mode!r}")
    return mode


class Avoidance:
    """Nav2 goal 하나 동안의 회피 흉내."""

    def __init__(self, mode: str = PERFECT, settings: MixedSettings | None = None):
        self.mode = check_mode(mode)
        self.settings = settings or MixedSettings()
        self._episode = False  # 지금 막혀 서 있다(mixed)
        self._detour_at: float | None = None  # 이번 막힘에서 돌아가기 시작할 시각

    @property
    def plans_around(self) -> bool:
        """처음 길부터 다른 로봇과 물체를 피해 짠다."""
        return self.mode == PERFECT

    def may_detour(self, rng, t: float) -> bool:
        """막혀 선 걸음마다 묻는다: 돌아가는 길을 짜도 되나."""
        if self.mode != MIXED:
            return self.mode == PERFECT
        if not self._episode:
            self._episode = True
            s = self.settings
            detour = rng.random() < s.p_detour
            delay = rng.uniform(0.0, s.delay_max_s)
            self._detour_at = t + delay if detour else None
        return self._detour_at is not None and t >= self._detour_at - 1e-9

    def moved(self) -> None:
        """다시 나아갔다: 이번 막힘은 끝났다."""
        self._episode = False

    def stray(self, rng, way):
        """mixed: stray_p 확률로 가장 긴 선분 가운데를 옆으로 민 점을 넣는다. 끝 점은 그대로."""
        s = self.settings
        if self.mode != MIXED or way is None or len(way) < 2 or rng.random() >= s.stray_p:
            return way
        k = max(range(len(way) - 1), key=lambda i: math.dist(way[i], way[i + 1]))
        (ax, ay), (bx, by) = way[k], way[k + 1]
        span = math.dist(way[k], way[k + 1])
        if span <= 1e-9:
            return way
        side = rng.uniform(0.0, s.stray_max_m) * (1 if rng.random() < 0.5 else -1)
        mid = ((ax + bx) / 2 - (by - ay) / span * side, (ay + by) / 2 + (bx - ax) / span * side)
        return [*way[:k + 1], mid, *way[k + 1:]]
