"""SafetyMeter: 실제 자세로 로봇이 얼마나 가까워졌는지 잰다. ROS를 쓰지 않는다.

심판은 그대로 두고 이 값은 요약 옆 safety.json에 둔다.
  early: 몸이 구역에서 반지름 안에 있는데 그 로봇의 구역을 풀었다(구역 경계에서 잰다).
  near: 두 로봇이 반지름 두 배보다 가까워졌다. 쌍마다 일어날 때마다 한 번 센다.
구역 하나를 여러 로봇이 쥘 수 있다(/fleet/zones의 holder는 이름을 쉼표로 이은 것).
"""
from __future__ import annotations

import itertools
import math

from fake_robots.judge import holder_set


class SafetyMeter:
    def __init__(self, site, radius_m: float, on_course=lambda robot: True):
        self.site = site
        self.radius_m = radius_m
        self.on_course = on_course
        self.poses: dict[str, tuple[float, float]] = {}
        self.holders: dict[str, frozenset[str]] = {}
        self.early = 0
        self.near = 0
        self.releases = 0
        self.min_gap_m = math.inf
        self.min_release_m = math.inf
        self.log: list[str] = []
        self._close: set[tuple[str, str]] = set()

    def on_pose(self, robot: str, x: float, y: float) -> None:
        self.poses[robot] = (x, y)

    def on_zones(self, t: float, holders: dict[str, str]) -> None:
        for zone, text in holders.items():
            new = holder_set(text)
            old = self.holders.get(zone, frozenset())
            self.holders[zone] = new
            for robot in sorted(old - new):
                if robot in self.poses:
                    self._on_release(t, zone, robot)

    def _on_release(self, t: float, zone: str, robot: str) -> None:
        x, y = self.poses[robot]
        gap = self.site.distance_to_zone(x, y, zone)
        self.releases += 1
        self.min_release_m = min(self.min_release_m, gap)
        if gap < self.radius_m:
            self.early += 1
            self.log.append(f"t={t:.1f} {robot} released {zone} {gap:.3f} m from it")

    def tick(self, t: float) -> None:
        present = sorted(r for r in self.poses if self.on_course(r))
        for one, other in itertools.combinations(present, 2):
            gap = math.dist(self.poses[one], self.poses[other])
            self.min_gap_m = min(self.min_gap_m, gap)
            if gap >= 2 * self.radius_m:
                self._close.discard((one, other))
            elif (one, other) not in self._close:
                self._close.add((one, other))
                self.near += 1
                self.log.append(f"t={t:.1f} {one} and {other} are {gap:.3f} m apart")

    def summary(self) -> dict:
        def finite(value):
            return value if math.isfinite(value) else None

        return {"early": self.early, "near": self.near, "releases": self.releases,
                "min_gap_m": finite(self.min_gap_m), "min_release_m": finite(self.min_release_m),
                "radius_m": self.radius_m, "log": self.log}
