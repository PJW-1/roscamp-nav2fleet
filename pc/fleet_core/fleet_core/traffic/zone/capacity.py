"""시작 때 한 번 보는 정원: 넓은 구역(shared) 정원의 합이 로봇 수보다 작으면 알린다.

모든 로봇이 넓은 구역에 설 수 없으면 서로를 끝없이 기다릴 수 있다(추가 지시서 2의 5절). 지도는
로봇 수를 모르므로 지도를 읽을 때가 아니라 교통 층이 처음 돌 때 본다. 알리기만 한다.
"""
from __future__ import annotations

from fleet_core.fleet_map import SHARED


class Capacity:
    def _check_capacity(self) -> None:
        if getattr(self, "_capacity_checked", False):
            return
        self._capacity_checked = True
        shared = {name: spec.capacity for name, spec in self.site.zones.items()
                  if spec.kind == SHARED}
        total, robots = sum(shared.values()), len(self._robots)
        if total < robots:
            each = ", ".join(f"{name} {capacity}" for name, capacity in shared.items())
            self._event("WARN", "ZONE_CAPACITY_LOW", f"shared zones hold {total} robots in all "
                        f"({each}) but {robots} robots run: they can wait on each other for good")
