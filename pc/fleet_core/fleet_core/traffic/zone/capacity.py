# ---- SHARED 구역 수용량 점검 믹스인 ----
from __future__ import annotations

from fleet_core.fleet_map import SHARED


# ---- SHARED 구역 수용량 합이 로봇 수보다 적으면 경고 ----
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
