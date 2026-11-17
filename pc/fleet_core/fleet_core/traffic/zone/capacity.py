# ---- SHARED 구역 수용량 점검 ----
from __future__ import annotations

from fleet_core.fleet_map import SHARED


# ---- SHARED 구역 수용량 합이 로봇 수보다 적으면 경고 ----
class CapacityCheck:
    def __init__(self, site, emit):
        self.site = site
        self.emit = emit
        self.checked = False

    def check(self, robots: int) -> None:
        if self.checked:
            return
        self.checked = True
        shared = {name: spec.capacity for name, spec in self.site.zones.items()
                  if spec.kind == SHARED}
        total = sum(shared.values())
        if total < robots:
            each = ", ".join(f"{name} {capacity}" for name, capacity in shared.items())
            self.emit("WARN", "ZONE_CAPACITY_LOW", f"shared zones hold {total} robots in all "
                      f"({each}) but {robots} robots run: they can wait on each other for good")
