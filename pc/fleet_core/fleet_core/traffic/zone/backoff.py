# ---- SHARED 구역에서 비켜서기 믹스인 ----
from __future__ import annotations

import math


# ---- 비켜서기 ----
class Backoff:
    def _stand_aside(self, m, other, now: float) -> None:
        p = self.params
        again = (m.avoid.yielded is not None and now - m.avoid.yielded[2] <= p.blocked_retry_s
                 and math.dist(m.avoid.yielded[:2], (m.x, m.y)) < p.near_m)
        m.avoid.yielded = (m.x, m.y, now)
        if again and self._to_parking(m, "INFO", "HALL_YIELD",
                                      f"out of the way of {other.name}, goes to", clear_of=other):
            m.avoid.yielded = None
            return
        stops = f"stops {p.near_stuck_s:.0f} s for {other.name}"
        m.avoid.yield_until = now + p.near_stuck_s
        self._event("INFO", "HALL_YIELD", stops, m.name)
