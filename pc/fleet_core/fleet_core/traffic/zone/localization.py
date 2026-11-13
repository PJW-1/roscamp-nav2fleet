# ---- 위치 추정 튐 감지 믹스인 ----
from __future__ import annotations

import math

SPEED_CAP_MPS = 0.30


# ---- 짧은 시간에 너무 크게 움직인 자세 판정 ----
def pose_jump(before, after, dt: float, p) -> str:
    if dt <= 0.0 or dt > p.pose_jump_window_s:
        return ""
    moved = math.dist(before[:2], after[:2])
    if moved <= SPEED_CAP_MPS * dt + p.pose_jump_m:
        return ""
    turned = abs(math.remainder(after[2] - before[2], math.tau))
    return f"moved {moved:.2f} m and turned {math.degrees(turned):.0f} deg in {dt:.1f} s"


# ---- 보고된 자세가 튀면 LOCALIZATION_SUSPECT 경고 ----
class Localization:
    def _watch_pose_jump(self, m, x: float, y: float, yaw: float, now: float) -> None:
        before, at = (m.x, m.y, m.yaw), m.pose_at
        m.pose_at = now
        if at is None:
            return
        why = pose_jump(before, (x, y, yaw), now - at, self.params)
        if why and now >= m.suspect_quiet_until:
            m.suspect_quiet_until = now + self.params.pose_jump_quiet_s
            self._event("WARN", "LOCALIZATION_SUSPECT",
                        f"the reported pose {why}: ({before[0]:.2f}, {before[1]:.2f}) to "
                        f"({x:.2f}, {y:.2f})", m.name)
