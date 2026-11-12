"""위치 추정 감시: 보고 자세가 로봇이 갈 수 있는 것보다 크게 튀면 사건으로 남긴다.

로봇의 위치 추정(AMCL)이 다른 자세로 옮겨 붙으면 보고 위치가 한 번에 크게 바뀐다. 관제는 실제
위치를 모르므로 "앞 보고에서 이 시간 안에 갈 수 없는 위치"만 잡는다. 바퀴가 헛돌아 보고 자세가
천천히 흐르는 것은 로봇이 정말 움직인 것과 구별되지 않아 여기서 잡지 못한다.
방향만 바뀐 것은 보지 않는다: 180° 뒤집힌 자세는 경기장 가운데가 아니면 위치도 함께 옮겨지고,
시험 세계의 로봇은 제자리에서 한 번에 돈다.
기록만 한다(LOCALIZATION_SUSPECT). 로봇을 세우지 않는다.
"""
from __future__ import annotations

import math

SPEED_CAP_MPS = 0.30  # 로봇이 낼 수 있는 속도보다 큰 값(Nav2 설정의 상한 0.25 m/s)


def pose_jump(before, after, dt: float, p) -> str:
    """dt 사이의 두 보고 자세 (x, y, yaw)의 위치가 갈 수 없는 만큼 다르면 그 까닭, 아니면 ""."""
    if dt <= 0.0 or dt > p.pose_jump_window_s:
        return ""  # 너무 떨어진 보고는 견주지 않는다
    moved = math.dist(before[:2], after[:2])
    if moved <= SPEED_CAP_MPS * dt + p.pose_jump_m:
        return ""
    turned = abs(math.remainder(after[2] - before[2], math.tau))
    return f"moved {moved:.2f} m and turned {math.degrees(turned):.0f} deg in {dt:.1f} s"


class Localization:
    def _watch_pose_jump(self, m, x: float, y: float, yaw: float, now: float) -> None:
        """새 보고 자세를 앞 자세와 견준다. 앞 자세를 믿을 수 있을 때만이다."""
        before, at = (m.x, m.y, m.yaw), m.pose_at
        m.pose_at = now
        if at is None:
            return  # 첫 자세이거나 위치를 다시 잡은 뒤다
        why = pose_jump(before, (x, y, yaw), now - at, self.params)
        if why and now >= m.suspect_quiet_until:
            m.suspect_quiet_until = now + self.params.pose_jump_quiet_s
            self._event("WARN", "LOCALIZATION_SUSPECT",
                        f"the reported pose {why}: ({before[0]:.2f}, {before[1]:.2f}) to "
                        f"({x:.2f}, {y:.2f})", m.name)
