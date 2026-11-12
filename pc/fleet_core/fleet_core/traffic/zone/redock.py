"""도킹이 실패한 로봇: 같은 자리에서 다시 하기 전에 도크 앞 지점까지 물렸다가 다시 들여보낸다.

도킹은 보고 위치가 맞아야 된다. 실패한 자리에서 그대로 다시 하면 위치 추정이 틀어진 로봇은 같은
자세로 세 번 실패하고 격리된다(B-G5). 고정 경로를 한 번 더 달리면 위치 추정이 움직임과 함께 다시
맞춰질 기회가 생긴다(AMCL은 움직여야 고친다). 실패한 도킹은 그대로 시도로 센다.
"""
from __future__ import annotations

from fleet_core.fleet_map import FOLLOW_PATH, RETREAT

from .trips import Trip


class Redock:
    def _dock_failed(self, m) -> None:
        """도킹이 실패로 끝났다. 물러났다 들어오는 사이에 지워지는 attempts 대신 따로 센다."""
        m.dock_tries += 1
        m.attempts = m.dock_tries - 1
        m.redock = True
        self._goal_failed(m, "DOCK_FAILED")

    def _reapproach(self, m) -> bool:
        """도크 앞 지점까지 후진했다가 다시 들어오는 길을 잡는다. 그런 길이 없으면 False."""
        m.redock = False
        dock = self.site.stations[m.dest].dock
        out = next((link for link in self.site.links_from(dock) if link.via == RETREAT), None)
        back = self.site.link(out.b, dock) if out is not None else None
        if back is None or back.via != FOLLOW_PATH:
            return False  # 뒤로 나오는 길이나 다시 들어오는 고정 경로가 없는 도크다
        m.trip = Trip(dock, [out, back])
        m.phase = self._leg_of(m.dest)
        self._event("INFO", "DOCK_REAPPROACH", f"backs to {out.b} and comes in to {dock} again",
                    m.name)
        return True
