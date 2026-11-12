"""도킹이 실패한 로봇: 같은 자리에서 다시 하기 전에 도크 앞 지점까지 물렸다가 다시 들여보낸다.

도킹은 보고 위치가 맞아야 된다. 실패한 자리에서 그대로 다시 하면 위치 추정이 틀어진 로봇은 같은
자세로 세 번 실패하고 격리된다(B-G5). 고정 경로를 한 번 더 달리면 위치 추정이 움직임과 함께 다시
맞춰질 기회가 생긴다(AMCL은 움직여야 고친다). 실패한 도킹은 그대로 시도로 센다.
"""
from __future__ import annotations

from fleet_core.fleet_map import FOLLOW_PATH, RETREAT

from .trips import Trip


class Redock:
    def _dock_failed(self, m, reason: str = "DOCK_FAILED") -> None:
        """도킹 시도 하나가 실패했다: 실패한 결과이거나 거절이다.

        물러났다 들어오는 사이에 지워지는 attempts 대신 따로 센다. 실패한 결과 뒤에는 물러났다
        다시 들어온다. 거절은 로봇이 받지 않은 것이므로 그 자리에서 다시 낸다.
        """
        m.dock_tries += 1
        m.attempts = m.dock_tries - 1
        if reason == "DOCK_FAILED":
            m.redock, m.redock_since = True, None
        self._goal_failed(m, reason)

    def _reapproach(self, m) -> bool:
        """도크 앞 지점까지 후진했다가 다시 들어오는 길을 잡는다.

        길을 잡았거나 길이 비기를 기다리는 중이면 True다. 그런 길이 없는 도크이거나, 로봇이
        도크에 서 있지 않거나, 처음 해 본 때부터 redock_wait_s 안에 못 잡았으면 False다: 그
        자리에서 다시 도킹한다. 도크 차례는 dock_tries가 지킨다(turns.py의 _ahead).
        """
        now = self._clock()
        dock = self.site.stations[m.dest].dock
        out = next((link for link in self.site.links_from(dock) if link.via == RETREAT), None)
        back = self.site.link(out.b, dock) if out is not None else None
        if m.redock_since is None:
            m.redock_since = now
        if (back is None or back.via != FOLLOW_PATH or m.node != dock
                or now - m.redock_since >= self.params.redock_wait_s):
            m.redock = False
            self._stop_wanting(m)
            return False
        trip = Trip(dock, [out, back])
        want = self._take(m, trip)
        if want is not None:
            # 나가는 길을 다른 로봇이 쥐고 있다. 도크에 선 채 기다린다. 대기 순환에는 세지
            # 않는다(watch.py): 순환으로 보이면 이 로봇이 주차 자리로 보내진다
            self._wants[m.name] = want
            self._note_wait(m, want)
            return True
        m.redock = False
        m.trip, m.phase = trip, self._leg_of(m.dest)
        self._event("INFO", "DOCK_REAPPROACH", f"backs to {out.b} and comes in to {dock} again",
                    m.name)
        return True
