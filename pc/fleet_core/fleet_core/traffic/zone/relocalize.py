# ---- 격리 전 위치 다시 찾기 믹스인(relocalize) ----
from __future__ import annotations

import math

from fleet_core.common.commands import CancelCmd, RelocalizeCmd
from fleet_core.traffic.mission import DOCKING, LOCALIZING, NOT_AT_TARGET, ON_A_GOAL

from .robots import Reloc

RELOCALIZE_SENT = "RELOCALIZE_SENT"
RELOCALIZED = "RELOCALIZED"
RELOCALIZE_FAILED = "RELOCALIZE_FAILED"
DOCK_FAILED = "DOCK_FAILED"
# 위치 탓일 수 있는 실패: 이것으로 격리하기 전에 다시 찾는다
RELOCALIZE_FOR = (NOT_AT_TARGET, DOCK_FAILED)
# 로봇이 시간 안에 답하지 않으면 이만큼 더 기다린 뒤 실패로 본다
REPLY_MARGIN_S = 5.0
# 액션 서버(NOT_SENT, fleet_manager가 붙임)나 위치 추정기가 없다: 예전 길로 돌아간다
UNAVAILABLE = ("NOT_SENT", "NO_LOCALIZER")
# 로봇이 아직 움직이는 중이라 받자마자 끝냈다: 실패가 아니다. 서면 시간 안에 다시 보낸다
BUSY = "BUSY"
REFUSED = "LOCALIZING"


class Relocalize:
    # ---- 시작: 쥔 구역을 그대로 두고 goal을 거둔다. 보내기는 선 뒤에 ----
    def _relocalize_first(self, m, why: str, before, late: bool = False) -> bool:
        """시작했으면 True. late는 재시도를 다 쓴 뒤(격리 직전)다."""
        if (not self.params.relocalize or not m.live or m.comm or m.reloc is not None
                or m.relocs >= (2 if late else 1)):
            return False
        if m.goal is not None:
            self._cancel_goal(m)
        self._stop_wanting(m)
        m.reloc = Reloc(why, late, tuple(before))
        m.relocs += 1
        return True

    # ---- 틱마다: 선 로봇에게 보내기, 시간 넘김 ----
    def _watch_relocalize(self, m, now: float) -> None:
        r = m.reloc
        if r is None:
            return
        if not m.live:
            if r.goal_id:
                self._out.append(CancelCmd(m.name, r.goal_id))
            m.reloc = None
        elif not r.goal_id:
            if (not m.comm and m.goal is None and now >= m.settle_until
                    and m.robot_state not in ON_A_GOAL):
                self._send_relocalize(m, now)
        elif now - r.since >= self.params.relocalize_timeout_s + REPLY_MARGIN_S:
            self._out.append(CancelCmd(m.name, r.goal_id))
            self._relocalize_failed(m, "TIMEOUT")

    def _send_relocalize(self, m, now: float) -> None:
        r, p = m.reloc, self.params
        if not r.busy:
            r.attempt, r.since = r.attempt + 1, now
        r.busy = False
        r.goal_id = self._next_goal_id()
        r.sent_at, r.xy = now, (m.x, m.y)
        wide = r.attempt >= 2  # 두 번째부터 전역 재초기화
        self._out.append(RelocalizeCmd(m.name, r.goal_id, True, wide, p.relocalize_timeout_s))
        m.robot_state = LOCALIZING  # 다음 보고까지
        self._event("WARN", RELOCALIZE_SENT, f"{r.why}: attempt {r.attempt}, spin"
                    f"{', global' if wide else ''}, keeps {list(self.grants.held_by(m.name))}",
                    m.name)

    # ---- 결과 받기 ----
    def on_relocalize_result(self, robot, goal_id, success, reason="", pose=None,
                             start=None) -> None:
        m = self._robots.get(robot)
        if m is None or m.reloc is None or m.reloc.goal_id != goal_id:
            return
        if success and pose is None:
            success, reason = False, "no valid pose"  # pose_valid 거짓은 실패
        if not success and reason == BUSY:
            self._relocalize_busy(m)
            return
        if not success:
            self._relocalize_failed(m, reason or "FAILED")
            return
        r, m.reloc = m.reloc, None
        x, y, yaw = (float(v) for v in pose[:3])
        shift = math.dist(tuple(start[:2]) if start is not None else r.xy, (x, y))
        m.x, m.y, m.yaw = x, y, yaw
        m.pose_at = None  # 다시 찾은 위치로 튐을 재지 않는다
        m.outside_n, m.outside_noted = 0, False
        m.clear_n.clear()
        m.attempts, m.dock_tries = r.before  # 다시 찾기는 재시도 수를 쓰지 않는다
        m.retry_at = self._clock()
        self._replace(m, r, shift)

    def _relocalize_busy(self, m) -> None:
        r, now = m.reloc, self._clock()
        if now - r.since >= self.params.relocalize_timeout_s:
            self._relocalize_failed(m, "TIMEOUT (busy)")
            return
        r.goal_id, r.busy = 0, True
        m.settle_until = now + self.params.arrival_settle_s

    def _relocalize_failed(self, m, reason: str) -> None:
        r = m.reloc
        self._event("WARN", RELOCALIZE_FAILED, f"{r.why}: attempt {r.attempt}, {reason}", m.name)
        if reason in UNAVAILABLE:
            # 다시 찾을 수 없는 로봇: 이번 실패 줄에서는 다시 묻지 않고 예전 길로
            m.reloc, m.relocs = None, 2
            if r.late:
                self._event("ERROR", "GOAL_FAILED", f"{r.why} after {m.attempts} attempts",
                            m.name)
                self._fail(m, r.why)
            else:
                self._goal_failed(m, r.why, r.before)
            return
        if r.attempt < self.params.relocalize_attempts:
            r.goal_id = 0
            m.settle_until = self._clock() + self.params.arrival_settle_s
            return
        m.reloc = None
        self._fail(m, r.why)

    # ---- 성공 뒤 새 위치로 다시 놓기 ----
    def _replace(self, m, r, shift: float) -> None:
        site, held = self.site, list(self.grants.held_by(m.name))
        where = f"moved {shift:.2f} m to ({m.x:.2f}, {m.y:.2f})"
        if any(site.distance_to_zone(m.x, m.y, zone) <= self.params.outside_zone_m
               for zone in held):
            self._clear_doubt(m)
            self._event("INFO", RELOCALIZED, f"{where}, inside {held}", m.name)
            if r.why == DOCK_FAILED and m.phase == DOCKING:
                self._back_to_dock(m, held)
            return
        zone = site.zone_at(m.x, m.y)
        if zone is None or self.grants.holders(zone) or not self.grants.force(m.name, [zone]):
            self._event("WARN", RELOCALIZED, f"{where}, outside {held}, "
                        f"{zone or 'no zone'} is not free", m.name)
            self._fail(m, r.why)
            return
        # 선 자리를 쥐었다: 그 구역의 노드에서 길을 새로 짠다
        self._clear_doubt(m)
        self._event("WARN", RELOCALIZED, f"{where}, outside {held}, takes {zone} where it stands",
                    m.name)
        m.node = site.nearest_node(m.x, m.y, site.nodes_in(zone)) or m.node
        m.trip, m.redock = None, False
        if m.phase == DOCKING:
            m.phase = self._leg_of(m.dest)

    def _back_to_dock(self, m, held) -> None:
        """도크에서 먼 로봇은 제자리 도킹 대신 도크까지 다시 간다."""
        dock = self.site.stations[m.dest].dock
        if self._gap(m, dock) <= self.params.arrival_tolerance_m:
            return  # 도크 위: 예전처럼 물러났다 다시 들어오거나 제자리 도킹
        inside = [node for zone in held for node in self.site.nodes_in(zone) if node != dock]
        m.node = self.site.nearest_node(m.x, m.y, inside) or m.node
        m.trip, m.redock = None, False
        m.phase = self._leg_of(m.dest)
