"""로봇의 건강: 시작 때 보고, 통신 끊김과 복구, 운영자가 치움(confirm_clear), 다시 투입."""
from __future__ import annotations

from fleet_core.common.commands import DispatchResult
from fleet_core.common.validate import checked, clear_problem, text
from fleet_core.traffic.mission import COMM_LOST, LOCALIZING, OUT_OF_SERVICE, RECOVERING

from .robots import Robot


class Health:
    def _check_startup(self, now: float) -> bool:
        """모든 로봇이 지도에 올랐는가. 오래 안 오르면 한 번 알린다."""
        missing = [m.name for m in self._robots.values()
                   if not m.placed and m.fault != OUT_OF_SERVICE and m.name not in self._joining]
        if (missing and not self._startup_warned
                and now - self._started > self.params.startup_wait_s):
            self._startup_warned = True
            self._event("WARN", "ROBOT_NOT_REPORTING", f"no usable report from {missing}")
        return not missing

    def _check_comm(self, now: float) -> None:
        for m in self._robots.values():
            if (m.placed and not m.fault and not m.comm
                    and now - m.last_seen > self.params.comm_timeout_s):
                m.comm = COMM_LOST
                m.called = False
                self._stop_wanting(m)
                self._event("WARN", "COMM_LOST", f"no state for {now - m.last_seen:.1f} s, keeps "
                            f"{list(self.grants.held_by(m.name))}", m.name)

    def _begin_recovery(self, m) -> None:
        m.comm = RECOVERING
        m.recover_at = self._clock() + self.params.recover_check_s
        if m.goal:
            self._cancel_goal(m)
        self._event("INFO", "COMM_RESTORED", "checks where it stands", m.name)

    def _finish_recoveries(self, now: float) -> None:
        for m in self._robots.values():
            if m.comm == RECOVERING and now >= m.recover_at and m.robot_state != LOCALIZING:
                self._resume(m)

    def _resume(self, m) -> None:
        """말이 없던 동안 지난 지점을 따라잡고 가던 길을 잇는다."""
        m.comm = ""
        m.stall_from = None
        trip = m.trip
        if trip is None:
            return
        tolerance = self.params.arrival_tolerance_m
        for index in range(trip.granted - 1, trip.done - 1, -1):
            if self._gap(m, trip.links[index].b) <= tolerance:
                trip.done, m.node = index + 1, trip.links[index].b
                break
        trip.sent = trip.done

    # ---- 운영자 ----

    def confirm_clear(self, robot, zone_ids, note="") -> list[str] | None:
        """운영자가 멈춘 로봇을 치웠다. 놓은 구역을 돌려주고, 거부면 None(clear_reason에 이유)."""
        note = text(note)
        problem = checked(clear_problem, robot, zone_ids, self._robots,
                          lambda zone: zone in self.site.zones)
        if problem:
            self.clear_reason = problem
            name = text(robot) if isinstance(robot, str) else ""
            self._event("WARN", "INPUT_REJECTED", f"confirm_clear refused: {problem}: {note}",
                        name if name in self._robots else "")
            return None
        held = self.grants.held_by(robot)
        refusal = f"no zones listed while it holds {list(held)}" if held and not zone_ids else ""
        self.clear_reason = refusal
        if refusal:
            self._event("WARN", "CLEAR_REFUSED", f"{refusal}: {note}", robot)
            return None
        m = self._robots[robot]
        released = [zone for zone in zone_ids if self.grants.release_zone(robot, zone)]
        for node in self.grants.nodes_of(robot):
            self.grants.release_node(robot, node)
        self._stop_wanting(m)
        if m.goal:
            self._cancel_goal(m)
        m.trip = None
        m.clear_n.clear()
        m.fault, m.fault_reason, m.comm = OUT_OF_SERVICE, note, ""
        m.called = m.hold = False
        m.retreat_to = ""
        self._event("INFO", "ZONES_CLEARED", f"released {released}: {note}", robot)
        still = self.grants.held_by(robot)
        if still:
            self._event("WARN", "ZONES_STILL_HELD", f"{list(still)} stay locked", robot)
        if m.dispatch:
            self._out.append(DispatchResult(m.dispatch.goal_id, False, robot, "ROBOT_REMOVED"))
            m.dispatch = None
        return released

    def reinstate_refusal(self, robot) -> str:
        m = self._robots.get(robot)
        if m is None:
            return f"no such robot {robot}"
        if m.fault != OUT_OF_SERVICE:
            return f"{robot} is {m.state}, not {OUT_OF_SERVICE}"
        held = self.grants.held_by(robot)
        if held:
            return f"{robot} still holds {list(held)}, confirm_clear them first"
        return ""

    def reinstate(self, robot) -> bool:
        """치운 로봇을 다시 넣는다. 다음 위치 보고로 지도에 오른다."""
        if self.reinstate_refusal(robot):
            return False
        old = self._robots[robot]
        self._robots[robot] = Robot(robot, x=old.x, y=old.y, yaw=old.yaw,
                                    robot_state=old.robot_state, battery=old.battery,
                                    charge=old.charge, last_seen=old.last_seen)
        self._wants.pop(robot, None)
        self._joining.add(robot)
        return True
