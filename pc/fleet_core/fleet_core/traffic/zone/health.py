# ---- 시작 보고, 통신 끊김, 운영자 치우기 믹스인 ----
from __future__ import annotations

from fleet_core.common.commands import DispatchResult
from fleet_core.common.validate import checked, clear_problem, text
from fleet_core.traffic.mission import COMM_LOST, LOCALIZING, OUT_OF_SERVICE, RECOVERING

from .robots import Robot


# ---- 보고하지 않는 로봇과 통신 끊김 감지 ----
class Health:
    def _check_startup(self, now: float) -> bool:
        if self._started is None:
            self._started = now
        missing = [m.name for m in self._robots.values()
                   if not m.placed and m.fault != OUT_OF_SERVICE and m.name not in self._joining]
        if (missing and not self._startup_warned
                and now - self._started > self.params.startup_wait_s):
            self._startup_warned = True
            self._event("WARN", "ROBOT_NOT_REPORTING", f"no usable report from {missing}")
        return not missing

    def _check_comm(self, now: float) -> None:
        for m in self._robots.values():
            if (m.placed and not m.fault and m.comm != COMM_LOST
                    and now - m.last_seen > self.params.comm_timeout_s):
                m.comm = COMM_LOST
                m.called = False
                self._stop_wanting(m)
                self._event("WARN", "COMM_LOST", f"no state for {now - m.last_seen:.1f} s, keeps "
                            f"{list(self.grants.held_by(m.name))}{self._hold_base(m, now)}",
                            m.name)

    # ---- 통신 복구 뒤 위치 확인과 트립 재개 ----
    def _begin_recovery(self, m) -> None:
        m.comm = RECOVERING
        m.recover_at = self._clock() + self.params.recover_check_s
        if m.goal:
            self._cancel_goal(m)
        self._event("INFO", "COMM_RESTORED", "checks where it stands", m.name)

    def _finish_recoveries(self, now: float) -> None:
        fresh = self.params.recover_check_s / 2
        for m in self._robots.values():
            if (m.comm == RECOVERING and now >= m.recover_at and m.robot_state != LOCALIZING
                    and now - m.last_seen <= fresh):
                self._resume(m)

    def _resume(self, m) -> None:
        m.comm = ""
        m.stall_from = None
        trip = m.trip
        if trip is None:
            return
        tolerance = self.params.arrival_tolerance_m
        for index in range(trip.granted - 1, trip.done - 1, -1):
            if self._link_reached(m, trip.links[index], tolerance):
                trip.done, m.node = index + 1, trip.links[index].b
                break
        trip.sent = trip.done

    # ---- confirm_clear 입력 검사와 거부 판단 ----
    def confirm_clear(self, robot, zone_ids, note="") -> list[str] | None:
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
        m = self._robots[robot]
        left = m.base_until - self._clock()
        if left > 0:
            refusal = f"it may still be driving to the end of its base for {left:.0f} s"
        else:
            refusal = (f"no zones listed while it holds {list(held)}"
                       if held and not zone_ids else "")
        self.clear_reason = refusal
        if refusal:
            self._event("WARN", "CLEAR_REFUSED", f"{refusal}: {note}", robot)
            return None
        # ---- 구역 놓고 OUT_OF_SERVICE로 만들기 ----
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

    # ---- 치운 로봇 다시 투입 ----
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
        if self.reinstate_refusal(robot):
            return False
        old = self._robots[robot]
        self._robots[robot] = Robot(robot, x=old.x, y=old.y, yaw=old.yaw,
                                    robot_state=old.robot_state, battery=old.battery,
                                    charge=old.charge, last_seen=old.last_seen)
        self._wants.pop(robot, None)
        self._joining.add(robot)
        return True
