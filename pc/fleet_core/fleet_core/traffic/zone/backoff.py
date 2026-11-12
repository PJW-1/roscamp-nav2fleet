"""마당 물러나기(제안, 기본 꺼짐): 넓은 구역에서 비켜 서는 로봇을 선 방향 그대로 뒤로 조금 물린다.

몸이 닿을 만큼 붙은 두 대는 한쪽을 세우기만 해서는 떨어지지 않았다(Gazebo 3대 경기: 곁에 로봇이
있으면 Nav2의 Spin과 BackUp이 돌지 못했다). 로봇의 회피가 안 풀려 그 로봇이 BLOCKED를 보고했을
때만 낸다(watch._watch_near). 명령은 path가 있는 RetreatCmd다: 로봇은 그 선을 후진 FollowPath로
따른다. 실패해도 시도로 세지 않는다: 잠시 서 있다가 가던 길을 다시 받는다.
"""
from __future__ import annotations

import math

from fleet_core.common.commands import RetreatCmd
from fleet_core.fleet_map import RETREAT, SHARED
from fleet_core.fleet_map.geometry import (
    point_rect_distance,
    point_segment_distance,
    sample,
    segment_rect_distance,
    wall_gap,
)
from fleet_core.traffic.mission import BLOCKED

from .robots import BACKOFF, Goal

NO_ROOM = "no room behind"
WAY_STEP_M = 0.02  # 가는 선을 이 간격으로 짚어 구역 안인지 본다


def backoff_spot(site, params, held, m, others, taken=()) -> tuple[tuple | None, str]:
    """m이 뒤로 물러나 설 자리 (x, y)와, 못 물러나는 까닭(물러날 수 있으면 빈 문자열).

    설 자리는 m이 서 있고 쥐고 있는 넓은 구역 안이어야 하고, 벽과 진열대와 고정 경로 구역에서
    yard_backoff_edge_m 이상 떨어져야 한다. 가는 선은 그 구역을 벗어나거나 진열대를 지나지 않아야
    하고, 다른 로봇에게 다가가지 않거나, 다가가더라도 retreat_clear_m 안으로 들지 않아야
    한다. others는 바닥에 있는 다른 로봇들이다. taken은 다른 로봇이 잡아 둔 지점들이다: 설 자리가
    그 지점에서 node_clear_m 안이면 그 로봇이 올 자리를 막으므로 물러나지 않는다.
    """
    p = params
    zone = site.zone_at(m.x, m.y)
    if zone is None or site.zones[zone].kind != SHARED or zone not in held:
        return None, "not in a yard it holds"
    way = (-math.cos(m.yaw), -math.sin(m.yaw))
    here = (m.x, m.y)
    end = (m.x + way[0] * p.yard_backoff_m, m.y + way[1] * p.yard_backoff_m)
    edge = p.yard_backoff_edge_m
    fixed = [z.id for z in site.zones.values() if z.kind != SHARED]
    if (any(site.zone_at(x, y) != zone for x, y in sample(here, end, WAY_STEP_M))
            or wall_gap(site.bounds(), *end) < edge
            or any(site.distance_to_zone(*end, z) < edge for z in fixed)
            or any(point_rect_distance(shelf, *end) < edge
                   or segment_rect_distance(here, end, shelf) <= 0.0 for shelf in site.shelves)):
        return None, NO_ROOM
    for other in others:
        nearer = (m.x - other.x) * way[0] + (m.y - other.y) * way[1] < 0.0
        gap = point_segment_distance((other.x, other.y), here, end)
        if nearer and gap < p.retreat_clear_m:
            return None, f"{other.name} behind"
    for node in taken:
        if math.dist(end, site.node_pose(node)) < p.node_clear_m:
            return None, f"{node} is taken"
    return end, ""


class Backoff:
    def _awaits_blocked(self, m, stuck_s: float) -> bool:
        """물러나기가 켜져 있으면 비킬 로봇의 BLOCKED 보고를 조금 더 기다린다.

        로봇은 제 걸음이 멈춘 때부터, 관제는 둘 가운데 누구든 마지막으로 나아간 때부터 잰다.
        늦게 선 쪽의 BLOCKED는 관제의 near_stuck_s와 거의 같은 때에 온다.
        """
        p = self.params
        return (p.yard_backoff and m.robot_state != BLOCKED
                and stuck_s < p.near_stuck_s + p.yard_backoff_wait_s)

    def _stand_aside(self, m, other, now: float) -> None:
        """goal을 거둔 m을 비키게 하고 HALL_YIELD를 낸다. 순서는 추가 지시서 2의 4절이다.

        처음에는 세운다. 물러나기가 켜져 있고 m이 BLOCKED를 보고했으면 세우기 전에 뒤로 물린다:
        그때는 물러나기가 끝난 뒤부터 서 있는 시간을 잰다(_backoff_ended). 같은 자리에서 곧 또
        비키게 되면 가까운 빈 주차 자리로 보낸다. 갈 주차 자리가 없으면 다시 세운다.
        """
        p = self.params
        again = (m.yielded is not None and now - m.yielded[2] <= p.blocked_retry_s
                 and math.dist(m.yielded[:2], (m.x, m.y)) < p.near_m)
        m.yielded = (m.x, m.y, now)
        if again and self._to_parking(m, "INFO", "HALL_YIELD",
                                      f"out of the way of {other.name}, goes to", clear_of=other):
            m.yielded = None  # 다음에 붙으면 세우기부터 다시 한다
            return
        stops = f"stops {p.near_stuck_s:.0f} s for {other.name}"
        if p.yard_backoff and m.robot_state == BLOCKED:
            why = self._back_off(m)
            if not why:
                self._event("INFO", "HALL_YIELD", f"backs {p.yard_backoff_m:.2f} m, then {stops}",
                            m.name)
                return
            stops += f" (no back-off: {why})"
        m.yield_until = now + p.near_stuck_s
        self._event("INFO", "HALL_YIELD", stops, m.name)

    def _back_off(self, m) -> str:
        """goal이 없는 m에게 물러나기를 낸다. 냈으면 빈 문자열, 못 냈으면 그 까닭이다."""
        p = self.params
        others = [other for other in self._robots.values() if other is not m and other.placed]
        taken = [node for node in self.site.nodes
                 if self.grants.node_holder(node) not in ("", m.name)]
        end, why = backoff_spot(self.site, p, self.grants.held_by(m.name), m, others, taken)
        if end is None:
            return why
        goal_id = self._next_goal_id()
        path = ((m.x, m.y, m.yaw), (*end, m.yaw))  # 로봇은 가는 쪽의 반대를 본다
        self._out.append(RetreatCmd(m.name, goal_id, p.yard_backoff_m, p.retreat_speed_mps,
                                    path))
        # 가던 길의 연결은 하나도 지나지 않는다: 끝나면 남은 길을 그대로 다시 받는다
        m.goal = Goal(BACKOFF, goal_id, m.trip.done if m.trip is not None else 0, "", RETREAT)
        m.backoff_from = (m.x, m.y)
        m.stall_from = None
        return ""

    def _backoff_result(self, robot, goal_id, reason: str) -> bool:
        """이 결과가 물러나기 goal의 것이면 끝내고 True. reason이 비면 성공이다."""
        m = self._active(robot, goal_id, BACKOFF)
        if m is None:
            return False
        m.goal = None
        self._backoff_ended(m, reason)
        return True

    def _backoff_ended(self, m, reason: str) -> None:
        """물러나기가 끝났고 goal은 비었다. 성공이든 아니든 잠시 서 있다가 가던 길을 다시 받는다."""
        moved = math.dist(m.backoff_from, (m.x, m.y))
        m.backoff_from = None
        m.yield_until = self._clock() + self.params.near_stuck_s
        if reason:
            self._event("WARN", "HALL_BACKOFF_FAILED", f"{reason} after {moved:.2f} m", m.name)
        else:
            self._event("INFO", "HALL_BACKOFF_DONE", f"backed {moved:.2f} m", m.name)
