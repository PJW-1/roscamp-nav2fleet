"""관제가 낸 것을 세계에 옮긴다: 이동, 물러나기, 도킹, 취소, 팔 작업, 주문 보고, 사건."""
from __future__ import annotations

import math

from fleet_core.common.commands import (
    CancelCmd,
    DockCmd,
    EventOut,
    FillCmd,
    MoveCmd,
    ReportOrderCmd,
    RetreatCmd,
    SwapCmd,
)

from .avoid import Avoidance
from .invariants import COMMAND, Violation
from .nav_goal import NavGoalRun
from .runs import DockRun, FollowRun, Result, RetreatRun

DOCK_NEAR_M = 0.05  # 도킹 명령은 도크 지점에서 이 안일 때만 받는다


class Commands:
    """ZoneWorld에 섞는다. self.robots, self.manager, self.params, self.queued 등을 쓴다."""

    def send(self, item) -> None:
        """관제의 출력 하나를 세계에 넣는다. 시험이 명령을 직접 줄 때도 쓴다."""
        r = self.robots.get(getattr(item, "robot", ""))
        self.metrics.on_output(item, docked=r is not None and bool(r.docked_at))
        if isinstance(item, EventOut):
            self.events.append((self.t, item))
            self.metrics.on_event(item)
            return
        self.commands.append((self.t, item))
        if isinstance(item, ReportOrderCmd):
            self.central.take_report(self.t, item)
        elif isinstance(item, FillCmd):
            self.arms.start_fill(self.t, item, self.robots[item.robot],
                                 self._slow(item.station_id))
        elif isinstance(item, SwapCmd):
            self.arms.start_swap(self.t, item, self.robots[item.robot],
                                 self._slow(item.station_id))
        elif isinstance(item, CancelCmd):
            self._cancel(item)
        elif isinstance(item, (MoveCmd, RetreatCmd, DockCmd)):
            self._start(item)
        else:
            self._violation(f"unknown output {item!r}")

    def _violation(self, detail, robot="") -> None:
        self.invariants.violations.append(Violation(self.t, COMMAND, detail, robot))

    def _slow(self, station: str) -> float:
        return self.schedule.slow_factor(station, self.t)

    def _reachable(self, name) -> bool:
        r = self.robots.get(name)
        return r is not None and r.comm_up and not r.removed

    def _preempt(self, r) -> None:
        """바뀌는 goal도 ROS 액션처럼 결과를 낸다(CANCELED, 다음 걸음에)."""
        if r.run is not None:
            self.queued.append((r.name, Result(r.run.kind, r.run.goal_id, False, "CANCELED")))
            r.run = None

    def _start(self, cmd) -> None:
        if not self._reachable(cmd.robot):
            return
        r = self.robots[cmd.robot]
        grants = self.grants()
        if isinstance(cmd, MoveCmd):
            self.invariants.check_command(self.t, grants, cmd, cmd.mode, (r.x, r.y))
            run = self._move_run(r, cmd)
        elif isinstance(cmd, RetreatCmd):
            self.invariants.check_command(self.t, grants, cmd, "retreat", (r.x, r.y, r.yaw))
            run = RetreatRun(cmd.goal_id, cmd.path, cmd.distance_m, cmd.speed_mps,
                             self.params.lane_error_warn_m)
        else:
            run = self._dock_run(r, cmd)
        self._preempt(r)
        if not isinstance(cmd, DockCmd):
            r.docked_at = ""
        r.run = run

    def _move_run(self, r, cmd):
        speed_factor = r.profile.speed_factor
        if cmd.mode == "nav_goal":
            return NavGoalRun(cmd.goal_id, (cmd.x, cmd.y), cmd.yaw,
                              self.params.nav_speed_mps * speed_factor, self.blocked_after_s,
                              Avoidance(self.avoid_mode, self.avoid_settings))
        return FollowRun(cmd.goal_id, cmd.path, (cmd.x, cmd.y), cmd.yaw,
                         self.params.follow_speed_mps * speed_factor,
                         self.params.lane_error_warn_m, turn_at_end=cmd.final_goal)

    def _dock_run(self, r, cmd):
        station = self.site.stations.get(cmd.dock_id)
        near = station is not None and math.dist(
            r.pose, self.site.node_pose(station.dock)) <= DOCK_NEAR_M
        if not near:
            self._violation(f"{cmd} but {r.name} is at ({r.x:.3f}, {r.y:.3f})", r.name)
        return DockRun(cmd.goal_id, cmd.dock_id, self.dock_s, fail=not near)

    def _cancel(self, cmd) -> None:
        if cmd.robot not in self.robots:
            job = self.arms.cancel(cmd.robot, cmd.goal_id)
            if job is not None:
                self.queued_arm.append((job, "CANCELED"))
            return
        r = self.robots[cmd.robot]
        if self._reachable(cmd.robot) and r.run is not None and r.run.goal_id == cmd.goal_id:
            self._preempt(r)
