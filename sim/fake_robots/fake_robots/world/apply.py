"""관제가 낸 것을 세계에 옮긴다: 이동, 물러나기, 도킹, 취소, 팔 작업, 주문 보고, 사건."""
from __future__ import annotations

import math

from fleet_core.common.commands import (
    CancelCmd,
    DockCmd,
    EventOut,
    FillCmd,
    MoveCmd,
    MoveOrderCmd,
    ReportOrderCmd,
    RetreatCmd,
    SwapCmd,
)

from .avoid import Avoidance
from .invariants import COMMAND, Violation
from .nav_goal import NavGoalRun
from .order_run import OrderRun, refusal
from .runs import MOVE, DockRun, FollowRun, Result, RetreatRun

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
            run = r.run if r is not None else None
            self.reaction.on_event(self.t, item, run.goal_id if run is not None else None)
            return
        self.commands.append((self.t, item))
        self.reaction.on_command(self.t, item)
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
        elif isinstance(item, MoveOrderCmd):
            self._order(item)
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
            self.queued.append((r.name, Result(r.run.kind, r.run.goal_id, False, "CANCELED",
                                               getattr(r.run, "done_seq", None))))
            r.run = None

    # ---- order 모델 ----

    def _order(self, cmd) -> None:
        """order goal(update 0)은 하던 goal을 바꾼다. update는 이음 조건 넷이 맞을 때만 붙이고,
        붙이면 이전 goal을 SUPERSEDED로, 아니면 새 goal을 ORDER_UPDATE_REJECTED로 끝낸다.
        답은 다음 걸음에 간다."""
        if not self._reachable(cmd.robot):
            return
        r = self.robots[cmd.robot]
        run = r.run if isinstance(r.run, OrderRun) else None
        if cmd.update_id == 0:
            self._check_segments(r, cmd)
            self._preempt(r)
            r.docked_at = ""
            r.run = OrderRun(cmd, lambda segment, final, goal_id: self._segment_run(
                r, segment, final, goal_id))
            self.order_metrics.run_started(r.name, r.run, r)
            return
        why = refusal(run, cmd)
        done = run.done_seq if run is not None else 0
        if why:
            self.queued.append((r.name, Result(MOVE, cmd.goal_id, False,
                                               f"ORDER_UPDATE_REJECTED: {why}", done)))
            return
        self._check_segments(r, cmd)
        old = run.stitch(cmd)
        self.order_metrics.base_given(r.name, cmd.base)
        self.queued.append((r.name, Result(MOVE, old, False, "SUPERSEDED", done)))

    def _check_segments(self, r, cmd) -> None:
        """4번 불변식: 새 base 조각마다 받은 때 쥔 구역 안인가."""
        grants = self.grants()
        start = (r.x, r.y, r.yaw)
        for segment in cmd.base:
            mode = segment.mode
            shaped = MoveCmd(cmd.robot, cmd.goal_id, segment.end_node, segment.x, segment.y,
                             segment.yaw, path=segment.path, mode=mode)
            if mode == "retreat":
                shaped = RetreatCmd(cmd.robot, cmd.goal_id, 0.0, 0.0, segment.path)
            self.invariants.check_command(self.t, grants, shaped, mode, start)
            start = (segment.x, segment.y, segment.yaw)

    def _segment_run(self, r, segment, final: bool, goal_id: int):
        speed_factor = r.profile.speed_factor
        if segment.mode == "nav_goal":
            return NavGoalRun(goal_id, (segment.x, segment.y), segment.yaw,
                              self.params.nav_speed_mps * speed_factor, self.blocked_after_s)
        if segment.mode == "retreat":
            return RetreatRun(goal_id, segment.path, 0.0, self.params.retreat_speed_mps,
                              self.params.lane_error_warn_m)
        return FollowRun(goal_id, segment.path, (segment.x, segment.y), segment.yaw,
                         self.params.follow_speed_mps * speed_factor,
                         self.params.lane_error_warn_m, turn_at_end=final)

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
        self.order_metrics.run_started(r.name, run, r)

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
