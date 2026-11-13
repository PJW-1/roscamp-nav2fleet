# ---- FleetManager: 세 코어를 만들고 입력을 넣고 명령을 전달하는 매니저 ----
from __future__ import annotations

import math

from fleet_core.arm.jobs import ArmJobs
from fleet_core.common.commands import (
    FILL,
    LOCAL_YIELD_STEPS,
    OBSTACLE_KINDS,
    SWAP,
    EventOut,
)
from fleet_core.common.validate import (
    checked,
    obstacle_of,
    pose_problem,
    reported_id_problem,
    robot_problem,
    run_mode_problem,
    text,
)
from fleet_core.manager_record import ManagerRecord
from fleet_core.manager_route import ManagerRoute
from fleet_core.manager_views import ManagerViews
from fleet_core.orders.orchestrator import OrderOrchestrator
from fleet_core.orders.validate import orders_problem, reinstate_problem
from fleet_core.traffic.mission import LOCALIZING
from fleet_core.traffic.modes import build_traffic


# ---- FleetManager 생성: 교통, 주문, 팔 작업 코어 구성 ----
class FleetManager(ManagerRoute, ManagerViews, ManagerRecord):
    def __init__(self, site, params, robots, clock):
        self.site = site
        self.params = params
        self._clock = clock
        self._time = clock()
        self._write = None
        self._fed = False
        self._goal_seq = 0
        self.traffic = build_traffic(site, params, robots, self._frozen, self._next_goal_id)
        self.orchestrator = OrderOrchestrator(site.stations, params, robots, self._frozen)
        self.arm = ArmJobs(site.stations, params, robots, self._frozen, self._next_goal_id)
        self._work = next(s.id for s in site.stations.values() if s.action == FILL)
        if hasattr(self.traffic, "station_left"):
            self.traffic.station_left = self.station_left_s
        self._outbound = next(s.id for s in site.stations.values() if s.action == SWAP)
        self._out: list = []
        self._bounds = site.bounds()
        self._board: list = []
        self._rejecting: set = set()
        self.last_clear_reason = ""
        self.last_reinstate_reason = ""

    # ---- 스테이션 작업 남은 시간 추정과 읽기 전용 속성 ----
    def station_left_s(self, robot) -> float | None:
        job = self.orchestrator.station_job(robot)
        if job is None:
            return None
        kind, items = job
        sent = self.arm.sent_at(robot)
        p = self.params
        total = (p.eta_fill_base_s + items * p.eta_fill_s_per_item if kind == FILL
                 else p.eta_swap_s)
        if sent is None:
            return total
        return max(1.0, total - (self._frozen() - sent))

    @property
    def zones(self):
        return self.traffic.zones

    @property
    def shipped(self) -> int:
        return self.orchestrator.shipped

    # ---- 로봇 상태 입력 검사 후 교통 코어에 전달 ----
    def on_robot_state(self, robot, x, y, yaw, state, battery=0.0, robot_id="",
                       obstacle_x=math.nan, obstacle_y=math.nan, obstacle_kind="",
                       local_yield="") -> None:
        no_id = isinstance(robot_id, str) and text(robot_id) == ""
        seen = [obstacle_x, obstacle_y, obstacle_kind]
        no_obstacle = all(isinstance(v, float) and math.isnan(v) for v in seen[:2]) and (
            isinstance(obstacle_kind, str) and text(obstacle_kind) == "")
        no_step = isinstance(local_yield, str) and text(local_yield) == ""
        tail = [] if no_obstacle and no_step else [robot_id, *seen]
        if not no_step:
            tail.append(local_yield)
        self._input("on_robot_state", robot, x, y, yaw, state, battery,
                    *(tail or ([] if no_id else [robot_id])))
        localizing = isinstance(state, str) and text(state) == LOCALIZING
        problem = checked(lambda: robot_problem(robot, self.traffic.robots)
                          or reported_id_problem(robot, robot_id)
                          or ("" if localizing else pose_problem(
                              x, y, yaw, self._bounds, self.params.input_pose_margin_m)))
        if self._topic_rejected(("robot_state", text(robot)), problem, "robot state dropped",
                                robot):
            return
        try:
            obstacle = obstacle_of(obstacle_x, obstacle_y, obstacle_kind, self._bounds,
                                   self.params.input_pose_margin_m, OBSTACLE_KINDS)
        except Exception:
            obstacle = None
        step = text(local_yield) if isinstance(local_yield, str) else ""
        self.traffic.on_robot_state(robot, x, y, yaw, text(state), battery, obstacle,
                                    step if step in LOCAL_YIELD_STEPS else "")
        self._pump()

    # ---- 막힘 지점 확인 답, 주문판, 이동, 도킹 등 외부 입력을 받아 코어에 전달 ----
    def on_clear_check(self, robot, check_id, answer) -> None:
        self._input("on_clear_check", robot, check_id, answer)
        if not self._robot_refused("clear check", robot):
            self.traffic.on_clear_check(robot, check_id, text(answer))
            self._pump()

    def on_order_board(self, run_mode, orders) -> None:
        self._input("on_order_board", run_mode, orders)
        mode_problem = checked(run_mode_problem, run_mode)
        if mode_problem:
            self._topic_rejected(("order_board",), mode_problem, "order board dropped")
            return
        if not self._topic_rejected(("order_board",), checked(orders_problem, orders),
                                    f"run_mode {text(run_mode)} taken, orders kept as they were"):
            self._board = list(orders)
        self.traffic.on_run_mode(run_mode)
        self.arm.on_run_mode(run_mode)
        self.orchestrator.on_order_board(run_mode, self._board)
        self._pump()

    def on_move_result(self, robot, goal_id, success, reason, last_done_seq=0) -> None:
        self._input("on_move_result", robot, goal_id, success, reason, last_done_seq)
        if not self._robot_refused("move result", robot):
            self.traffic.on_move_result(robot, goal_id, success, reason, last_done_seq)
            self._pump()

    def on_move_feedback(self, robot, goal_id, last_done_seq, waiting_at_base_end,
                         horizon_warning="") -> None:
        self._input("on_move_feedback", robot, goal_id, last_done_seq, waiting_at_base_end,
                    horizon_warning)
        if not self._robot_refused("move feedback", robot):
            self.traffic.on_move_feedback(robot, goal_id, last_done_seq, waiting_at_base_end,
                                          horizon_warning)
            self._pump()

    def on_goal_rejected(self, robot, goal_id) -> None:
        self._input("on_goal_rejected", robot, goal_id)
        if not self._robot_refused("goal rejection", robot):
            self.traffic.on_goal_rejected(robot, goal_id)
            self._pump()

    def on_goal_not_sent(self, robot, goal_id) -> None:
        self._input("on_goal_not_sent", robot, goal_id)
        if not self._robot_refused("goal not sent", robot):
            self.traffic.on_goal_not_sent(robot, goal_id)
            self._pump()

    def on_goal_lost(self, robot, goal_id) -> None:
        self._input("on_goal_lost", robot, goal_id)
        if not self._robot_refused("goal lost", robot):
            self.traffic.on_goal_lost(robot, goal_id)
            self._pump()

    def on_dock_result(self, robot, goal_id, success, error_x_cm=0.0, error_y_cm=0.0,
                       error_yaw_deg=0.0) -> None:
        self._input("on_dock_result", robot, goal_id, success, error_x_cm, error_y_cm,
                    error_yaw_deg)
        if not self._robot_refused("dock result", robot):
            self.traffic.on_dock_result(robot, goal_id, success, error_x_cm, error_y_cm,
                                        error_yaw_deg)
            self._pump()

    def on_retreat_result(self, robot, goal_id, success, reason="") -> None:
        self._input("on_retreat_result", robot, goal_id, success, reason)
        if not self._robot_refused("retreat result", robot):
            self.traffic.on_retreat_result(robot, goal_id, success, reason)
            self._pump()

    # ---- 팔 작업 결과 입력 전달 ----
    def on_fill_result(self, goal_id, success, placed, missing, reason="") -> None:
        self._input("on_fill_result", goal_id, success, placed, missing, reason)
        self.arm.on_fill_result(goal_id, success, placed, missing, reason)
        self._pump()

    def on_swap_result(self, goal_id, success, took_full, put_empty_done, reason="") -> None:
        self._input("on_swap_result", goal_id, success, took_full, put_empty_done, reason)
        self.arm.on_swap_result(goal_id, success, took_full, put_empty_done, reason)
        self._pump()

    # ---- 운영자 조작: 치운 로봇의 구역 풀기와 다시 투입 ----
    def confirm_clear(self, robot, zone_ids, note="") -> list[str] | None:
        self._input("confirm_clear", robot, zone_ids, note)
        released = self.traffic.confirm_clear(robot, zone_ids, note)
        self.last_clear_reason = self.traffic.clear_reason
        if released is not None:
            self.orchestrator.on_robot_removed(robot)
        self._pump()
        return released

    def reinstate(self, robot, basket, note="") -> bool:
        self._input("reinstate", robot, basket, note)
        problem = checked(lambda: reinstate_problem(robot, basket, self.traffic.robots)
                          or self.traffic.reinstate_refusal(text(robot)))
        self.last_reinstate_reason = problem
        if problem:
            self._reject(f"reinstate refused: {problem}: {text(note)}", robot)
            return False
        name, basket = text(robot), text(basket)
        self.traffic.reinstate(name)
        self.orchestrator.on_robot_reinstated(name, basket)
        self._out.append(EventOut("INFO", "ROBOT_REINSTATED",
                                  f"basket {basket}, on the map at its next report: {text(note)}",
                                  name))
        self._pump()
        return True

    # ---- 주기 처리: 코어를 순서대로 진행하고 나온 출력 반환 ----
    def tick(self) -> list:
        self._input("tick")
        traffic = self.traffic
        now = self._time
        everyone_here = traffic.tick_begin(now)
        self._pump()
        self.arm.check_timeouts(now)
        self._pump()
        stopped = traffic.tick_progress(now)
        self._pump()
        if traffic.running and everyone_here:
            traffic.tick_pick(stopped)
            self._pump()
            for robot in traffic.robots:
                if traffic.available(robot) and robot not in stopped:
                    self._step_robot(robot)
        out, self._out = self._out, []
        for item in out:
            self._note("out", item)
        return out

    # ---- 현재 시각과 목표 번호 발급 ----
    def _frozen(self) -> float:
        return self._time

    def _next_goal_id(self) -> int:
        self._goal_seq += 1
        return self._goal_seq
