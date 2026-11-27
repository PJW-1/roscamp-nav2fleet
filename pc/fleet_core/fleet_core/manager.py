# ---- FleetManager: 세 코어를 만들고 입력을 넣고 명령을 전달하는 매니저 ----
from __future__ import annotations

import math

from fleet_core.arm.jobs import ArmJobs
from fleet_core.common.commands import (
    FILL,
    FORCE_NOT_IMPLEMENTED,
    FORCE_UNKNOWN_ORDER,
    LOCAL_YIELD_STEPS,
    OBSTACLE_KINDS,
    REJECT_LOCK_RELEASED,
    REJECT_NOT_FOUND,
    REJECT_NOT_READY,
    REJECT_NOT_STOPPED,
    REJECT_ORDER_STATE,
    REJECT_UNKNOWN_TARGET,
    SWAP,
    EventOut,
)
from fleet_core.common.validate import (
    checked,
    clear_problem,
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
from fleet_core.orders.validate import force_ship_problem, orders_problem, reinstate_problem
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
        self._outbound = next(s.id for s in site.stations.values() if s.action == SWAP)
        self._out: list = []
        self._bounds = site.bounds()
        self._board: list = []
        self._rejecting: set = set()
        self.last_clear_reason = ""
        self.last_clear_code = ""
        self.last_reinstate_reason = ""
        self.last_reinstate_code = ""

    # ---- 읽기 전용 속성 ----
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
                         horizon_warning="", distance_remaining=None) -> None:
        distance = () if distance_remaining is None else (distance_remaining,)
        self._input("on_move_feedback", robot, goal_id, last_done_seq, waiting_at_base_end,
                    horizon_warning, *distance)
        if not self._robot_refused("move feedback", robot):
            self.traffic.on_move_feedback(robot, goal_id, last_done_seq, waiting_at_base_end,
                                          horizon_warning, distance_remaining)
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
    def on_fill_result(self, goal_id, success, placed, missing, reason="",
                       placed_confirmed=None, uncertain=None) -> None:
        self._input("on_fill_result", goal_id, success, placed, missing, reason,
                    placed_confirmed, uncertain)
        self.arm.on_fill_result(goal_id, success, placed, missing, reason, placed_confirmed,
                                uncertain)
        self._pump()

    # ---- 팔 상태 입력: ERROR로 바뀌면 하던 작업을 팔 고장으로 넘긴다 ----
    def on_arm_state(self, arm, state, detail="", holding=False, boot_id="", lock=True) -> None:
        self._input("on_arm_state", arm, state, detail, holding, boot_id, lock)
        self.arm.on_arm_state(text(arm), text(state), text(detail), bool(holding), text(boot_id),
                              bool(lock))
        self._pump()

    # ---- 팔 상태가 끊김: 상태 모름, 그 작업대 배차를 막음 (팔 관제 합의 ⑥) ----
    def on_arm_state_lost(self, arm) -> None:
        self._input("on_arm_state_lost", arm)
        self.arm.on_arm_state_lost(text(arm))
        self._pump()

    # ---- 담기 진행: 담은 수가 바뀌면 운반 건 진행 보고 ----
    def on_fill_feedback(self, goal_id, arm, phase, placed_total) -> None:
        self._input("on_fill_feedback", goal_id, arm, phase, placed_total)
        self.arm.on_fill_feedback(goal_id, text(arm), text(phase), int(placed_total))
        self._pump()

    # ---- 관리자 [복구]: (RESUME_SET_PAUSE_OFF | RESUME_REFILL, "") 또는 ("", 거절 사유) ----
    def on_resume_arm(self, target) -> tuple[str, str]:
        self._input("on_resume_arm", target)
        answer = self.arm.on_resume_arm(text(target))
        self._pump()
        return answer

    # ---- 상태판 팔 줄: (진행 단계, 담는 운반 건) ----
    def arm_task(self, arm) -> tuple[str, str]:
        return self.arm.arm_task(text(arm))

    def on_swap_result(self, goal_id, success, took_full, put_empty_done, reason="") -> None:
        self._input("on_swap_result", goal_id, success, took_full, put_empty_done, reason)
        self.arm.on_swap_result(goal_id, success, took_full, put_empty_done, reason)
        self._pump()

    # ---- 운영자 조작: 치운 로봇의 구역 풀기와 다시 투입 ----
    def confirm_clear(self, robot, zone_ids, note="") -> list[str] | None:
        self._input("confirm_clear", robot, zone_ids, note)
        released = self.traffic.confirm_clear(robot, zone_ids, note)
        self.last_clear_reason = self.traffic.clear_reason
        self.last_clear_code = "" if released is not None else self._clear_code(robot, zone_ids)
        if released is not None:
            self.orchestrator.on_robot_removed(robot)
        self._pump()
        return released

    def reinstate(self, robot, basket, note="") -> bool:
        self._input("reinstate", robot, basket, note)
        code, problem = self._reinstate_problem(robot, basket)
        self.last_reinstate_reason, self.last_reinstate_code = problem, code
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

    def _clear_code(self, robot, zone_ids) -> str:
        if checked(robot_problem, robot, self.traffic.robots):
            return REJECT_UNKNOWN_TARGET
        if checked(clear_problem, robot, zone_ids, self.traffic.robots,
                   lambda zone: zone in self.site.zones):
            return REJECT_NOT_FOUND
        return REJECT_NOT_STOPPED if self.traffic.clear_moving else REJECT_NOT_READY

    def _reinstate_problem(self, robot, basket) -> tuple[str, str]:
        """(계약 거절 코드, 까닭). 받아들이면 둘 다 빈 값."""
        problem = checked(robot_problem, robot, self.traffic.robots)
        if problem:
            return REJECT_UNKNOWN_TARGET, problem
        problem = checked(reinstate_problem, robot, basket, self.traffic.robots)
        if problem:
            return REJECT_ORDER_STATE, problem
        name = text(robot)
        if self.traffic.lock_released(name):
            return REJECT_LOCK_RELEASED, f"{name} has its lock released, lock it first"
        problem = self.traffic.reinstate_refusal(name)
        return (REJECT_NOT_READY, problem) if problem else ("", "")

    # ---- 락이 풀린 Pode는 새 운반 건에서 뺀다 (W-34) ----
    def set_lock_released(self, robot, released, detail="") -> None:
        self._input("set_lock_released", robot, released, detail)
        if self._robot_refused("lock release", robot):
            return
        name = text(robot)
        if not self.traffic.set_lock_released(name, bool(released)):
            return
        if released:
            why = f"{text(detail)}; " if text(detail) else ""
            self._out.append(EventOut("WARN", "ROBOT_LOCK_RELEASED",
                                      f"{why}no new trips until it is locked", name))
        else:
            self._out.append(EventOut("INFO", "ROBOT_LOCKED", "takes new trips again", name))
        self._pump()

    # ---- [충전 끝내기](시험): 로봇이 받아들이면 충전 칸에서 다시 일을 받는다 ----
    def end_charge(self, robot, detail="") -> None:
        self._input("end_charge", robot, detail)
        if self._robot_refused("charge end", robot):
            return
        if self.traffic.end_charge(text(robot)):
            self._pump()

    # ---- 운영 모드가 다른 Pode는 새 운반 건에서 뺀다 (IR-73) ----
    def set_mode_blocked(self, robot, blocked, detail="") -> None:
        self._input("set_mode_blocked", robot, blocked, detail)
        if self._robot_refused("mode block", robot):
            return
        name = text(robot)
        if not self.traffic.set_mode_blocked(name, bool(blocked)):
            return
        if blocked:
            self._out.append(EventOut("WARN", "ROBOT_MODE_MISMATCH",
                                      f"{text(detail)}; no new trips until it matches", name))
        else:
            self._out.append(EventOut("INFO", "ROBOT_MODE_MATCHED", "takes new trips again", name))
        self._pump()

    # ---- 운영자 조작: 팔 고장으로 보류된 주문의 강제 출고 ----
    def force_ship(self, trip_id) -> tuple[bool, str]:
        self._input("force_ship", trip_id)
        problem = checked(force_ship_problem, trip_id)
        if problem:
            self._reject(f"force_ship refused: {problem}")
            return False, FORCE_UNKNOWN_ORDER
        accepted, reason = self.orchestrator.on_force_ship(text(trip_id))
        if not accepted:
            self._reject(f"force_ship refused: {text(trip_id)} {reason}")
        self._pump()
        return accepted, reason

    # ---- 운영자 조작: 운반 건 명령 (resume_fill 등). 판단은 주문 쪽 몫 ----
    def on_trip_command(self, command, trip_id, mark_missing=None) -> tuple[bool, str, str]:
        """(받음, 거절 코드, 까닭). 주문 쪽에 on_trip_command가 없으면 NOT_IMPLEMENTED."""
        mark_missing = dict(mark_missing or {})
        self._input("on_trip_command", command, trip_id, mark_missing)
        handler = getattr(self.orchestrator, "on_trip_command", None)
        if handler is None:
            return False, FORCE_NOT_IMPLEMENTED, "주문 쪽 미구현"
        accepted, reason, detail = handler(text(command), text(trip_id), mark_missing)
        if not accepted:
            self._reject(f"{text(command)} refused: {text(trip_id)} {reason}")
        self._pump()
        return accepted, reason, detail

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
        # 서비스 정지(PAUSE): 대기 주문 배차는 막고 하던 이동은 마침. 새 일은 주문 코어가 막는다 (팔 관제 합의 ⑦)
        pausing = self.orchestrator.service_paused
        if everyone_here and (traffic.running or pausing):
            # 정지 중에는 모두 멈춘 로봇으로 넘겨 새 호출을 받지 않고, 빈 로봇은 대기 자리로 보낸다
            traffic.tick_pick(stopped if traffic.running else frozenset(traffic.robots))
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
