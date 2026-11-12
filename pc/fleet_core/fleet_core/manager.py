"""주문, 교통, 팔 작업 코어를 직접 호출로 잇는다.

셋 사이 명령은 밖에 안 나가고 나머지는 tick()으로 나간다.

주문이 위에서 정하고 교통(로봇 이동)과 팔 작업(fill, swap)에 요청한다. 교통과 팔은 서로 모른다.

배차 API 밖으로 넘기는 것은 둘: run mode(STOP은 주문 없이 달리는 로봇도 세움)와 운영자의 로봇 제거.
외부 호출은 모두 _input을 거친다. 호출 동안 시각을 하나로 고정하고, 기록 중이면 남겨
ROS 없이 재생할 수 있다(형식은 tools/replay.py).

밖에서 온 잘못된 입력은 기록한 뒤, 코어에 닿기 전에 여기서 거른다(common/validate.py).
서비스는 거부, 토픽은 버리고 INPUT_REJECTED(WARN)를 낸다. 서비스는 호출마다 한 번,
토픽은 출처마다 한 번 내고 정상 메시지가 오면 다시 무장한다.
"""
from __future__ import annotations

from fleet_core.arm.jobs import ArmJobs
from fleet_core.common.commands import (
    FILL,
    SWAP,
    EventOut,
)
from fleet_core.common.validate import (
    checked,
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
            self.traffic.station_left = self.station_left_s  # 스테이션 작업 ETA를 쓰는 교통 층
        self._outbound = next(s.id for s in site.stations.values() if s.action == SWAP)
        self._out: list = []
        self._bounds = site.bounds()
        self._board: list = []  # 검사를 통과한 마지막 주문 목록
        self._rejecting: set = set()  # 거부를 이미 알린 토픽 출처
        self.last_clear_reason = ""
        self.last_reinstate_reason = ""

    def station_left_s(self, robot) -> float | None:
        """스테이션 팔 작업이 끝나기까지(초, 추정). 팔 일 중이 아니면 None."""
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
        return max(1.0, total - (self._frozen() - sent))  # 시간이 지난 일은 곧 끝난다고 봄

    @property
    def zones(self):
        return self.traffic.zones

    @property
    def shipped(self) -> int:
        return self.orchestrator.shipped

    # ---- inputs -------------------------------------------------------------------------

    def on_robot_state(self, robot, x, y, yaw, state, battery=0.0, robot_id="") -> None:
        """robot은 토픽의 로봇, robot_id는 메시지에 적힌 이름(빈 값 가능)."""
        # 받은 그대로 기록한다. robot_id가 없던 기록도 똑같이 읽힌다.
        no_id = isinstance(robot_id, str) and text(robot_id) == ""
        self._input("on_robot_state", robot, x, y, yaw, state, battery,
                    *([] if no_id else [robot_id]))
        # 위치를 못 잡은 보고의 자세는 쓰지 않으니 검사하지 않는다.
        localizing = isinstance(state, str) and text(state) == LOCALIZING
        problem = checked(lambda: robot_problem(robot, self.traffic.robots)
                          or reported_id_problem(robot, robot_id)
                          or ("" if localizing else pose_problem(
                              x, y, yaw, self._bounds, self.params.input_pose_margin_m)))
        if self._topic_rejected(("robot_state", text(robot)), problem, "robot state dropped",
                                robot):
            return
        # state는 판단에 쓰므로 글자로 바꿔 둔다. 비교가 실패하는 값이 tick까지 가면 안 된다.
        self.traffic.on_robot_state(robot, x, y, yaw, text(state), battery)
        self._pump()

    def on_order_board(self, run_mode, orders) -> None:
        """run_mode가 틀리면 통째로 버린다. 주문이 틀리면 모드는 적용하고 주문은 전 것을 둔다.

        틀린 주문 하나가 STOP을 막으면 안 되고, 하나만 빼면 철회로 보이기 때문이다.
        """
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
        """order 모델의 MoveTo 피드백(계약 0.11-draft 제안). 값이 바뀔 때만 불러도 된다."""
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
        """노드가 취소한 goal의 결과를 잃었다(goal_tracker가 취소를 그만둠)."""
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
        """Retreat(물러나기)의 결과.

        reason: OBSTACLE_BEHIND, TIMEOUT, CANCELED, FAILED(성공은 빈 값).
        """
        self._input("on_retreat_result", robot, goal_id, success, reason)
        if not self._robot_refused("retreat result", robot):
            self.traffic.on_retreat_result(robot, goal_id, success, reason)
            self._pump()

    def on_fill_result(self, goal_id, success, placed, missing, reason="") -> None:
        self._input("on_fill_result", goal_id, success, placed, missing, reason)
        self.arm.on_fill_result(goal_id, success, placed, missing, reason)
        self._pump()

    def on_swap_result(self, goal_id, success, took_full, put_empty_done, reason="") -> None:
        self._input("on_swap_result", goal_id, success, took_full, put_empty_done, reason)
        self.arm.on_swap_result(goal_id, success, took_full, put_empty_done, reason)
        self._pump()

    def confirm_clear(self, robot, zone_ids, note="") -> list[str] | None:
        """놓은 구역, 거부면 None. 거부 이유는 last_clear_reason(받아들이면 빈 값).

        격리된 로봇이면 주문은 이미 돌아갔으니 구역만 푼다. 아니면 묶인 주문도 되돌린다.

        모르는 로봇·구역은 INPUT_REJECTED, 이름이 맞는 거부는 CLEAR_REFUSED. 판단은 traffic이 한다.
        """
        self._input("confirm_clear", robot, zone_ids, note)
        released = self.traffic.confirm_clear(robot, zone_ids, note)
        self.last_clear_reason = self.traffic.clear_reason
        if released is not None:
            self.orchestrator.on_robot_removed(robot)
        self._pump()
        return released

    def reinstate(self, robot, basket, note="") -> bool:
        """치운 로봇을 다시 넣는다. 거부면 False, 이유는 last_reinstate_reason(받아들이면 빈 값).

        모르는 로봇, OUT_OF_SERVICE가 아닌 로봇, 틀린 basket은 INPUT_REJECTED.
        맵에는 다음 위치 보고로 오른다. 그 전에는 일을 받지 않고 다른 로봇도 멈추지 않는다.
        """
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

    # ---- outputs ------------------------------------------------------------------------

    def tick(self) -> list:
        self._input("tick")
        traffic = self.traffic
        now = self._time
        everyone_here = traffic.tick_begin(now)
        self._pump()
        self.arm.check_timeouts(now)
        self._pump()
        stopped = traffic.tick_progress(now)  # robots held still this tick (zone layer: none)
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

    # ---- the join -----------------------------------------------------------------------

    def _frozen(self) -> float:
        return self._time

    def _next_goal_id(self) -> int:
        self._goal_seq += 1
        return self._goal_seq
