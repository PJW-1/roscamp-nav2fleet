"""교통 층의 바깥 API: FleetManager(manager.py)와 ROS 껍데기(fleet_manager_node.py)가 부르는 것.

ZoneTraffic이 채우는 자리다. 옛 레인 제어기(지웠다, 1da1e61 traffic/controller.py)의 공개 메서드를
시그니처 그대로 따랐으므로 manager.py를 고치지 않고 끼운다. 한 줄 설명은 부르는 쪽이 기대하는
것이다. 로봇 이름은 모두 생성자의 robots 중 하나이고, 모르는 이름은 조용히 무시한다
(INPUT_REJECTED는 FleetManager가 낸다). 시각은 생성자의 clock으로 읽는다.

생성자: ZoneTraffic(site, params, robots, clock, next_goal_id). next_goal_id()는 FleetManager가
goal id를 나눠 주는 함수다(팔과 같은 수열).
선택: station_left 속성이 있으면 FleetManager가 station_left_s(robot)를 넣어 준다(스테이션 팔 작업이
끝나기까지 남은 초, 추정. 팔 일 중이 아니면 None).
"""
from __future__ import annotations

from typing import Protocol, runtime_checkable

from fleet_core.common.commands import ZoneView


@runtime_checkable
class TrafficApi(Protocol):
    clear_reason: str  # 마지막 confirm_clear를 거부한 이유. 받아들였으면 ""

    # ---- inputs -------------------------------------------------------------------------

    def on_robot_state(self, robot, x, y, yaw, state, battery=0.0, obstacle=None,
                       local_yield="") -> None:
        """로봇 상태 보고(검사를 마친 값, state는 글자). 지도에 올리고 위치를 따라간다.
        obstacle은 BLOCKED와 함께 온 장애물 (x, y, 종류)이고 모르면 None이다.
        local_yield는 RECOVERING일 때 로봇단 교착 처리의 단계다(계약 0.13-draft). 아니면 빈 값."""

    def on_clear_check(self, robot, check_id, answer) -> None:
        """CheckClearCmd의 답(clear, blocked, unknown). 모르는 check_id면 버린다."""

    def on_run_mode(self, run_mode) -> None:
        """중앙의 RUN/PAUSE/STOP. STOP이면 열린 goal을 취소한다. RUN이 아니면 새로 내지 않는다."""

    def on_dispatch(self, goal_id, station_id, robot_id="") -> None:
        """로봇을 스테이션으로 부른다. robot_id가 없으면 채우기 스테이션의 열린 호출이다.
        끝나면 DispatchResult(goal_id, ...)를, 중간 단계는 DispatchFeedback을 take_output에 낸다."""

    def on_dispatch_cancel(self, goal_id) -> None:
        """그 배차(열린 호출 포함)를 거둔다. 지정 배차였으면 DispatchResult(CANCELED)."""

    def on_release(self, robot) -> None:
        """스테이션에 붙잡아 둔 로봇을 놓아 준다. 다음 선택 뒤에 움직인다."""

    def on_robot_fault(self, robot, reason="") -> None:
        """주문 쪽이 알린 고장. 로봇을 ERROR로 두고 격리한다(구역은 쥔 채)."""

    def on_move_feedback(self, robot, goal_id, last_done_seq, waiting_at_base_end,
                         horizon_warning="") -> None:
        """order 모델(MoveOrderCmd)의 피드백: 끝낸 마지막 조각, base 끝에서 기다리는가, horizon
        미리 검사의 경고("HORIZON_INVALID: ..." 또는 빈 값). order가 아닌 goal에는 오지 않는다."""

    def on_move_result(self, robot, goal_id, success, reason, last_done_seq=0) -> None:
        """MoveTo 결과. 지금 goal id가 아니면 버린다. 실패는 재시도 수로 센다."""

    def on_goal_rejected(self, robot, goal_id) -> None:
        """액션 서버가 goal을 거절했다. 실패 한 번으로 센다(Retreat면 retreat 결과로)."""

    def on_goal_not_sent(self, robot, goal_id) -> None:
        """액션 서버가 없어 goal이 나가지 않았다. 시도로 세지 않고 잠시 뒤 다시 낸다."""

    def on_goal_lost(self, robot, goal_id) -> None:
        """노드가 취소한 goal의 결과를 잃었다. 끝난 것으로 본다."""

    def on_retreat_result(self, robot, goal_id, success, reason="") -> None:
        """RetreatCmd의 결과. reason: OBSTACLE_BEHIND, TIMEOUT, CANCELED, FAILED, REJECTED,
        NOT_SENT, LOST. RetreatCmd를 내지 않는 교통 층은 아무것도 하지 않는다."""

    def on_dock_result(self, robot, goal_id, success, error_x_cm=0.0, error_y_cm=0.0,
                       error_yaw_deg=0.0) -> None:
        """Dock 결과. 성공이면 AT_STATION이 되고 배차에 DispatchResult(성공)로 답한다."""

    def confirm_clear(self, robot, zone_ids, note="") -> list[str] | None:
        """운영자가 멈춘 로봇을 치웠다. 놓은 구역, 거부면 None(이유는 clear_reason).
        로봇은 OUT_OF_SERVICE가 된다."""

    def reinstate_refusal(self, robot) -> str:
        """치운 로봇을 다시 넣을 수 없는 이유, 넣을 수 있으면 ""."""

    def reinstate(self, robot) -> bool:
        """치운 로봇을 다시 넣는다. 다음 위치 보고로 지도에 오른다. 거부면 False."""

    # ---- state the caller reads ---------------------------------------------------------

    @property
    def robots(self) -> list[str]:
        """생성자의 로봇 이름, 같은 순서."""

    @property
    def everyone_placed(self) -> bool:
        """시작 때 모든 로봇이 지도에 올랐다. 그 전에는 /fleet/zones를 내지 않는다."""

    @property
    def running(self) -> bool:
        """run mode가 RUN이다."""

    @property
    def zones(self):
        """구역 허락 기록(zone/grants.py의 ZoneGrants: held_by, holders, nodes_of, waiting)."""

    def available(self, robot) -> bool:
        """이번 tick에 새 일을 받고 step할 수 있다(지도 위, 고장·두절 없음)."""

    # ---- one tick -----------------------------------------------------------------------

    def tick(self) -> list:
        """한 주기 전체(tick_begin, tick_progress, tick_pick, step). 나온 것을 돌려준다."""

    def tick_begin(self, now: float) -> bool:
        """시작과 통신 점검. 모든 로봇이 보고했으면 True."""

    def tick_progress(self, now: float) -> frozenset[str]:
        """위치를 따라가며 구역을 놓고 감시를 돈다. 이번 tick에 새로 움직이면 안 되는 로봇
        (대기 순환의 로봇)을 돌려준다."""

    def tick_pick(self, stopped: frozenset[str] = frozenset()) -> None:
        """열린 호출에 응할 로봇을 정한다. stopped의 로봇은 고르지 않는다."""

    def step(self, robot) -> None:
        """그 로봇의 단계를 진행한다(구역 요청, 명령 내기). available한 로봇만 부른다."""

    def take_output(self) -> list:
        """쌓인 명령과 이벤트(MoveCmd, DockCmd, RetreatCmd, CancelCmd, EventOut,
        DispatchFeedback, DispatchResult, RobotIsolated)를 꺼낸다."""

    # ---- views --------------------------------------------------------------------------

    def zones_view(self) -> list[ZoneView]:
        """/fleet/zones에 낼 구역마다의 주인과 대기 줄."""

    def robot_view(self, robot) -> dict:
        """모르는 로봇이면 {}. 키: state, phase, on_map, fault, comm, called, charging, held,
        path, waypoint, destination, goal_id, attempts, blocked_lane."""

    def status_view(self, robot) -> dict:
        """/fleet/robots의 교통 몫. 키: fsm_state, fault, fault_reason, wait_zone, wait_robots,
        battery, x, y, yaw, base, horizon(앞에 남은 길의 지점 이름: 허락받은 것과 아닌 것)."""
