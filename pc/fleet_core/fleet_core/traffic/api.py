# ---- 교통 계층이 지켜야 할 인터페이스 ----
from __future__ import annotations

from typing import Protocol, runtime_checkable

from fleet_core.common.commands import ZoneView


# ---- TrafficApi 프로토콜과 거부 사유 ----
@runtime_checkable
class TrafficApi(Protocol):
    clear_reason: str

    # ---- 로봇 상태 보고와 운영 입력 ----
    def on_robot_state(self, robot, x, y, yaw, state, battery=0.0, obstacle=None,
                       local_yield="") -> None:
        """로봇 상태 보고를 받아 지도에 올리고 위치를 따라간다."""

    def on_clear_check(self, robot, check_id, answer) -> None:
        """막힘 지점 확인(CheckClear)의 답을 받는다."""

    def on_run_mode(self, run_mode) -> None:
        """운행 모드를 바꾼다. STOP이 되면 열린 goal을 취소한다."""

    def on_dispatch(self, goal_id, station_id, robot_id="") -> None:
        """로봇을 스테이션으로 부르거나(실패는 DispatchResult) 채우기 스테이션의 호출을 연다."""

    def on_dispatch_cancel(self, goal_id) -> None:
        """배차나 열린 호출을 거둔다."""

    def on_release(self, robot) -> None:
        """스테이션에 붙잡아 둔 로봇을 놓아 준다."""

    def on_robot_fault(self, robot, reason="") -> None:
        """주문 쪽이 알린 고장으로 로봇을 격리한다."""

    # ---- 이동, 물러나기, 도킹 goal 결과 ----
    def on_move_feedback(self, robot, goal_id, last_done_seq, waiting_at_base_end,
                         horizon_warning="") -> None:
        """order 모델 이동 피드백을 받는다."""

    def on_move_result(self, robot, goal_id, success, reason, last_done_seq=0) -> None:
        """이동 goal 결과를 받는다."""

    def on_goal_rejected(self, robot, goal_id) -> None:
        """액션 서버가 goal을 거절했음을 받는다."""

    def on_goal_not_sent(self, robot, goal_id) -> None:
        """goal이 나가지 않았음을 받아 잠시 뒤 다시 낸다."""

    def on_goal_lost(self, robot, goal_id) -> None:
        """결과를 잃은 goal을 끝난 것으로 본다."""

    def on_retreat_result(self, robot, goal_id, success, reason="") -> None:
        """물러나기 goal 결과를 받는다."""

    def on_dock_result(self, robot, goal_id, success, error_x_cm=0.0, error_y_cm=0.0,
                       error_yaw_deg=0.0) -> None:
        """도킹 goal 결과를 받는다."""

    # ---- 운영자의 로봇 치우기와 다시 투입 ----
    def confirm_clear(self, robot, zone_ids, note="") -> list[str] | None:
        """지정 구역을 놓고 로봇을 OUT_OF_SERVICE로 만든다. 거절하면 None, 사유는 clear_reason."""

    def reinstate_refusal(self, robot) -> str:
        """치운 로봇을 다시 투입할 수 없는 이유를 돌려준다(없으면 빈 값)."""

    def reinstate(self, robot) -> bool:
        """치운 로봇을 다시 투입한다. 거절되면 False, 받아들이면 다음 위치 보고 때 지도에 올린다."""

    # ---- 로봇 목록과 운행 상태 조회 ----
    @property
    def robots(self) -> list[str]:
        """로봇 이름 목록."""

    @property
    def everyone_placed(self) -> bool:
        """모든 로봇이 지도에 올랐는지."""

    @property
    def running(self) -> bool:
        """운행 모드가 RUN인지."""

    @property
    def zones(self):
        """구역 허락 기록."""

    def available(self, robot) -> bool:
        """로봇이 지도에 올라 있고 고장이나 통신 두절이 아닌지."""

    # ---- 틱 진행 ----
    def tick(self) -> list:
        """한 주기 전체를 돌리고 나온 출력을 돌려준다."""

    def tick_begin(self, now: float) -> bool:
        """시작과 통신을 점검하고, 시작에 필요한 로봇이 모두 지도에 올랐는지 돌려준다."""

    def tick_progress(self, now: float) -> frozenset[str]:
        """막힘 만료와 로봇별 감시, 교착 감시를 돌리고 이번 틱에 진행시키지 않을 로봇을 돌려준다."""

    def tick_pick(self, stopped: frozenset[str] = frozenset()) -> None:
        """stopped를 뺀 로봇 가운데 열린 호출에 응할 로봇을 정한다."""

    def step(self, robot) -> None:
        """로봇의 단계를 진행한다."""

    # ---- 출력 꺼내기와 상태 보기 ----
    def take_output(self) -> list:
        """쌓인 명령과 이벤트를 꺼낸다."""

    def zones_view(self) -> list[ZoneView]:
        """구역마다의 주인과 대기 줄을 돌려준다."""

    def robot_view(self, robot) -> dict:
        """로봇 하나의 관제 상태를 돌려준다."""

    def status_view(self, robot) -> dict:
        """/fleet/robots에 낼 로봇 상태 가운데 교통 계층이 채우는 항목을 돌려준다."""
