"""주문과 팔 시험이 쓰는 스테이션 둘(지도 없이). 주문(orders/)과 팔(arm/)이 읽는 칸(id, server, arms, action)만 있다."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Station:
    id: str
    server: str  # 이 스테이션 액션의 서버
    arms: tuple[str, ...]  # 그 서버 뒤에서 일하는 팔
    action: str  # fill 또는 swap


STATIONS = {
    "WORK": Station("WORK", "workcell", ("arm_left", "arm_right"), "fill"),
    "OUTBOUND": Station("OUTBOUND", "arm_ship", ("arm_ship",), "swap"),
}
