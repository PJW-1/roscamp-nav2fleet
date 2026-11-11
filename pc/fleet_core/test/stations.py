"""주문과 팔 시험이 쓰는 스테이션 둘. 지도 없이 쓴다.

값은 지운 시험 지도(1da1e61 test/data/grid_cells.yaml)의 stations 중 주문(orders/)과 팔(arm/)이
읽는 칸(id, server, arms, action)만 옮겼다. 도크 자리, 줄, commit은 교통의 일이라 뺐다.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Station:
    id: str
    server: str  # 이 스테이션 액션의 서버
    arms: tuple[str, ...]  # 그 서버 뒤에서 일하는 팔
    action: str  # fill 또는 swap


STATIONS = {
    "WORK": Station("WORK", "workcell", ("arm1", "arm2"), "fill"),
    "OUTBOUND": Station("OUTBOUND", "arm3", ("arm3",), "swap"),
}
