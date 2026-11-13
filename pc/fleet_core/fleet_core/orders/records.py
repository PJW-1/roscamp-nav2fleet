# ---- 주문 코어의 상태 상수와 기록 데이터 클래스 ----
from __future__ import annotations

from dataclasses import dataclass, field

# ---- 단계와 바구니 상태 상수 ----
FILLING = "FILLING"
SWAPPING = "SWAPPING"

EMPTY = "EMPTY"
FULL = "FULL"
NONE = "NONE"


# ---- 수량 합산과 차감 ----
def _merge(into: dict[str, int], more: dict[str, int]) -> None:
    for kind, count in more.items():
        if count > 0:
            into[kind] = into.get(kind, 0) + count


def _subtract(items: dict[str, int], *taken: dict[str, int]) -> dict[str, int]:
    left = {}
    for kind, count in items.items():
        rest = count - sum(t.get(kind, 0) for t in taken)
        if rest > 0:
            left[kind] = rest
    return left


# ---- 잃은 작업, 묶인 주문, 로봇 기록 ----
@dataclass
class _LostJob:
    robot: str
    server: str
    kind: str
    order_id: str
    cleared: bool = False
    bound: int = 0
    isolated: bool = False


@dataclass
class _BoundOrder:
    order_id: str
    items: dict[str, int]
    placed: dict[str, int] = field(default_factory=dict)
    missing: dict[str, int] = field(default_factory=dict)


@dataclass
class _Robot:
    name: str
    stage: str = ""
    basket: str = EMPTY
    order: _BoundOrder | None = None
    last_order_id: str = ""
    outbound_goal: int | None = None
    dispatch_failures: int = 0
    dispatch_retry_at: float | None = None
    dispatch_reason: str = ""
    asked: str = ""
    bound: int = 0
    isolated: str = ""
