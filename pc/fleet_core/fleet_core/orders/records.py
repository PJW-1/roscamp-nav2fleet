"""주문 쪽이 함께 쓰는 기록(묶인 주문, 로봇, 놓친 작업)과 상수, 품목 합치기·빼기."""
from __future__ import annotations

from dataclasses import dataclass, field

# 스테이션에 머무는 목적
FILLING = "FILLING"
SWAPPING = "SWAPPING"

# 트레이 위 상태
EMPTY = "EMPTY"
FULL = "FULL"
NONE = "NONE"


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


@dataclass
class _LostJob:
    """ARM_NO_RESULT로 포기했거나 해제로 취소한 작업. 결과가 늦게 올 수 있다."""

    robot: str
    server: str
    kind: str  # FILL 또는 SWAP
    order_id: str  # 포기할 때 묶여 있던 주문, 이미 보고했으면 빈 값
    cleared: bool = False  # 운영자가 로봇을 해제해서 취소됨
    bound: int = 0  # 그때 주문을 묶은 차례. 같은 주문을 다시 묶으면 다른 값이다
    isolated: bool = False  # 자동 격리로 취소됨(cleared도 참)


@dataclass
class _BoundOrder:
    order_id: str
    items: dict[str, int]
    placed: dict[str, int] = field(default_factory=dict)
    missing: dict[str, int] = field(default_factory=dict)  # 확정, 다시 시도하지 않음


@dataclass
class _Robot:
    name: str
    stage: str = ""  # 팔 작업으로 스테이션에 잡혀 있는 동안 FILLING 또는 SWAPPING
    basket: str = EMPTY
    order: _BoundOrder | None = None
    last_order_id: str = ""  # 주문 보고 뒤에도 교환 goal에 붙일 이름
    outbound_goal: int | None = None
    dispatch_failures: int = 0  # failed outbound dispatches in a row
    dispatch_retry_at: float | None = None  # send the outbound dispatch again from this time
    dispatch_reason: str = ""  # why the last outbound dispatch failed
    asked: str = ""  # arm work asked for and not answered yet: FILL, SWAP or ""
    bound: int = 0  # 지금 주문을 묶은 차례(_bind_seq)
    isolated: str = ""  # 격리 이유. 해제(confirm_clear)로 지운다
