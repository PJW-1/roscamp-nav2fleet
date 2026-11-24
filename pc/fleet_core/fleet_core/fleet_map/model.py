# ---- 지도 모델: 상수와 구역, 노드, edge, 스테이션 데이터 클래스 ----
from __future__ import annotations

from dataclasses import dataclass

from .geometry import Point, Rect

# ---- 구역, 노드, edge, 작업 종류 상수 ----
SHARED = "shared"
LANE = "lane"
DOCK = "dock"
ZONE_KINDS = (SHARED, LANE, DOCK)

AT_DOCK = "dock"
PASS = "pass"
FORK = "fork"
WAIT = "wait"
PARKING = "parking"
NODE_KINDS = (AT_DOCK, PASS, FORK, WAIT, PARKING)
WAITING_KINDS = (AT_DOCK, WAIT, PARKING)

NAV_GOAL = "nav_goal"
FOLLOW_PATH = "follow_path"
RETREAT = "retreat"

FILL = "fill"
SWAP = "swap"


# ---- 지도 오류와 지도 요소 데이터 클래스 ----
class FleetMapError(ValueError):
    pass


@dataclass(frozen=True)
class Zone:
    id: str
    kind: str
    capacity: int
    rects: tuple[Rect, ...]


@dataclass(frozen=True)
class Node:
    id: str
    x: float
    y: float
    kind: str
    zone: str
    yaw: float | None = None
    through: bool = False
    via: bool = False  # 나눈 goal의 중간 지점 전용: 자유 이음의 목적지가 아니다


@dataclass(frozen=True)
class Edge:
    a: str
    b: str
    mode: str
    path: str
    points: tuple[Point, ...]
    zones: tuple[str, ...]
    length: float
    lane: str = ""
    extra_m: float = 0.0

    @property
    def price_m(self) -> float:
        return self.length + self.extra_m


# 출발 자리가 rects 안이면 to로 가는 Nav2 goal을 via를 거쳐 둘로 나눈다(해결안 B, plan_split)
@dataclass(frozen=True)
class Split:
    to: str
    via: str
    rects: tuple[Rect, ...]


@dataclass(frozen=True)
class Station:
    id: str
    dock: str
    queue: tuple[str, ...]
    server: str
    arms: tuple[str, ...]
    action: str
    line: bool = False  # 대기 자리가 줄이다(앞자리 먼저, 새 로봇은 꼬리로)
