# ---- 지도 모델: 상수와 구역, 노드, 링크, 스테이션 데이터 클래스 ----
from __future__ import annotations

from dataclasses import dataclass

from .geometry import Point, Rect

# ---- 구역, 노드, 링크, 작업 종류 상수 ----
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


@dataclass(frozen=True)
class Link:
    a: str
    b: str
    via: str
    path: str
    points: tuple[Point, ...]
    zones: tuple[str, ...]
    length: float
    lane: str = ""


@dataclass(frozen=True)
class Station:
    id: str
    dock: str
    queue: tuple[str, ...]
    server: str
    arms: tuple[str, ...]
    action: str
    line: bool = False  # 대기 자리가 줄이다: 앞자리 먼저, 빈 앞자리로 당기기, 새 로봇은 꼬리로
