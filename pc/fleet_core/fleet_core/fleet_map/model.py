"""구역 지도의 기록들: 구역, 지점, 연결, 스테이션. 로직은 없다."""
from __future__ import annotations

from dataclasses import dataclass

from .geometry import Point, Rect

# 구역 종류
SHARED = "shared"  # 넓은 곳: Nav2 goal로 다니고 여러 대가 같이 쓴다
LANE = "lane"  # 한 줄 구역(급행): FollowPath로 다니고 한 번에 한 방향만
DOCK = "dock"  # 도크와 그 앞 갈림목: FollowPath로 다니고 한 대만
ZONE_KINDS = (SHARED, LANE, DOCK)

# 지점 종류. 기다려도 되는 자리는 도크, 대기 자리, 주차뿐이다
AT_DOCK = "dock"
PASS = "pass"  # 갈림목이나 급행 안: 지나가기만 한다
FORK = "fork"  # 넓은 구역의 갈림 지점: 명령이 바뀌는 자리, 오래 서지 않는다
WAIT = "wait"
PARKING = "parking"
NODE_KINDS = (AT_DOCK, PASS, FORK, WAIT, PARKING)
WAITING_KINDS = (AT_DOCK, WAIT, PARKING)

# 연결을 달리는 방법
NAV_GOAL = "nav_goal"
FOLLOW_PATH = "follow_path"
RETREAT = "retreat"

# 스테이션 동작
FILL = "fill"
SWAP = "swap"


class FleetMapError(ValueError):
    """지도 파일이 맞지 않는다."""


@dataclass(frozen=True)
class Zone:
    id: str
    kind: str
    capacity: int  # shared: 같이 있을 수 있는 대수. lane: 같은 방향으로 함께 들어갈 수 있는 대수
    rects: tuple[Rect, ...]


@dataclass(frozen=True)
class Node:
    id: str
    x: float
    y: float
    kind: str
    zone: str
    yaw: float | None = None  # 여기서 서는 방향(rad). 적지 않았으면 None


@dataclass(frozen=True)
class Link:
    a: str
    b: str
    via: str  # nav_goal, follow_path, retreat
    path: str  # 고정 경로 이름. nav_goal이면 ""
    points: tuple[Point, ...]  # a에서 b까지의 선
    zones: tuple[str, ...]  # a의 구역부터 b의 구역까지, 지나는 순서
    length: float
    lane: str = ""  # 지나는 한 줄 구역. 없으면 ""


@dataclass(frozen=True)
class Station:
    id: str
    dock: str  # 지점
    queue: tuple[str, ...]  # 도크가 찼을 때 기다리는 지점, 앞자리부터
    server: str  # 이 스테이션 액션의 서버
    arms: tuple[str, ...]  # 그 서버 뒤에서 일하는 팔
    action: str  # fill 또는 swap
