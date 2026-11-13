# ---- 로컬 양보 판단에 쓰는 상대 로봇과 후진 거리 계산 ----
from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

from pinky_fsm.geometry import Point

# ---- 우선순위와 양보 단계 상수, 상대 로봇(Peer) 자료형 ----
NO_PRIORITY = -1
WAIT, BACKUP, RETRY = "wait", "backup", "retry"


@dataclass(frozen=True)
class Peer:
    name: str
    x: float
    y: float
    priority: int = NO_PRIORITY


# ---- 앞을 막는 상대 로봇 찾기와 양보 여부 판정 ----
def peers_from(rows) -> list[Peer]:
    return [Peer(str(r.robot_id), float(r.x), float(r.y),
                 int(getattr(r, "priority", NO_PRIORITY))) for r in rows]


def blocking_peer(ahead: Sequence[Point], peers: Iterable[Peer], me: str,
                  near_m: float) -> Peer | None:
    if not ahead:
        return None
    found = [p for p in peers
             if p.name != me and any(math.dist((p.x, p.y), q) <= near_m for q in ahead)]
    if not found:
        return None
    return min(found, key=lambda p: (math.dist((p.x, p.y), ahead[0]), p.name))


def yields_to(me: str, my_priority: int, peer: Peer) -> bool:
    if my_priority < 0 or peer.priority < 0 or my_priority == peer.priority:
        return me < peer.name
    return my_priority < peer.priority


# ---- 지나온 궤적으로 곧게 후진할 거리 계산과 궤적 기록 ----
def straight_back_m(trail: Sequence[Point], pose: Sequence[float], max_m: float,
                    lateral_m: float) -> float:
    ux, uy = math.cos(pose[2]), math.sin(pose[2])
    best = 0.0
    for qx, qy in reversed(trail):
        dx, dy = qx - pose[0], qy - pose[1]
        behind = -(dx * ux + dy * uy)
        side = abs(-dx * uy + dy * ux)
        if side > lateral_m:
            break
        if behind < best - lateral_m:
            break
        best = max(best, behind)
        if best >= max_m:
            return max_m
    return max(0.0, min(best, max_m))


def extend_trail(trail: list[Point], here: Sequence[float], step_m: float,
                 keep_m: float) -> float:
    p = (float(here[0]), float(here[1]))
    if trail and math.dist(trail[-1], p) < step_m:
        return 0.0
    gained = math.dist(trail[-1], p) if trail else 0.0
    trail.append(p)
    length = sum(math.dist(a, b) for a, b in zip(trail, trail[1:], strict=False))
    while len(trail) > 2 and length - math.dist(trail[0], trail[1]) >= keep_m:
        length -= math.dist(trail[0], trail[1])
        trail.pop(0)
    return gained
