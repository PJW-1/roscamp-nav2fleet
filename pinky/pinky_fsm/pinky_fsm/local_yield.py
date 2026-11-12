"""로봇단 교착 처리(추가 지시서 3의 7-4, 기준(사용자))의 순수 판단. ROS를 모른다.

순서: 물체 앞에 서면 local_wait_s 기다린다 → 앞을 막은 것이 다른 로봇이고 내가 낮은 쪽이면 지나온
길 안에서 local_backup_m까지 물러나 다시 간다 → 그래도 막히면 BLOCKED. 이 순서는 core
(yield_moves.py)가 돌리고, 여기는 아래 셋만 정한다.
- 막은 것이 다른 로봇인가(blocking_peer): 관제의 /fleet/robots 위치가 가던 길 앞에 있다.
- 누가 물러나는가(yields_to): (priority, 이름)이 작은 쪽. 관제 choose_retreat와 같은 규칙이다.
- 얼마나 물러나는가(straight_back_m): 곧은 후진이 지나온 길에서 옆으로 벗어나지 않는 만큼.
"""
from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

from pinky_fsm.geometry import Point

NO_PRIORITY = -1  # MoveTo.priority, RobotStatus.priority가 비었다
# RobotState.local_yield(계약 0.13-draft). 로봇단 처리가 아니면 빈 값이다
WAIT, BACKUP, RETRY = "wait", "backup", "retry"


@dataclass(frozen=True)
class Peer:
    """관제가 아는 다른 로봇(/fleet/robots의 한 줄)."""
    name: str
    x: float
    y: float
    priority: int = NO_PRIORITY


def peers_from(rows) -> list[Peer]:
    """/fleet/robots(RobotStatusArray.robots)의 줄들을 Peer로. priority 칸이 없던 메시지면
    priority는 비었다."""
    return [Peer(str(r.robot_id), float(r.x), float(r.y),
                 int(getattr(r, "priority", NO_PRIORITY))) for r in rows]


def blocking_peer(ahead: Sequence[Point], peers: Iterable[Peer], me: str,
                  near_m: float) -> Peer | None:
    """가던 길 앞의 점(ahead, 로봇 쪽부터) 가운데 하나에서 near_m 안에 중심이 있는 다른 로봇.
    여럿이면 첫 점(로봇)에 가장 가까운 로봇이다. 없으면 None."""
    if not ahead:
        return None
    found = [p for p in peers
             if p.name != me and any(math.dist((p.x, p.y), q) <= near_m for q in ahead)]
    if not found:
        return None
    return min(found, key=lambda p: (math.dist((p.x, p.y), ahead[0]), p.name))


def yields_to(me: str, my_priority: int, peer: Peer) -> bool:
    """내가 물러나는가. (priority, 이름)이 작은 쪽이 물러난다. 둘 중 하나라도 priority가 없거나
    둘이 같으면 이름만 본다. 두 로봇이 같은 두 값을 보면 꼭 한쪽만 참이다."""
    if my_priority < 0 or peer.priority < 0 or my_priority == peer.priority:
        return me < peer.name
    return my_priority < peer.priority


def straight_back_m(trail: Sequence[Point], pose: Sequence[float], max_m: float,
                    lateral_m: float) -> float:
    """로봇이 보는 방향 반대로 곧게 물러날 때 지나온 길(trail, 오래된 점부터)에서 옆으로 lateral_m
    안에 머무는 가장 먼 거리. max_m을 넘지 않는다. 길이 꺾이거나 끝나면 거기까지다."""
    ux, uy = math.cos(pose[2]), math.sin(pose[2])
    best = 0.0
    for qx, qy in reversed(trail):
        dx, dy = qx - pose[0], qy - pose[1]
        behind = -(dx * ux + dy * uy)
        side = abs(-dx * uy + dy * ux)
        if side > lateral_m:
            break
        if behind < best - lateral_m:
            break  # 길이 앞으로 돌아온다
        best = max(best, behind)
        if best >= max_m:
            return max_m
    return max(0.0, min(best, max_m))


def extend_trail(trail: list[Point], here: Sequence[float], step_m: float,
                 keep_m: float) -> float:
    """지나온 길에 지금 자리를 더한다. 마지막 점에서 step_m 넘게 움직였을 때만 더하고, 길이가
    keep_m을 넘는 앞부분은 버린다. 더한 길이를 돌려준다(목표에서 간 거리를 세려고)."""
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
