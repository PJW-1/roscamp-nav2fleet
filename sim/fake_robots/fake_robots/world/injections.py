"""시험이 세계에 미리 넣어 두는 일: 사람이나 상자, 통신 끊김, 로봇 치우기와 다시 넣기, 느린 팔,
배터리 부족, 멈춘 로봇, 크게 튀는 위치 보고. 관제는 이것을 모른다."""
from __future__ import annotations

import math
from dataclasses import dataclass

FOREVER = math.inf


@dataclass(frozen=True)
class Obstacle:
    """지도에 없는 원(사람, 상자). t0부터 t1 전까지 바닥에 있다."""
    x: float
    y: float
    radius: float = 0.05
    t0: float = 0.0
    t1: float = FOREVER


@dataclass(frozen=True)
class CommLoss:
    """보고가 안 나가고 명령과 결과가 오가지 않는다. 로봇은 하던 goal을 이어 간다."""
    robot: str
    t0: float
    t1: float


@dataclass(frozen=True)
class Removal:
    """로봇을 바닥에서 들어낸다. clear면 운영자가 쥔 구역을 모두 풀어 준다(confirm_clear)."""
    robot: str
    at_s: float
    clear: bool = True


@dataclass(frozen=True)
class Reinstate:
    """치운 로봇을 node에 놓고 운영자가 다시 넣는다(reinstate)."""
    robot: str
    at_s: float
    node: str
    basket: str = "EMPTY"


@dataclass(frozen=True)
class SlowStation:
    """그 스테이션의 팔이 factor배 느리다."""
    station: str
    factor: float
    t0: float = 0.0
    t1: float = FOREVER


@dataclass(frozen=True)
class LowBattery:
    """at_s부터 배터리가 낮고, 충전 자리에 duration_s 서 있으면 다시 찬다(fake_robots.battery)."""
    robot: str
    at_s: float
    duration_s: float
    type: str = "low_battery"


@dataclass(frozen=True)
class Stuck:
    """로봇이 움직이지 못한다(바퀴 고장). 이동은 TIMEOUT으로 끝난다."""
    robot: str
    t0: float
    t1: float = FOREVER


@dataclass(frozen=True)
class NoisyPose:
    """보고 위치에 sigma_m 잡음을 더하고, jump_m만큼 한쪽(+x)으로 민다. 실제 자리는 그대로다.

    drive_by_estimate면 로봇이 민 자리(믿는 자리)로 달린다(AMCL이 틀린 로봇). 보고는 길 위에 있고
    몸이 -jump_m만큼 벗어난다. 창이 끝나면 믿는 자리가 실제로 돌아온다(다시 수렴)."""
    robot: str
    t0: float
    t1: float
    sigma_m: float = 0.0
    jump_m: float = 0.0
    drive_by_estimate: bool = False


@dataclass(frozen=True)
class RelocalizeFails:
    """이 창에서 받은 위치 다시 찾기(Relocalize)는 reason으로 실패한다. 추정은 그대로 틀린 채다."""
    robot: str
    t0: float = 0.0
    t1: float = FOREVER
    reason: str = "NOT_CONVERGED"


@dataclass(frozen=True)
class OrderFault:
    """관제의 시험 고리(traffic.order_fault)를 켠다. "stitch"면 다음 update의 이음 지점이 틀린다."""
    at_s: float
    kind: str = "stitch"


def _during(item, t: float) -> bool:
    return item.t0 <= t < item.t1


class Schedule:
    def __init__(self, items=()):
        self.items = list(items)
        self._fired: set[int] = set()
        self._relocalized: dict[str, float] = {}  # 로봇 -> 마지막으로 위치를 다시 찾은 시각

    def of(self, kind) -> list:
        return [i for i in self.items if isinstance(i, kind)]

    def objects_now(self, t: float) -> list[tuple[float, float, float]]:
        return [(o.x, o.y, o.radius) for o in self.of(Obstacle) if _during(o, t)]

    def comm_down(self, robot: str, t: float) -> bool:
        return any(c.robot == robot and _during(c, t) for c in self.of(CommLoss))

    def stuck(self, robot: str, t: float) -> bool:
        return any(s.robot == robot and _during(s, t) for s in self.of(Stuck))

    def slow_factor(self, station: str, t: float) -> float:
        return math.prod(s.factor for s in self.of(SlowStation)
                         if s.station == station and _during(s, t))

    def noise(self, robot: str, t: float) -> tuple[float, float]:
        """(더할 잡음, 미는 거리)."""
        found = [n for n in self._noisy(robot, t)]
        return (max((n.sigma_m for n in found), default=0.0),
                sum(n.jump_m for n in found))

    def _noisy(self, robot: str, t: float) -> list:
        """지금 걸린 NoisyPose 창. 다시 찾기 전에 시작한 drive_by_estimate 창은 끝난 것으로 본다."""
        fixed = self._relocalized.get(robot, -math.inf)
        return [n for n in self.of(NoisyPose) if n.robot == robot and _during(n, t)
                and not (n.drive_by_estimate and n.t0 <= fixed)]

    def relocalize(self, robot: str, t: float) -> None:
        """위치를 다시 찾았다: 그때까지 시작한 믿는 치우침이 끝난다(보고만 미는 창은 그대로)."""
        self._relocalized[robot] = t

    def relocalize_fails(self, robot: str, t: float) -> str:
        """지금 다시 찾기가 실패할 까닭. 없으면 빈 값."""
        return next((f.reason for f in self.of(RelocalizeFails)
                     if f.robot == robot and _during(f, t)), "")

    def belief_dx(self, robot: str, t: float) -> float:
        """로봇이 믿는 자리 - 실제 자리(+x). drive_by_estimate인 것만 더한다."""
        return sum(n.jump_m for n in self._noisy(robot, t) if n.drive_by_estimate)

    def due(self, t: float) -> list:
        """지금 일어날 한 번짜리 일(Removal, Reinstate). 한 번만 돌려준다."""
        out = []
        for k, item in enumerate(self.items):
            if (isinstance(item, (Removal, Reinstate, OrderFault)) and k not in self._fired
                    and item.at_s <= t):
                self._fired.add(k)
                out.append(item)
        return out
