"""가짜 핑키 한 대의 실제 상태와 씨드로 고른 흠(속도 차이, 도착 오차, 위치 잡음, 보고 늦음)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Imperfections:
    """세계 전체의 흠 크기. 로봇마다 실제 값은 씨드로 고른다(같은 씨드면 같다)."""

    speed_spread: float = 0.10  # 속도 상한의 (1 - spread)..1 사이에서 로봇마다
    speed_jitter: float = 0.03  # 걸음마다 더하는 흔들림(비율)
    arrival_error_m: float = 0.01  # 목표에서 이 안의 아무 자리에 선다
    pose_noise_m: float = 0.005  # 보고 위치의 가우스 잡음(표준편차)
    report_delay_max_s: float = 0.2  # 로봇마다 0..이 값 사이로 보고가 늦다
    result_early_p: float = 0.5  # 결과가 같은 걸음의 위치 보고보다 먼저 갈 확률

    @staticmethod
    def none() -> Imperfections:
        return Imperfections(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Profile:
    speed_factor: float = 1.0
    report_delay_s: float = 0.0


def profile_for(rng, imperfect: Imperfections) -> Profile:
    return Profile(speed_factor=1.0 - rng.uniform(0.0, imperfect.speed_spread),
                   report_delay_s=round(rng.uniform(0.0, imperfect.report_delay_max_s), 1))


@dataclass
class SimRobot:
    name: str
    x: float
    y: float
    yaw: float = 0.0
    profile: Profile = field(default_factory=Profile)
    run: object | None = None  # 지금 하는 goal(world/runs.py)
    comm_up: bool = True
    removed: bool = False
    stuck: bool = False
    docked_at: str = ""
    has_basket: bool = True
    basket: dict = field(default_factory=dict)
    localizing_until: float = 0.0
    noise_m: float = 0.0  # 지금 보고 잡음. 주입으로 커질 수 있다
    jump_m: float = 0.0  # 보고를 한쪽으로 미는 거리(위치가 크게 튐)

    @property
    def pose(self) -> tuple[float, float]:
        return (self.x, self.y)

    def gap(self, other: SimRobot) -> float:
        return math.dist(self.pose, other.pose)
