"""시뮬 전용 도크 대역(sim_dock_stub)의 판정과 켜도 되는지 검사. ROS 없이 돈다.
로봇은 움직이지 않고, 도크 지점에 놓인 실제 위치를 재서 APPROACH, ALIGN, DONE 피드백과 결과를 낸다.
Gazebo 시각(use_sim_time)과 isolation.py를 통과한 망에서만 켜진다.
"""
from __future__ import annotations

import math
from collections.abc import Mapping
from pathlib import Path

from fleet_gz_sim.isolation import problems as isolation_problems

REACH_M = 0.15  # 성공 결과의 위치 기준(R4)과 같다


def dock_phase(elapsed: float, duration: float) -> str:
    done = elapsed / max(duration, 1e-6)
    return "APPROACH" if done < 0.5 else "ALIGN" if done < 1.0 else "DONE"


def dock_outcome(pose, dock_xy, reach_m: float = REACH_M) -> tuple[bool, float, float, float]:
    """(성공, x 오차 cm, y 오차 cm, 방향 오차 deg). 오차는 지도 좌표다.

    레인 지도에는 도크 방향이 없어서 방향 오차는 재지 않는다(0)."""
    ex, ey = pose[0] - dock_xy[0], pose[1] - dock_xy[1]
    return math.hypot(ex, ey) <= reach_m + 1e-9, ex * 100.0, ey * 100.0, 0.0


class ClockWatch:
    """limit_s 실제 초 동안 Gazebo 시각이 멈췄는지 알려 준다."""

    def __init__(self, limit_s: float):
        self.limit_s = limit_s
        self._sim: float | None = None
        self._since = 0.0

    def stalled(self, sim_now: float, wall_now: float) -> bool:
        if self._sim is None or sim_now != self._sim:
            self._sim, self._since = sim_now, wall_now
            return False
        return wall_now - self._since > self.limit_s


def dock_done(elapsed: float, duration: float) -> bool:
    """도크 목표를 끝내도 되는지: duration_s가 지났고 목표 이후 Gazebo 시각이 움직였어야 한다."""
    return elapsed > 0.0 and elapsed >= duration


def stub_may_answer(ros_time_active: bool, now_s: float) -> bool:
    """Gazebo 시각이 흐르는 동안에만 도크 목표에 답한다."""
    return ros_time_active and now_s > 0.0


def dock_stub_problems(use_sim_time: bool, env: Mapping[str, str], *, rmw: str | None = None,
                       workdir: Path | None = None) -> list[str]:
    """도크 대역을 켜면 안 되는 이유 목록. 비면 켜도 된다."""
    found = [] if use_sim_time else ["use_sim_time is false: the dock stub runs only on "
                                     "Gazebo time"]
    return found + isolation_problems(env, rmw=rmw, workdir=workdir)


def require_dock_stub_allowed(use_sim_time: bool, env: Mapping[str, str], *,
                              rmw: str | None = None, workdir: Path | None = None) -> None:
    found = dock_stub_problems(use_sim_time, env, rmw=rmw, workdir=workdir)
    if found:
        raise RuntimeError("sim_dock_stub refused:\n  " + "\n  ".join(found))
