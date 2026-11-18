"""실물 두 대 시험용 PC 도킹 대역(real_dock_wait)의 판정. ROS 없이 돈다.

pode_dock이 아직 없어서 실물 pode_fsm은 도킹을 DOCK_UNAVAILABLE로 실패시킨다. 주문을 시뮬처럼
끝까지 돌려 보려고, 로봇이 DOCK_STUB=1(로봇의 dock 서버를 /<ns>/dock_fsm으로 비킴)로 떴을 때
PC가 /<ns>/dock을 맡는다. 로봇을 움직이지 않는다: 관제의 move_to가 도크 지점에 데려다 놓은
로봇을 시나리오의 dock.duration_s만큼 기다린 뒤, 로봇이 보고한 자세(AMCL)가 도크 지점에서
REACH_M 안이면 성공이다. 벽시계로만 돈다(시뮬 시계면 거부).
"""
from __future__ import annotations

import math
import re

REACH_M = 0.15  # 시뮬 도크 대역(fleet_gz_sim.dock_stand_in)과 같은 기준
STATE_MAX_AGE_S = 2.0  # 이보다 오래된 state는 믿지 않는다
_NAME = re.compile(r"^[a-z_][a-z0-9_]*$")


def dock_phase(elapsed: float, duration: float) -> str:
    done = elapsed / max(duration, 1e-6)
    return "APPROACH" if done < 0.5 else "ALIGN" if done < 1.0 else "DONE"


def dock_done(elapsed: float, duration: float) -> bool:
    return elapsed >= duration


def dock_outcome(pose, dock_xy, reach_m: float = REACH_M) -> tuple[bool, float, float, float]:
    """(성공, x 오차 cm, y 오차 cm, 방향 오차 deg). 지도에 도크 방향이 없어 방향은 0이다."""
    ex, ey = pose[0] - dock_xy[0], pose[1] - dock_xy[1]
    return math.hypot(ex, ey) <= reach_m + 1e-9, ex * 100.0, ey * 100.0, 0.0


def pose_fresh(received_at: float | None, now: float, max_age: float = STATE_MAX_AGE_S) -> bool:
    return received_at is not None and now - received_at <= max_age


def dock_wait_problems(dock_robots, fake_robots, *, sim_clock: bool) -> list[str]:
    """켜면 안 되는 까닭. 비면 켜도 된다."""
    found = []
    if sim_clock:
        found.append("use_sim_clock is true: the real dock wait runs only on the wall clock "
                     "with real robots (Gazebo has sim_dock_stub)")
    bad = [r for r in dock_robots if not _NAME.match(r)]
    if bad:
        found.append(f"bad robot names {bad}")
    both = sorted(set(dock_robots) & set(fake_robots))
    if both:
        found.append(f"{both} are fake robots, which serve their own dock")
    return found
