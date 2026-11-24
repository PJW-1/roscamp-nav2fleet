"""가짜 Pode의 retreat 목표: 시작 검사와 틱마다 뒤로 가기."""
from __future__ import annotations

import math

from fleet_interfaces.action import Retreat

from fake_robots.nav_speed import nav_speed_mps
from fake_robots.pode_goal import _Goal, _project

# 실물 pode_fsm의 경로 물러나기와 같게(FollowPathReverse 속도, 이보다 짧게 남으면 FAILED)
REVERSE_SPEED_MPS = 0.05
REVERSE_MIN_M = 0.02


class PodeRetreat:
    def _start_retreat(self, t: float, goal: _Goal, request) -> None:
        """보는 방향 그대로 뒤로. 갈 길을 지나온 선(trail)으로 두고 Motion.back으로 간다.

        path가 없으면 곧게 distance_m을, 있으면 제 자리에서 가장 가까운 선분부터 끝 점까지 간다.
        따를 수 없는 path(연결 허용 거리 밖, 남은 길 0.02 m 미만)는 거절한다.
        """
        path = [(p.position.x, p.position.y) for p in request.path]
        if not self._begin_retreat(t, goal, float(request.distance_m),
                                   float(request.speed_mps), path):
            self._end(goal, False, Retreat.Result.REASON_FAILED)

    def _begin_retreat(self, t: float, goal: _Goal, distance: float, speed: float,
                       path) -> bool:
        """후진을 준비한다. 따를 수 없는 요청이면 False(order 조각도 이것을 쓴다)."""
        m = self._motion
        rest = math.inf
        speed = min(speed, nav_speed_mps())  # 시킨 속도가 커도 단일 출처의 최고 속도까지
        if path:
            i, rest = _project(m.x, m.y, path)
            ahead = path[i + 1:]
            speed = min(speed, REVERSE_SPEED_MPS)
        else:
            ahead = [(m.x - math.cos(m.yaw) * distance, m.y - math.sin(m.yaw) * distance)]
        length = sum(math.dist(a, b) for a, b in zip([(m.x, m.y), *ahead], ahead, strict=False))
        if (distance <= 0.0 or speed <= 0.0 or length <= 1e-9 or rest < REVERSE_MIN_M
                or (path and self._path_misses_me(t, path))):
            return False
        m.clear_target()
        m.trail = list(reversed(ahead))  # Motion.back은 trail 끝에서부터 되짚는다
        goal.left, goal.speed = length, speed
        return True

    def _retreat_step(self, t: float, dt: float, goal: _Goal) -> None:
        out = self._retreat_move(t, dt, goal)
        if out is not None:
            self._end(goal, *out)

    def _retreat_move(self, t: float, dt: float, goal: _Goal):
        """한 틱 뒤로 간다. 끝나면 (성공, 이유), 아니면 None."""
        radius = self._recovery.p.robot_radius_m
        wall = self._faults.behind(t)
        others = self._others_discs()
        # 자기 복구의 뒤 보기와 같은 검사(recovery.py). 남은 길 전체에서 몸이 닿는가
        if ((wall is not None and wall <= goal.left + radius)
                or self._recovery._something_behind(self._motion, goal.left, others)):
            return False, Retreat.Result.REASON_OBSTACLE_BEHIND
        step = min(goal.speed * dt, goal.left)
        self._motion.back(step)
        goal.left -= step
        if goal.left <= 1e-9:
            return True, ""
        return None
