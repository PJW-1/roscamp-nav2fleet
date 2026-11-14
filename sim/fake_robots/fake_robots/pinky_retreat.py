"""가짜 핑키의 retreat 목표: 시작 검사와 틱마다 뒤로 가기."""
from __future__ import annotations

import math

from fleet_interfaces.action import Retreat

from fake_robots.pinky_goal import _Goal, _project

# 실물 pinky_fsm의 경로 물러나기와 같게(FollowPathReverse 속도, 이보다 짧게 남으면 FAILED)
REVERSE_SPEED_MPS = 0.10
REVERSE_MIN_M = 0.02


class PinkyRetreat:
    def _start_retreat(self, t: float, goal: _Goal, request) -> None:
        """보는 방향 그대로 뒤로. 갈 길을 지나온 선(trail)으로 두고 Motion.back으로 간다.

        path가 없으면 뒤로 곧게 distance_m을 시킨 속도로. 있으면 실물 pinky_fsm처럼 제 자리에서
        가장 가까운 선분부터 끝 점까지 차례로, REVERSE_SPEED_MPS(0.10 m/s)를 넘지 않게 간다.
        로봇 자리에서 시작하지 않는 path(이동의 path와 같은 연결 허용 거리)나 제 자리를 투영한
        점부터 끝까지가 0.02 m보다 짧은 path는 따르지 않는다. 투영은 실물(Polyline.project)과 같이 같으면 앞 선분이다.
        """
        distance, speed = float(request.distance_m), float(request.speed_mps)
        path = [(p.position.x, p.position.y) for p in request.path]
        m = self._motion
        rest = math.inf
        if path:
            i, rest = _project(m.x, m.y, path)
            ahead = path[i + 1:]
            speed = min(speed, REVERSE_SPEED_MPS)
        else:
            ahead = [(m.x - math.cos(m.yaw) * distance, m.y - math.sin(m.yaw) * distance)]
        length = sum(math.dist(a, b) for a, b in zip([(m.x, m.y), *ahead], ahead, strict=False))
        if (distance <= 0.0 or speed <= 0.0 or length <= 1e-9 or rest < REVERSE_MIN_M
                or (path and self._path_misses_me(t, path))):
            self._end(goal, False, Retreat.Result.REASON_FAILED)
            return
        m.clear_target()
        m.trail = list(reversed(ahead))  # Motion.back은 trail 끝에서부터 되짚는다
        goal.left, goal.speed = length, speed

    def _retreat_step(self, t: float, dt: float, goal: _Goal) -> None:
        radius = self._recovery.p.robot_radius_m
        wall = self._faults.behind(t)
        others = [(x, y, radius) for x, y in list(self._others.values())]
        # 자기 복구의 뒤 보기와 같은 검사(recovery.py). 남은 길 전체에서 몸이 닿는가
        if ((wall is not None and wall <= goal.left + radius)
                or self._recovery._something_behind(self._motion, goal.left, others)):
            self._end(goal, False, Retreat.Result.REASON_OBSTACLE_BEHIND)
            return
        step = min(goal.speed * dt, goal.left)
        self._motion.back(step)
        goal.left -= step
        if goal.left <= 1e-9:
            self._end(goal, True)
