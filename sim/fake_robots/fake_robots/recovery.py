"""앞 물체 앞에서 서고, 짧게 후진해 다시 가 보고, 그래도 막히면 BLOCKED. ROS를 쓰지 않는다.

pode_fsm과 같은 순서: block_report_s 서 있으면 시도를 쓰고 RECOVERING, 뒤로 물러났다 남은 path를 다시 간다.
시도를 다 쓰고 또 서면 BLOCKED다. 물체는 시각으로만 있어 처음 선 자리를 물체 앞으로 본다(blocked 고장).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from fake_robots.motion import path_gap

# 관제는 꼭짓점을 release_distance_m 지나면 뒤 구역을 놓는다(fleet_params).
# 후진한 중심도 그 꼭짓점에서 0.23 m 앞에 둔다.
RELEASE_DISTANCE_M = 0.2
ROBOT_RADIUS_M = 0.08
BACKUP_CLEARANCE_M = 0.03
BACKUP_MARGIN_M = RELEASE_DISTANCE_M - ROBOT_RADIUS_M + BACKUP_CLEARANCE_M  # 0.15


@dataclass(frozen=True)
class RecoveryParams:
    block_report_s: float = 1.0  # 물체 앞에서 이만큼 서 있으면 시도, 시도를 다 썼으면 BLOCKED
    block_clear_s: float = 1.0  # 물체가 이만큼 안 보이면 사라진 것이다
    # follow_path는 쓰지 않아 맞추지 않았다
    recovery_attempts: int = 2
    backup_max_m: float = 0.15
    robot_radius_m: float = ROBOT_RADIUS_M
    backup_margin_m: float = BACKUP_MARGIN_M
    backup_min_m: float = 0.02
    backup_speed_mps: float = 0.08
    backup_timeout_s: float = 5.0
    recovery_pass_m: float = 0.05  # 시도를 시작한 자리에서 이만큼 더 가면 RECOVERING 끝
    recovery_reset_m: float = 0.3  # 처음 센 시도의 자리에서 이만큼 멀어지면 시도 수 0

    def backup_distance(self, s_ref: float) -> float:
        """마지막 꼭짓점에서 s_ref만큼 왔을 때의 후진 거리. 0이면 후진하지 않는다."""
        d = min(self.backup_max_m, s_ref - self.robot_radius_m - self.backup_margin_m)
        return d if d >= self.backup_min_m - 1e-9 else 0.0


@dataclass
class _BackUp:
    left: float
    until: float
    fixed: dict  # 뒤 물체: 처음 보인 거리 -> (x, y, 반지름)


class ObstacleRecovery:
    """한 로봇의 앞 물체 처리. 노드는 매 틱 watch()를, 이동 목표가 있으면 drive()를 부른다."""

    def __init__(self, params: RecoveryParams | None = None):
        self.p = params or RecoveryParams()
        self.blocked = False
        self.used = 0
        self._hold: float | None = None  # 물체 앞에 선 자리(path 위 거리)
        self._clear_at: float | None = None
        self._stood_at: float | None = None
        self._attempt_s: float | None = None  # 지금 시도를 시작한 자리(path 위 거리)
        self._abort_s: float | None = None  # Nav2가 처음 중단한 자리. 다시 가도 넘지 못한다
        self._first_at: tuple[float, float] | None = None
        self._backup: _BackUp | None = None

    @property
    def recovering(self) -> bool:
        return self._attempt_s is not None

    @property
    def backing_up(self) -> bool:
        return self._backup is not None

    def watch(self, t: float, ahead: bool, driving: bool, motion) -> None:
        """물체가 보이는지. 이동 중에 나타나면 그 자리가 물체 앞이다."""
        if ahead:
            self._clear_at = None
            if self._hold is None and driving:
                self._hold = motion.travelled
            return
        if self._hold is None:
            return
        if self._clear_at is None:
            self._clear_at = t
        if t - self._clear_at >= self.p.block_clear_s:
            self._hold = self._clear_at = self._stood_at = None
            self.blocked = False

    def drive(self, t: float, dt: float, motion, speed: float, others=(),
              wall_behind_m: float | None = None) -> None:
        """이동 목표 한 틱. others: 다른 로봇 (x, y, 반지름). wall_behind_m: 뒤 물체 거리."""
        if self._backup is not None:
            self._back_step(t, dt, motion, others, wall_behind_m)
            return
        ends = [s for s in (self._hold, self._abort_s) if s is not None]
        limit = max(0.0, min(ends) - motion.travelled) if ends else math.inf
        motion.step(dt, speed, limit)
        s = motion.travelled
        if self._attempt_s is not None and s >= self._attempt_s + self.p.recovery_pass_m - 1e-9:
            self._attempt_s = None
        if self._hold is not None and s >= self._hold - 1e-9:
            if self._stood_at is None:
                self._stood_at = t
            if not self.blocked and t - self._stood_at >= self.p.block_report_s - 1e-9:
                self._trigger(t, motion, wall_behind_m)
        else:
            self._stood_at = None
        if (self._attempt_s is None and self._first_at is not None and math.dist(
                self._first_at, (motion.x, motion.y)) >= self.p.recovery_reset_m - 1e-9):
            self.used, self._first_at = 0, None

    def nav_aborted(self, t: float, motion, wall_behind_m: float | None = None) -> bool:
        """FollowPath가 중단됐다(명세 2b, pode_fsm core.py nav_result). 시도가 남았으면 시도를
        시작하고 False, 다 썼으면 True(목표를 NAV_ABORTED로 끝낸다).

        FakePode의 nav_abort는 'Nav2가 계속 중단한다'이다. 다시 가도 처음 중단한 자리를 넘지
        못하고, 노드가 정한 때 또 중단한다.
        """
        if self.used >= self.p.recovery_attempts:
            return True
        if self._abort_s is None:
            self._abort_s = motion.travelled
        self._trigger(t, motion, wall_behind_m)
        return False

    def _trigger(self, t: float, motion, wall_behind_m: float | None) -> None:
        if self.used >= self.p.recovery_attempts:
            self.blocked, self._attempt_s = True, None
            return
        self.used += 1
        if self._first_at is None:
            self._first_at = (motion.x, motion.y)
        self._attempt_s = motion.travelled
        self._stood_at = t  # 후진하지 않으면 여기서 다시 센다
        d = self.p.backup_distance(motion.since_vertex)
        if d <= 0.0:
            return
        self._backup = _BackUp(d, t + self.p.backup_timeout_s, {})
        self._stood_at = None

    def _back_step(self, t: float, dt: float, motion, others, wall_behind_m) -> None:
        b = self._backup
        if wall_behind_m is not None and wall_behind_m not in b.fixed:
            # 뒤 물체는 처음 보인 때 그 자리에 놓인다(로봇에서 path를 따라 wall_behind_m 뒤)
            x, y = motion.way_back(wall_behind_m)[-1]
            b.fixed[wall_behind_m] = (x, y, 0.0)
        if (t >= b.until or b.left <= 1e-9
                or self._something_behind(motion, b.left, [*others, *b.fixed.values()])):
            self._backup = None  # 그 자리에서 멈춘다. 시도는 이어 간다
            return
        step = min(self.p.backup_speed_mps * dt, b.left)
        motion.back(step)
        b.left -= step
        if b.left <= 1e-9:
            self._backup = None

    def _something_behind(self, motion, left: float, objects) -> bool:
        """남은 후진 길에서 몸(반지름)이 물체에 닿는가(Nav2 BackUp처럼 남은 길 전체를 본다).

        지금 자리보다 가까워지지 않는 물체(로봇 바로 앞 등)는 후진을 막지 않는다.
        """
        way = motion.way_back(left)
        for x, y, radius in objects:
            gap = path_gap(x, y, way)
            if (gap <= self.p.robot_radius_m + radius
                    and gap < math.dist((x, y), (motion.x, motion.y)) - 1e-9):
                return True
        return False

    def restart(self) -> None:
        """목표가 끝났거나 바뀌었다. 후진과 RECOVERING을 멈추고 시도 수는 둔다.

        물체가 아직 보이면 새 목표는 지금 자리에서 서 있는다(path 위 거리 0).
        """
        self._backup = None
        self._attempt_s = self._abort_s = None
        self._stood_at = None
        if self._hold is not None:
            self._hold = 0.0

    def succeeded(self) -> None:
        self.used, self._first_at = 0, None

    def forget(self) -> None:
        """로봇을 코스에서 들어냈다. 물체도 시도도 잊는다."""
        self.__init__(self.p)
