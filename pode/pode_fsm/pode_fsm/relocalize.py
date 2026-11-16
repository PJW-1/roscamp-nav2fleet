# ---- 위치 다시 찾기: 스캔 일치율, 판정, 단계 기계 (ROS 없음) ----
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from pode_fsm.params import RelocalizeParams

# ---- 결과 사유와 단계 이름(Relocalize.action과 같은 값) ----
NOT_CONVERGED = "NOT_CONVERGED"
SCAN_MISMATCH = "SCAN_MISMATCH"
SPIN_BLOCKED = "SPIN_BLOCKED"
BUSY = "BUSY"
NO_LOCALIZER = "NO_LOCALIZER"
TIMEOUT = "TIMEOUT"
CANCELED = "CANCELED"
NOMOTION = "NOMOTION"
SPIN = "SPIN"
GLOBAL = "GLOBAL"

# ---- 바쁜 상태와 Nav2 Spin 시간 초과 코드 ----
BUSY_STATES = ("MOVING", "DOCKING", "RECOVERING", "BLOCKED")
SPIN_TIMEOUT_CODE = 701
FULL_TURN = 2.0 * math.pi


# ---- 지도 거리장과 추정 ----
class MapField(NamedTuple):
    res: float
    ox: float
    oy: float
    yaw: float
    dist: np.ndarray  # (높이, 너비) 가장 가까운 벽 칸까지 거리(m), 상한 너머는 inf
    occ: np.ndarray | None = None  # (높이, 너비) 벽 칸이면 참. 있으면 점-칸 거리를 정확히 잰다


class Estimate(NamedTuple):
    x: float
    y: float
    yaw: float
    std_xy: float
    std_yaw: float
    match: float


# ---- 단계 기계가 내는 명령 ----
@dataclass(frozen=True)
class Nomotion:
    pass


@dataclass(frozen=True)
class Spin:
    yaw: float
    allowance_s: float


@dataclass(frozen=True)
class CancelSpin:
    pass


@dataclass(frozen=True)
class GlobalInit:
    pass


@dataclass(frozen=True)
class Done:
    success: bool
    reason: str
    estimate: Estimate | None


# ---- 점유 격자의 거리 변환(상한까지 정확한 유클리드 거리) ----
def distance_field(res: float, width: int, height: int, data: Sequence[int],
                   occupied: int = 65, cap_m: float = 0.3) -> np.ndarray:
    occ = np.asarray(data, dtype=np.int16).reshape(height, width) >= occupied
    dist = np.full((height, width), np.inf)
    ys, xs = np.nonzero(occ)
    if len(ys) == 0:
        return dist
    r = int(math.ceil(cap_m / res))
    # 상한 원 안의 칸 이동마다 벽을 옮겨 가장 작은 거리를 남긴다
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            d = math.hypot(dx, dy) * res
            if d > cap_m + 1e-9:
                continue
            ty, tx = ys + dy, xs + dx
            ok = (ty >= 0) & (ty < height) & (tx >= 0) & (tx < width)
            np.minimum.at(dist, (ty[ok], tx[ok]), d)
    return dist


# ---- 지도 하나로 거리장과 점유 격자를 함께 만든다 ----
def map_field(res: float, width: int, height: int, data: Sequence[int], ox: float, oy: float,
              yaw: float, occupied: int, tol: float) -> MapField:
    occ = np.asarray(data, dtype=np.int16).reshape(height, width) >= occupied
    # 칸 중심 거리 상한: tol 안의 점이 볼 수 있는 벽 칸을 모두 담는다
    dist = distance_field(res, width, height, data, occupied, tol + 2.0 * res)
    return MapField(res, ox, oy, yaw, dist, occ)


# ---- 레이저 거리에서 유효한 점(레이저 좌표) ----
def scan_points(ranges: Sequence[float], angle_min: float, angle_inc: float,
                range_min: float, range_max: float) -> np.ndarray:
    r = np.asarray(ranges, dtype=float)
    if r.size == 0:
        return np.zeros((0, 2))
    a = angle_min + angle_inc * np.arange(r.size)
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)


# ---- 스캔 일치율: 추정 자세로 옮긴 점 중 벽 칸 tol 안의 비율 ----
def scan_match(field: MapField, pts: np.ndarray, pose: Sequence[float],
               laser: Sequence[float], tol: float) -> float:
    if len(pts) == 0:
        return 0.0
    x, y, yaw = pose
    lx, ly, lyaw = laser
    c, s = math.cos(yaw), math.sin(yaw)
    # 레이저 자세(지도 좌표)
    gx, gy, ga = x + c * lx - s * ly, y + s * lx + c * ly, yaw + lyaw
    ca, sa = math.cos(ga), math.sin(ga)
    mx = gx + ca * pts[:, 0] - sa * pts[:, 1]
    my = gy + sa * pts[:, 0] + ca * pts[:, 1]
    # 지도 원점 좌표의 칸 번호로
    dx, dy = mx - field.ox, my - field.oy
    co, so = math.cos(-field.yaw), math.sin(-field.yaw)
    u, v = co * dx - so * dy, so * dx + co * dy
    i = np.floor(u / field.res).astype(int)
    j = np.floor(v / field.res).astype(int)
    h, w = field.dist.shape
    inside = (i >= 0) & (i < w) & (j >= 0) & (j < h)
    hit = np.zeros(len(pts), dtype=bool)
    if field.occ is None:
        hit[inside] = field.dist[j[inside], i[inside]] <= tol + 1e-9
        return float(hit.sum()) / len(pts)
    # 거리장으로 먼저 거른다: 칸 중심 거리가 tol + 대각선보다 멀면 어떤 벽 칸도 tol 밖이다
    res = field.res
    cand = np.nonzero(inside)[0]
    cand = cand[field.dist[j[cand], i[cand]] <= tol + res * math.sqrt(2.0) + 1e-9]
    ci, cj, cu, cv = i[cand], j[cand], u[cand], v[cand]
    found = np.zeros(len(cand), dtype=bool)
    r = int(math.ceil(tol / res)) + 1
    # 남은 점은 주변 벽 칸 네모까지 거리를 모두 잰다
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            ti, tj = ci + dx, cj + dy
            ok = (ti >= 0) & (ti < w) & (tj >= 0) & (tj < h)
            ok[ok] = field.occ[tj[ok], ti[ok]]
            gx = np.maximum(np.maximum(ti * res - cu, cu - (ti + 1) * res), 0.0)
            gy = np.maximum(np.maximum(tj * res - cv, cv - (tj + 1) * res), 0.0)
            found |= ok & (np.hypot(gx, gy) <= tol + 1e-9)
    hit[cand] = found
    return float(hit.sum()) / len(pts)


# ---- 공분산(6x6 행 우선)에서 위치·방향 표준편차 ----
def std_from_covariance(cov: Sequence[float]) -> tuple[float, float]:
    # 계약의 정의: sqrt(max(var_x, var_y))
    return math.sqrt(max(0.0, cov[0], cov[7])), math.sqrt(max(0.0, cov[35]))


# ---- 판정과 실패 사유 ----
def is_good(e: Estimate, p: RelocalizeParams) -> bool:
    return (e.std_xy < p.max_std_xy_m and e.std_yaw < p.max_std_yaw_rad
            and e.match >= p.min_scan_match)


def failure_reason(e: Estimate | None, p: RelocalizeParams) -> str:
    if e is None:
        return TIMEOUT
    if e.std_xy >= p.max_std_xy_m or e.std_yaw >= p.max_std_yaw_rad:
        return NOT_CONVERGED
    if e.match < p.min_scan_match:
        return SCAN_MISMATCH
    # 좋았지만 이어지지 못했다: 시간이 모자랐다
    return TIMEOUT


def is_busy(state: str, relocalizing: bool) -> bool:
    return relocalizing or state in BUSY_STATES


# ---- 단계 기계: 무이동 → 회전 → 전역 ----
class Relocalizer:
    def __init__(self, p: RelocalizeParams, allow_spin: bool, allow_global: bool,
                 timeout_s: float, now: float):
        self.p = p
        self.allow_spin, self.allow_global = allow_spin, allow_global
        total = timeout_s if timeout_s > 0 else p.default_timeout_s
        self.end = now + total
        # 단계별 끝 시각. 뒤 단계가 없으면 전체 끝까지 쓴다
        self.nomotion_end = (now + p.nomotion_share * total
                             if allow_spin or allow_global else self.end)
        self.spin_end = (now + (p.nomotion_share + p.spin_share) * total
                         if allow_global else self.end)
        self.stage = NOMOTION
        self.last: Estimate | None = None
        self.result: Done | None = None
        self.fresh_updates = 0  # 판정에 쓴 새 추정 수
        self._t0 = now
        self._stable = 0
        self._counting = True
        self._spinning = False
        self._spin_blocked = False
        self._waiting_init = False
        self._last_req = -math.inf

    @property
    def spinning(self) -> bool:
        return self._spinning

    # ---- 시작: 위치 추정기가 없으면 바로 끝 ----
    def start(self, localizer_ready: bool) -> list:
        if not localizer_ready:
            self.stage = ""  # 시작 전 거절
            return self._finish(False, NO_LOCALIZER)
        self._last_req = self._t0
        return [Nomotion()]

    # ---- 새 스캔: 무이동 갱신 요청 ----
    def scan(self, now: float) -> list:
        if self.result is not None or self._waiting_init:
            return []
        if now - self._last_req < self.p.nomotion_period_s:
            return []
        self._last_req = now
        return [Nomotion()]

    # ---- 새 AMCL 추정: K번 이어지면 성공 ----
    def estimate(self, now: float, e: Estimate) -> list:
        # 시간이 지난 뒤의 추정은 쓰지 않는다(tick이 끝낸다)
        if self.result is not None or now >= self.end:
            return []
        self.last = e
        if not self._counting:
            return []
        self.fresh_updates += 1
        self._stable = self._stable + 1 if is_good(e, self.p) else 0
        if self._stable >= self.p.stable_updates:
            return self._finish(True, "")
        return []

    # ---- 주기 처리: 전체와 단계 시간 ----
    def tick(self, now: float) -> list:
        if self.result is not None:
            return []
        if now >= self.end:
            return self._finish(False, failure_reason(self.last, self.p))
        if self.stage == NOMOTION and now >= self.nomotion_end:
            if self.allow_spin:
                self.stage = SPIN
                return self._spin(now, self.spin_end)
            return self._start_global()
        if self.stage == SPIN and not self._spinning and now >= self.spin_end:
            return self._start_global()
        return []

    # ---- Nav2 Spin 결과 ----
    def spin_result(self, now: float, outcome: str, code: int) -> list:
        if self.result is not None or not self._spinning:
            return []
        self._spinning = False
        if outcome == "canceled":
            return []
        # 거절이나 시간 초과 아닌 중단(충돌 등)은 돌 수 없다는 뜻
        if outcome == "rejected" or (outcome != "succeeded" and code != SPIN_TIMEOUT_CODE):
            self._spin_blocked = True
            if self.stage == SPIN:
                if self.allow_global:
                    return self._start_global()
                return self._finish(False, SPIN_BLOCKED)
        # 회전이 끝났다: 여기서부터 판정한다
        self._count_from_now()
        return []

    # ---- 전역 재초기화 서비스 응답 ----
    def global_ready(self, now: float, ok: bool) -> list:
        if self.result is not None or not self._waiting_init:
            return []
        self._waiting_init = False
        if not ok:
            return self._finish(False, NO_LOCALIZER)
        self._last_req = -math.inf  # 다음 스캔에서 바로 갱신을 부른다
        if self.allow_spin and not self._spin_blocked:
            return self._spin(now, self.end)
        self._count_from_now()
        return []

    # ---- 취소 ----
    def cancel(self, now: float) -> list:
        if self.result is not None:
            return []
        return self._finish(False, CANCELED)

    # ---- 단계 시작과 끝 ----
    def _spin(self, now: float, stage_end: float) -> list:
        self._spinning = True
        self._counting = False
        self._stable = 0
        # Jazzy Spin은 0을 무제한으로 본다
        allowance = max(self.p.min_spin_allowance_s, stage_end - now - self.p.settle_s)
        return [Spin(FULL_TURN, allowance)]

    def _start_global(self) -> list:
        if not self.allow_global:
            return self._finish(False, failure_reason(self.last, self.p))
        self.stage = GLOBAL
        self._counting = False
        self._stable = 0
        self._waiting_init = True
        return [GlobalInit()]

    def _count_from_now(self) -> None:
        self._counting = True
        self._stable = 0

    def _finish(self, success: bool, reason: str) -> list:
        cmds: list = [CancelSpin()] if self._spinning else []
        self._spinning = False
        self.result = Done(success, reason, self.last)
        return cmds + [self.result]
