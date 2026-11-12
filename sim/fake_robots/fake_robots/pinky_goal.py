"""가짜 핑키의 목표 한 건(_Goal)과 꺾은선 투영(_project)."""
from __future__ import annotations

import math
import threading

from fake_robots.nav_goal_drive import NavGoalDrive

NAV_ABORT_DELAY_S = 0.2  # 해제 거리(0.2 m)를 지나기 전, 관제가 선을 늘리기 전에 중단한다


def _project(px: float, py: float, path) -> tuple[int, float]:
    """(px, py)에 가장 가까운 선분의 번호와 그 투영점부터 path 끝까지의 길이. 실물 pinky_fsm의
    Polyline.project처럼 거리가 같으면 앞 선분이다."""
    best, i, s_at, s = math.inf, 0, 0.0, 0.0
    for k, ((ax, ay), (bx, by)) in enumerate(zip(path, path[1:], strict=False)):
        seg = math.dist((ax, ay), (bx, by))
        f = 0.0 if seg == 0.0 else min(max(
            ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / (seg * seg), 0.0), 1.0)
        d = math.dist((px, py), (ax + f * (bx - ax), ay + f * (by - ay)))
        if d < best - 1e-12:
            best, i, s_at = d, k, s + f * seg
        s += seg
    return i, s - s_at


class _Goal:
    def __init__(self, kind: str, handle):
        self.kind = kind
        self.handle = handle
        self.done = threading.Event()
        self.success = False
        self.reason = ""
        self.abort_at: float | None = None
        self.abort_from = None  # 쓴 고장 일정
        self.aborting = False  # Nav2가 이 목표를 중단하고 있다(자기 복구 중에도 계속)
        self.end_at = 0.0
        self.start_at = 0.0
        self.error = (0.0, 0.0, 0.0)
        self.target_id = ""  # target_id of a move, dock_id of a dock (current_goal)
        self.left = 0.0  # retreat: 남은 거리
        self.speed = 0.0
        self.nav: NavGoalDrive | None = None  # move의 nav_goal 모드

    def finish(self, success: bool, reason: str = "") -> None:
        if not self.done.is_set():
            self.success, self.reason = success, reason
            self.done.set()
