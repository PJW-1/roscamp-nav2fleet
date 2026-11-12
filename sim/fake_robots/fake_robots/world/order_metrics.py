"""order 모델 표의 숫자: 이음매 공백, base 끝 기다림, 이동 한 건 시간, 정지 횟수, base 밖 진입.

이동(trip)은 관제의 것을 쓴다: robot_view의 route가 비지 않은 동안이 이동 한 건이다(잡을 구역을
기다리는 시간도 든다). 두 모델에서 같은 뜻이다.

- 이음매: 이동 안에서 조각 하나가 끝난 때부터 로봇의 실제 속도가 다시 MOVING_MPS 이상이 될
  때까지. order 모델이 꺼져 있으면 조각은 goal 하나(MoveCmd, RetreatCmd)이고, 그 끝이 이동의 끝
  지점이 아닐 때만 이음매다. 켜져 있으면 order goal 안에서 끝난 조각(done_seq가 늘고 order가
  이어짐)이다.
- base 끝 기다림: 조각이 끝났을 때 받은 base가 다 떨어져 update를 기다린 시간(찬 도크 앞의
  기다리는 자리, 못 잡은 구역 앞). 이음매 공백에 넣지 않고 따로 센다. 모델이 꺼져 있으면 같은
  기다림이 이동과 이동 사이에 있어서 어느 쪽에도 들지 않는다.
- 정지: 이동 안에서 실제 속도가 STOP_S 넘게 MOVING_MPS 아래인 구간(제자리 회전 포함).
- base 밖 진입: order goal을 달리는 로봇이 받은 base 조각들의 구역(과 출발한 구역)에서
  INSIDE_TOL_M 넘게 나간 횟수.
"""
from __future__ import annotations

import math

from .order_run import OrderRun
from .runs import DOCK, MOVE, RETREAT

MOVING_MPS = 0.01
STOP_S = 0.5
INSIDE_TOL_M = 0.01
END_M = 0.05  # 조각의 끝이 이동의 끝 지점에서 이 안이면 이음매가 아니다


class OrderMetrics:
    def __init__(self, site, robots):
        self.site = site
        self.seams: list[float] = []
        self.base_waits: list[float] = []
        self.trips: list[float] = []
        self.stops = 0
        self.outside_base = 0
        self._last = {name: r.pose for name, r in robots.items()}
        self._seam: dict[str, float] = {}  # 로봇 -> 이음매가 열린 시각
        self._at_base_end: set[str] = set()  # 열린 이음매가 base 끝의 기다림이다
        self._trip: dict[str, float] = {}  # 로봇 -> 이동이 시작된 시각
        self._still: dict[str, float] = {}  # 로봇 -> 이동 안에서 선 시각
        self._counted: set[str] = set()  # 이번 정지를 이미 셌다
        self._seq: dict[str, int] = {}  # 로봇 -> order run의 done_seq
        self._allowed: dict[str, set] = {}  # 로봇 -> order의 base 구역
        self._outside: set[str] = set()

    # ---- 세계가 알려 주는 것 ----

    def run_started(self, name: str, run, robot) -> None:
        if run.kind == DOCK:
            self._seam.pop(name, None)
            return
        if isinstance(run, OrderRun):
            here = self.site.zone_at(robot.x, robot.y)
            self._allowed[name] = {here} if here else set()
            self._seq[name] = 0
            self.base_given(name, run.queue)

    def base_given(self, name: str, segments) -> None:
        self._allowed.setdefault(name, set()).update(z for s in segments for z in s.zones)

    def run_ended(self, t: float, name: str, run, result, robot, view) -> None:
        if isinstance(run, OrderRun):
            self._allowed.pop(name, None)
            return
        route = (view or {}).get("route") or []
        if not result.success or run.kind not in (MOVE, RETREAT) or not route:
            return
        if math.dist(robot.pose, self.site.node_pose(route[-1])) > END_M:
            self._seam[name] = t

    # ---- 걸음마다 ----

    def step(self, t: float, dt: float, robots, views) -> None:
        for name, r in robots.items():
            last = self._last.get(name, r.pose)
            self._last[name] = r.pose
            moving = math.dist(last, r.pose) / dt >= MOVING_MPS
            run = r.run
            if isinstance(run, OrderRun):
                if run.done_seq > self._seq.get(name, 0) and (run.queue or run.horizon
                                                              or run.sub is not None):
                    self._seam[name] = t
                    (self._at_base_end.add if run.waiting else self._at_base_end.discard)(name)
                self._seq[name] = run.done_seq
                self._check_base(name, r)
            if moving and t - self._seam.get(name, t) > 1e-9:  # 연 걸음의 움직임은 세지 않는다
                waited = name in self._at_base_end
                self._at_base_end.discard(name)
                (self.base_waits if waited else self.seams).append(t - self._seam.pop(name))
            self._track_trip(t, name, bool((views.get(name) or {}).get("route")), moving)

    def _track_trip(self, t: float, name: str, active: bool, moving: bool) -> None:
        if not active:
            start = self._trip.pop(name, None)
            if start is not None:
                self.trips.append(t - start)
            self._seam.pop(name, None)
            self._still.pop(name, None)
            self._counted.discard(name)
            return
        self._trip.setdefault(name, t)
        if moving:
            self._still.pop(name, None)
            self._counted.discard(name)
            return
        since = self._still.setdefault(name, t)
        if t - since > STOP_S and name not in self._counted:
            self._counted.add(name)
            self.stops += 1

    def _check_base(self, name: str, r) -> None:
        allowed = self._allowed.get(name)
        if not allowed:
            return
        gap = min(self.site.distance_to_zone(r.x, r.y, z) for z in allowed)
        if gap > INSIDE_TOL_M:
            if name not in self._outside:
                self._outside.add(name)
                self.outside_base += 1
        else:
            self._outside.discard(name)

    # ---- 표 ----

    def seam_text(self) -> str:
        if not self.seams:
            return "-"
        return f"{sum(self.seams) / len(self.seams):.2f} / {max(self.seams):.2f}"

    def wait_text(self) -> str:
        if not self.base_waits:
            return "0"
        return f"{len(self.base_waits)} / {max(self.base_waits):.1f}"

    def trip_text(self) -> str:
        return f"{sum(self.trips) / len(self.trips):.1f}" if self.trips else "-"
