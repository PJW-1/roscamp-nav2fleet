# ---- 주문(order) 모델을 더한 상태 기계 OrderFsm ----
from __future__ import annotations

import math
from collections.abc import Callable, Hashable, Sequence
from dataclasses import dataclass

from pinky_fsm.core import FsmParams, PinkyFsm, _Move
from pinky_fsm.geometry import Point, Polyline, Pose
from pinky_fsm.local_yield import NO_PRIORITY
from pinky_fsm.nav_goal import MODE_NAV_GOAL, NavGoal
from pinky_fsm.order_queue import MODE_RETREAT, OrderGoal, OrderQueue, Seg, segments_problem
from pinky_fsm.results import (  # noqa: F401 - 결과 이름은 여기서도 내보낸다
    CANCELED,
    HEARTBEAT_LOST,
    HORIZON_INVALID,
    IDLE,
    MOVING,
    NAV_ABORTED,
    ORDER_UPDATE_REJECTED,
    SUPERSEDED,
    Finish,
)

# ---- 주문 설정값과 구간 묶음 키 ----
THROUGH = "FollowPathThrough"
HorizonCheck = Callable[[Seg], str]


@dataclass(frozen=True)
class OrderParams:
    join_m: float = 0.02
    pass_m: float = 0.15


@dataclass(frozen=True)
class _SegKey:
    order_no: int
    run_no: int


# ---- OrderFsm 초기화와 상태, 피드백, 주문 조회 ----
class OrderFsm(PinkyFsm):
    def __init__(self, params: FsmParams, order_params: OrderParams | None = None,
                 horizon_check: HorizonCheck | None = None, robot_id: str = ""):
        super().__init__(params, robot_id)
        self.op = order_params or OrderParams()
        self.horizon_check = horizon_check
        self._q: OrderQueue | None = None
        self._owner: Hashable | None = None
        self._run: list[Seg] = []
        self._key: _SegKey | None = None
        self._bounds: list[tuple[float, int]] = []
        self._waiting = False
        self._warning = ""
        self._orders = 0
        self._runs = 0

    def state(self) -> str:
        s = super().state()
        return MOVING if self._waiting and s == IDLE else s

    def current_goal(self) -> str:
        return self._q.last_node if self._waiting else super().current_goal()

    def is_running(self, goal_id: Hashable) -> bool:
        if self._q is not None:
            return goal_id == self._owner
        return super().is_running(goal_id)

    def order_feedback(self) -> tuple[int, bool, str]:
        if self._q is None:
            return 0, False, ""
        return self._q.done, self._waiting, self._warning

    def order_view(self) -> tuple[list[Seg], list[Seg]]:
        if self._q is None:
            return [], []
        done = self._q.done
        base = [s for s in self._run if s.seq_id > done] + list(self._q.pending)
        return base, list(self._q.horizon)

    # ---- 이동 요청으로 새 주문 시작과 주문 갱신 이어 붙이기 ----
    def move(self, goal_id: Hashable, now: float, target_id: str, target: Pose,
             path: Sequence[Point], pose: Sequence[float] | None,
             final_goal: bool = True, mode: str = "", order: OrderGoal | None = None,
             priority: int = NO_PRIORITY) -> list:
        if order is None or not order.order_id:
            return self._close(CANCELED) + self._own(
                super().move(goal_id, now, target_id, target, path, pose, final_goal, mode,
                             priority), now, pose)
        self._alive = max(self._alive, now)
        self._priority = int(priority)
        if order.update_id > 0:
            return self._update(goal_id, now, order, pose)
        cmds = self._close(CANCELED)
        if pose is None or segments_problem(order.base) or (order.base
                                                           and order.base[0].seq_id != 1):
            return (cmds + self._own(self._end_active(CANCELED, stop=True), now, pose)
                    + [Finish(goal_id, False, NAV_ABORTED)])
        self._q, self._owner = OrderQueue(order), goal_id
        self._orders += 1
        self._check_horizon()
        return cmds + self._own(self._advance(now, pose), now, pose)

    def _update(self, goal_id: Hashable, now: float, order: OrderGoal, pose) -> list:
        if self._q is None:
            return [Finish(goal_id, False, f"{ORDER_UPDATE_REJECTED}: no order is running")]
        problem = self._q.stitch(order)
        if problem:
            return [Finish(goal_id, False, f"{ORDER_UPDATE_REJECTED}: {problem}", self._q.done)]
        old, self._owner = self._owner, goal_id
        cmds = [Finish(old, False, SUPERSEDED, self._q.done)]
        self._check_horizon()
        more = self._advance(now, pose) if self._waiting else self._extend(now, pose)
        return cmds + self._own(more, now, pose)

    # ---- 도킹, 물러나기, 취소 요청 처리 ----
    def dock(self, goal_id: Hashable, now: float, dock_id: str) -> list:
        return self._close(CANCELED) + self._own(super().dock(goal_id, now, dock_id), now,
                                                 self._pose)

    def retreat(self, goal_id: Hashable, now: float, distance_m: float, speed_mps: float,
                path: Sequence[Sequence[float]] = (), pose: Sequence[float] | None = None) -> list:
        return self._close(CANCELED) + self._own(
            super().retreat(goal_id, now, distance_m, speed_mps, path, pose), now, pose)

    def cancel(self, goal_id: Hashable, now: float) -> list:
        if self._q is not None and goal_id == self._owner:
            return self._end_order(False, CANCELED) + self._own(
                self._end_active(CANCELED, stop=True), now, self._pose)
        return self._own(super().cancel(goal_id, now), now, self._pose)

    # ---- Nav2 결과 콜백을 주문 처리로 감싸기 ----
    def nav_result(self, seq, outcome, now, pose, obstacle, error_code=0) -> list:
        return self._own(super().nav_result(seq, outcome, now, pose, obstacle, error_code),
                         now, pose)

    def nav_lost(self, seq, now, pose, obstacle) -> list:
        return self._own(super().nav_lost(seq, now, pose, obstacle), now, pose)

    def nav_cancel_accepted(self, seq, now) -> list:
        return self._own(super().nav_cancel_accepted(seq, now), now, self._pose)

    def backup_result(self, seq, outcome, now, pose, error_code=0) -> list:
        return self._own(super().backup_result(seq, outcome, now, pose, error_code), now, pose)

    def nav_goal_result(self, seq, outcome, now, pose, lost=False) -> list:
        return self._own(super().nav_goal_result(seq, outcome, now, pose, lost), now, pose)

    def nav_goal_cancel_accepted(self, seq, now) -> list:
        return self._own(super().nav_goal_cancel_accepted(seq, now), now, self._pose)

    # ---- 주기 처리와 통신 끊김 시 정지 여부 판단 ----
    def tick(self, now, pose, speed, obstacle) -> list:
        if self._heartbeat_lost(now) and self._stops_on_heartbeat_loss():
            self._count_bounds(pose, track=True)
        cmds = self._own(super().tick(now, pose, speed, obstacle), now, pose)
        cmds += self._own(self._order_tick(now, pose), now, pose)
        self._lost_contact = False
        return cmds

    def _stops_on_heartbeat_loss(self) -> bool:
        if self.p.comm_loss_policy == "stop":
            return True
        g = self._active
        if self._key is None or g is None or g.goal_id != self._key:
            return True
        return (self._blocked or getattr(g, "blocked", False)
                or getattr(g, "waiting_since", None) is not None)

    # ---- 지나간 구간 경계 세기와 주문 주기 감시 ----
    def _count_bounds(self, pose, track: bool = False) -> None:
        g = self._active
        if self._q is None or not isinstance(g, _Move) or g.goal_id != self._key:
            return
        if track and pose is not None:
            g.s = self._track(g, pose)
        while self._bounds and g.s >= self._bounds[0][0]:
            self._q.done = self._bounds.pop(0)[1]

    def _order_tick(self, now: float, pose) -> list:
        if self._q is None:
            return []
        if self._waiting:
            return self._end_order(False, HEARTBEAT_LOST) if self._heartbeat_lost(now) else []
        g = self._active
        if isinstance(g, _Move) and g.goal_id == self._key:
            self._count_bounds(pose)
        if (isinstance(g, NavGoal) and g.goal_id == self._key and pose is not None
                and self._q.pending and self._q.pending[0].mode == MODE_NAV_GOAL
                and math.dist(pose[:2], g.target[:2]) <= self.op.pass_m):
            self._q.done = self._run[-1].seq_id
            return self._advance(now, pose)
        return []

    # ---- 구간 묶음 Finish를 가로채 다음 묶음으로 넘어가기 ----
    def _own(self, cmds: list, now: float, pose) -> list:
        out, finished = [], None
        for c in cmds:
            if isinstance(c, Finish) and isinstance(c.goal_id, _SegKey):
                if self._q is not None and c.goal_id == self._key:
                    finished = c
                continue
            out.append(c)
        if finished is None:
            return out
        if not finished.success:
            lost, self._lost_contact = self._lost_contact, False
            return out + self._end_order(False, HEARTBEAT_LOST if lost else finished.reason)
        self._q.done = self._run[-1].seq_id
        return out + self._own(self._advance(now, pose), now, pose)

    # ---- 다음 구간 묶음을 꺼내 주행 시작 ----
    def _advance(self, now: float, pose) -> list:
        q = self._q
        self._bounds = []
        if not q.pending:
            self._run, self._key = [], None
            cmds = self._end_active(CANCELED, stop=True) if self._active is not None else []
            if not q.horizon:
                return cmds + self._end_order(True, "")
            if self._heartbeat_lost(now):
                return cmds + self._end_order(False, HEARTBEAT_LOST)
            self._waiting = True
            return cmds
        self._waiting = False
        self._runs += 1
        self._run, self._key = q.take_run(self.op.join_m), _SegKey(self._orders, self._runs)
        return self._start(now, pose)

    def _start(self, now: float, pose) -> list:
        alive = self._alive
        cmds = self._start_run(now, pose)
        self._alive = alive
        return cmds

    def _start_run(self, now: float, pose) -> list:
        first, last, key = self._run[0], self._run[-1], self._key
        if first.mode == MODE_NAV_GOAL:
            return super().move(key, now, first.end_node_id, first.target, [], pose,
                                mode=MODE_NAV_GOAL, priority=self._priority)
        points = _joined(self._run)
        if first.mode == MODE_RETREAT:
            return super().retreat(key, now, Polyline(points).length, self.p.reverse_speed_mps,
                                   points, pose)
        self._bounds = _bounds(self._run)
        return super().move(key, now, last.end_node_id, last.path[-1], points, pose,
                            final_goal=last.controller_id != THROUGH, priority=self._priority)

    # ---- 달리는 경로에 이어지는 구간 덧붙이기 ----
    def _extend(self, now: float, pose) -> list:
        g = self._active
        if not (self._run and isinstance(g, _Move) and g.goal_id == self._key):
            return []
        more = self._q.take_merge(self._run[-1], self.op.join_m)
        if not more:
            return []
        end = g.route.length
        self._bounds += [(end, self._run[-1].seq_id)] + [
            (end + s, i) for s, i in _bounds(more, start=g.route.points[-1])]
        g.route = Polyline(g.route.points + _joined(more)[1:])
        g.sent = None
        self._run += more
        g.target, g.target_id = more[-1].path[-1], more[-1].end_node_id
        if g.nav_running and g.handover_since is None:
            return self._send_rest(g, now, pose)
        return []

    # ---- horizon 검사와 주문 종료 ----
    def recheck_horizon(self) -> None:
        if self._q is not None:
            self._check_horizon()

    def _check_horizon(self) -> None:
        self._warning = ""
        if self.horizon_check is None:
            return
        for s in self._q.horizon:
            why = self.horizon_check(s)
            if why:
                self._warning = f"{HORIZON_INVALID}: seq {s.seq_id}: {why}"
                return

    def _end_order(self, success: bool, reason: str) -> list:
        owner, done = self._owner, self._q.done
        self._q = self._owner = self._key = None
        self._run, self._bounds, self._waiting, self._warning = [], [], False, ""
        return [Finish(owner, success, reason, done)]

    def _close(self, reason: str) -> list:
        return self._end_order(False, reason) if self._q is not None else []


# ---- 구간 경로 잇기와 구간 경계 거리 계산 ----
def _joined(run: Sequence[Seg]) -> list[Point]:
    points = [p[:2] for p in run[0].path]
    for s in run[1:]:
        points += [p[:2] for p in s.path[1:]]
    return points


def _bounds(run: Sequence[Seg], start: Point | None = None) -> list[tuple[float, int]]:
    out, s = [], 0.0
    prev = start if start is not None else run[0].path[0][:2]
    for seg in run[:-1]:
        points = [prev] + [p[:2] for p in seg.path[1:]]
        s += Polyline(points).length
        out.append((s, seg.seq_id))
        prev = points[-1]
    return out
