"""order 모델을 PinkyFsm 위에 얹는다(계약 0.11-draft 제안, order 추가지시서 3절). ROS를 모른다.

order goal 하나(관제의 MoveTo, order_id가 있음)를 조각 줄(order_queue)로 풀어, 조각마다 core의
움직임 하나를 낸다. 조각의 goal id는 _SegKey이고, core가 내는 그 Finish는 여기서 바꾼다: 성공이면
다음 조각을 바로 시작하고(관제를 기다리지 않음), 실패면 order goal을 그 reason으로 끝낸다.

- follow_path: 합친 조각들을 한 경로로 core.move에 준다. controller_id가 FollowPathThrough면
  final_goal 거짓(잠금 경계처럼 설 수 있는 속도로 줄임), 아니면 참이다.
- nav_goal: core.move mode nav_goal. 다음 base 조각도 nav_goal이면 중간 목표 pass_m 안에서 다음
  목표로 바꾼다(bt_navigator가 goal을 바꾸므로 서지 않는다).
- retreat: core.retreat에 경로를 준다. Retreat 액션의 경로 물러나기와 같은 FollowPathReverse다.
- 그 밖의 넘김(방식이 바뀜, controller가 다름, 끝이 안 닿음)은 앞 조각이 끝나 선 뒤 다음을 시작한다.
- 불변식: base가 끝나면 선다. horizon이 있으면 waiting_at_base_end로 update를 기다리고, 없으면
  order가 끝났다(성공).
- 통신 끊김: comm_loss_policy가 base_end이면 base 끝까지 가서 서고(제안), stop(기본)이면
  order가 아닐 때처럼 heartbeat_timeout_s 뒤 바로 선다.
- 감속 늦추기(추가지시서 3절 5번 셋째, 선택)는 만들지 않았다(기본 끔).
"""
from __future__ import annotations

import math
from collections.abc import Callable, Hashable, Sequence
from dataclasses import dataclass

from pinky_fsm.core import FsmParams, PinkyFsm, _Move
from pinky_fsm.geometry import Point, Polyline, Pose
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

THROUGH = "FollowPathThrough"
HorizonCheck = Callable[[Seg], str]  # 조각의 문제(지도 밖, keepout). 없으면 빈 문자열


@dataclass(frozen=True)
class OrderParams:
    join_m: float = 0.02  # follow_path 끝점과 다음 시작점이 이만큼 안이면 닿았다(합침)
    pass_m: float = 0.15  # nav_goal → nav_goal: 중간 목표에서 이만큼 안이면 다음 목표로(R4와 같음)


@dataclass(frozen=True)
class _SegKey:
    """조각 실행 하나의 core goal id. 노드의 액션 goal과 겹치지 않는다."""
    order_no: int
    run_no: int


class OrderFsm(PinkyFsm):
    def __init__(self, params: FsmParams, order_params: OrderParams | None = None,
                 horizon_check: HorizonCheck | None = None):
        super().__init__(params)
        self.op = order_params or OrderParams()
        self.horizon_check = horizon_check
        self._q: OrderQueue | None = None
        self._owner: Hashable | None = None  # 지금 order를 가진 MoveTo goal
        self._run: list[Seg] = []  # 달리는 조각(합친 것 포함)
        self._key: _SegKey | None = None
        self._bounds: list[tuple[float, int]] = []  # 합친 경로 위 중간 조각의 끝(s, seq_id)
        self._waiting = False
        self._warning = ""
        self._orders = 0
        self._runs = 0

    # -- 보고 -----------------------------------------------------------------------------

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
        """(last_done_seq_id, waiting_at_base_end, horizon_warning)."""
        if self._q is None:
            return 0, False, ""
        return self._q.done, self._waiting, self._warning

    def order_view(self) -> tuple[list[Seg], list[Seg]]:
        """표시용: 아직 안 끝난 base 조각(달리는 것 포함)과 horizon."""
        if self._q is None:
            return [], []
        done = self._q.done
        base = [s for s in self._run if s.seq_id > done] + list(self._q.pending)
        return base, list(self._q.horizon)

    # -- 입력 -----------------------------------------------------------------------------

    def move(self, goal_id: Hashable, now: float, target_id: str, target: Pose,
             path: Sequence[Point], pose: Sequence[float] | None,
             final_goal: bool = True, mode: str = "", order: OrderGoal | None = None) -> list:
        if order is None or not order.order_id:
            return self._close(CANCELED) + self._own(
                super().move(goal_id, now, target_id, target, path, pose, final_goal, mode),
                now, pose)
        self._alive = max(self._alive, now)  # 목표 수락도 관제가 살아 있다는 증거
        if order.update_id > 0:
            return self._update(goal_id, now, order, pose)
        cmds = self._close(CANCELED)
        # seq_id는 1부터다(0은 "끝낸 조각 없음"이다. Codex 리뷰)
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

    def tick(self, now, pose, speed, obstacle) -> list:
        self._count_bounds(pose)  # heartbeat로 끝나기 전에 방금 끝낸 조각을 센다
        cmds = self._own(super().tick(now, pose, speed, obstacle), now, pose)
        return cmds + self._own(self._order_tick(now, pose), now, pose)

    # -- 안쪽 -----------------------------------------------------------------------------

    def _stops_on_heartbeat_loss(self) -> bool:
        """comm_loss_policy가 base_end이면 order의 조각을 달리는 중 끊겨도 base 끝까지 간다
        (추가지시서 3절 6번 제안). 단 막혀 나아가지 못하면(BLOCKED, Nav2가 물체 앞에서 섬)
        지금처럼 선다(Codex 리뷰: 끝없이 돌지 않게). stop이면(기본) order가 아닌 목표처럼 바로
        선다."""
        if self.p.comm_loss_policy == "stop":
            return True
        g = self._active
        if self._key is None or g is None or g.goal_id != self._key:
            return True
        return (self._blocked or getattr(g, "blocked", False)
                or getattr(g, "waiting_since", None) is not None)

    def _count_bounds(self, pose) -> None:
        """합친 경로에서 지난 중간 조각을 끝낸 것으로 센다."""
        g = self._active
        if (self._q is None or pose is None or not isinstance(g, _Move)
                or g.goal_id != self._key):
            return
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
            return self._advance(now, pose)  # 서지 않고 다음 NavigateToPose로
        return []

    def _own(self, cmds: list, now: float, pose) -> list:
        """조각의 Finish를 order의 것으로 바꾼다. 지난 조각의 Finish는 버린다."""
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

    def _advance(self, now: float, pose) -> list:
        """다음 조각을 시작한다. base가 끝났으면 서서 기다리거나 order를 끝낸다."""
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
        """조각 시작은 관제가 보낸 goal이 아니므로 heartbeat 시계를 건드리지 않는다(Codex 리뷰)."""
        alive = self._alive
        cmds = self._start_run(now, pose)
        self._alive = alive
        return cmds

    def _start_run(self, now: float, pose) -> list:
        first, last, key = self._run[0], self._run[-1], self._key
        if first.mode == MODE_NAV_GOAL:
            return super().move(key, now, first.end_node_id, first.target, [], pose,
                                mode=MODE_NAV_GOAL)
        points = _joined(self._run)
        if first.mode == MODE_RETREAT:
            return super().retreat(key, now, Polyline(points).length, self.p.reverse_speed_mps,
                                   points, pose)
        self._bounds = _bounds(self._run)
        return super().move(key, now, last.end_node_id, last.path[-1], points, pose,
                            final_goal=last.controller_id != THROUGH)

    def _extend(self, now: float, pose) -> list:
        """update의 첫 조각이 달리는 FollowPath에 이어지면 경로를 늘려 선점한다(서지 않음)."""
        g = self._active
        if not (self._run and isinstance(g, _Move) and g.goal_id == self._key):
            return []
        more = self._q.take_merge(self._run[-1], self.op.join_m)
        if not more:
            return []
        # 달리던 마지막 조각의 끝(지금 경로 끝)이 이제 중간 조각의 끝이다
        end = g.route.length
        self._bounds += [(end, self._run[-1].seq_id)] + [
            (end + s, i) for s, i in _bounds(more, start=g.route.points[-1])]
        g.route = Polyline(g.route.points + _joined(more)[1:])
        self._run += more
        g.target, g.target_id = more[-1].path[-1], more[-1].end_node_id
        if g.nav_running and g.handover_since is None:
            return self._send_rest(g, now, pose)
        return []  # 다시 보낼 때(복구, 물체가 비킴, 넘김) 늘린 경로를 쓴다

    def recheck_horizon(self) -> None:
        """지도나 keepout 마스크가 order 뒤에 왔을 수 있다. 노드가 주기적으로 부른다."""
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
        """새 goal이 order를 밀어낸다. 달리던 조각은 core가 새 goal의 규칙대로 멈춘다."""
        return self._end_order(False, reason) if self._q is not None else []


def _joined(run: Sequence[Seg]) -> list[Point]:
    """합친 조각의 경로 점. 뒤 조각의 첫 점은 앞 조각의 끝과 닿으므로 뺀다."""
    points = [p[:2] for p in run[0].path]
    for s in run[1:]:
        points += [p[:2] for p in s.path[1:]]
    return points


def _bounds(run: Sequence[Seg], start: Point | None = None) -> list[tuple[float, int]]:
    """보낸(합친) 경로 위 각 조각 끝의 거리(s, seq_id). 뒤 조각의 첫 점은 _joined처럼 빼고 앞 끝에서
    잇는다. 마지막 조각은 FollowPath 결과로 끝나므로 뺀다. start는 앞 경로의 끝(이어 붙일 때)."""
    out, s = [], 0.0
    prev = start if start is not None else run[0].path[0][:2]
    for seg in run[:-1]:
        points = [prev] + [p[:2] for p in seg.path[1:]]
        s += Polyline(points).length
        out.append((s, seg.seq_id))
        prev = points[-1]
    return out
