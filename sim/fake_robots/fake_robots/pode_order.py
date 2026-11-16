"""가짜 Pode의 order goal: base 조각을 차례로 달리고, update를 이어 붙인다(계약 0.11-draft).

- 조각 줄, 진행(done_seq), base 끝 대기, 이음 검사는 세계 모의의 OrderRun과 refusal을 쓴다
  (world/order_run.py). 같은 규칙을 세 번째로 쓰지 않는다.
- 조각 하나는 이 노드의 이동(path, nav_goal)이나 후진(retreat)으로 달린다.
- update가 맞으면 이전 goal을 SUPERSEDED로 끝내고 새 goal이 달리던 것을 그대로 이어받는다.
  안 맞으면 새 goal만 ORDER_UPDATE_REJECTED로 끝낸다(실물 pode_fsm order_fsm.py와 같다).
- 통신이 끊기면 comm_loss_policy stop처럼 바로 선다(노드의 heartbeat 처리 그대로).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

from fleet_interfaces.action import MoveTo

from fake_robots.ahead import PathWatch
from fake_robots.motion import noisy_pose
from fake_robots.nav_goal_drive import NavGoalDrive
from fake_robots.pode_goal import _Goal
from fake_robots.pode_retreat import REVERSE_SPEED_MPS
from fake_robots.world.order_run import OrderRun, refusal
from fake_robots.world.runs import MOVE, Result

FOLLOW_PATH, NAV_GOAL, RETREAT = "follow_path", "nav_goal", "retreat"
SUPERSEDED = MoveTo.Result.REASON_SUPERSEDED
REJECTED = MoveTo.Result.REASON_ORDER_UPDATE_REJECTED
SEG_MODES = (FOLLOW_PATH, NAV_GOAL, RETREAT)
JOIN_M = 0.02  # 실물 OrderParams.join_m: 이보다 가까이 이어진 path 조각은 한 줄로 달린다
_OWN = ("handle", "done", "success", "reason", "last_done")  # 넘겨받지 않는 goal 칸


# ---- ROS 메시지를 OrderRun이 읽는 모양으로 ----

@dataclass(frozen=True)
class Seg:
    seq_id: int
    mode: str
    end_node: str
    path: tuple[tuple[float, float], ...]
    x: float
    y: float
    controller_id: str = ""


@dataclass(frozen=True)
class OrderCmd:
    goal_id: bytes
    order_id: str
    update_id: int
    stitch_node: str
    base: tuple[Seg, ...]
    horizon: tuple[Seg, ...]


def segment(msg) -> Seg:
    path = tuple((p.pose.position.x, p.pose.position.y) for p in msg.path.poses)
    if msg.mode == NAV_GOAL:
        x, y = msg.target_pose.pose.position.x, msg.target_pose.pose.position.y
    else:
        x, y = path[-1] if path else (math.nan, math.nan)
    return Seg(int(msg.seq_id), msg.mode, msg.end_node_id, path, x, y, msg.controller_id)


def segments_problem(segs) -> str:
    """실물 order_queue.segments_problem과 같은 검사: seq가 이어지고, mode를 알고, path가 있다."""
    for a, b in zip(segs, segs[1:], strict=False):
        if b.seq_id != a.seq_id + 1:
            return f"seq_id {b.seq_id} does not follow {a.seq_id}"
    for s in segs:
        if s.mode not in SEG_MODES:
            return f"seq {s.seq_id}: unknown mode '{s.mode}'"
        if s.mode != NAV_GOAL and len(s.path) < 2:
            return f"seq {s.seq_id}: {s.mode} needs a path of two poses or more"
    return ""


def joins(a: Seg | None, b: Seg) -> bool:
    """실물 order_queue.mergeable: 앞 조각에 바로 이어 달리는 path 조각인가."""
    return (a is not None and a.mode == b.mode == FOLLOW_PATH
            and a.controller_id == b.controller_id
            and math.dist(a.path[-1], b.path[0]) <= JOIN_M)


def order_cmd(goal_id: bytes, req) -> OrderCmd | None:
    """order_id가 비면 order goal이 아니다(옛 MoveTo)."""
    if not req.order_id:
        return None
    return OrderCmd(goal_id, req.order_id, int(req.order_update_id), req.stitch_node_id,
                    tuple(segment(s) for s in req.base), tuple(segment(s) for s in req.horizon))


class _SegmentDrive:
    """OrderRun이 조각마다 만드는 run. 움직임은 노드가 한다. ctx는 (t, dt), robot은 _Goal."""

    blocked = False

    def __init__(self, node, seg: Seg, final: bool):
        self.node, self.seg, self.final = node, seg, final
        self.started = False

    def step(self, ctx, goal) -> Result | None:
        t, dt = ctx
        if not self.started:
            self.started = True
            why = self.node._start_segment(t, goal, self.seg, self.final)
            if why:
                return Result(MOVE, 0, False, why)
        out = self.node._segment_step(t, dt, goal)
        return None if out is None else Result(MOVE, 0, *out)


# ---- 노드 믹스인 ----

class PodeOrder:
    def _start_order(self, goal: _Goal, cmd: OrderCmd) -> None:
        goal.order = OrderRun(cmd, lambda seg, final, _id: _SegmentDrive(self, seg, final))
        if segments_problem(cmd.base) or (cmd.base and cmd.base[0].seq_id != 1):
            self._end(goal, False, MoveTo.Result.REASON_NAV_ABORTED)  # 실물처럼 서서 끝낸다
            return
        goal.target_id = cmd.base[-1].end_node if cmd.base else ""
        self._motion.clear_target()

    def _update_order(self, goal: _Goal, cmd: OrderCmd) -> None:
        """update goal 하나. 붙이면 이전 goal을 SUPERSEDED로, 아니면 이 goal만 끝낸다."""
        old = self._active
        run = old.order if old is not None and not old.done.is_set() else None
        why = refusal(run, cmd) or segments_problem(cmd.base)
        if why:
            goal.last_done = run.done_seq if run is not None else 0
            goal.finish(False, f"{REJECTED}: {why}")
            return
        for key, value in vars(old).items():
            if key not in _OWN:
                setattr(goal, key, value)
        run.stitch(cmd)
        if cmd.base:
            goal.target_id = cmd.base[-1].end_node
        old.abort_at = None  # 고장 일정은 새 goal이 가져갔다
        self._active = goal
        self._last_beat = time.monotonic()
        old.last_done = run.done_seq
        old.finish(False, SUPERSEDED)

    def _order_step(self, t: float, dt: float, goal: _Goal) -> None:
        result = goal.order.step((t, dt), goal)
        if goal.order.waiting:
            # base 끝에 서 있다. 지난 조각의 막힘과 이음은 잊는다(다음 조각은 새 줄로 시작한다)
            goal.nav = goal.line = goal.seg = None
        if result is not None:
            self._end(goal, result.success, result.reason)

    def _start_segment(self, t: float, goal: _Goal, seg: Seg, final: bool) -> str:
        """조각 하나를 시작한다. 못 하면 이유. 이음 점의 정지 오차는 마지막 조각에만 준다."""
        m = self._motion
        joined = goal.seg_mode == FOLLOW_PATH and joins(goal.seg, seg)  # 앞 조각에 잇는다
        goal.nav = goal.line = None
        goal.aborting = False  # 앞 조각의 Nav2 중단 복구는 조각과 함께 끝난다
        goal.seg_mode, goal.seg = seg.mode, seg
        target = (seg.x, seg.y)
        if final:
            target = noisy_pose(seg.x, seg.y, self._cfg.stop_error_m, self._rng)
        if seg.mode == RETREAT:
            path = list(seg.path)
            length = sum(math.dist(a, b) for a, b in zip(path, path[1:], strict=False))
            ok = self._begin_retreat(t, goal, length, REVERSE_SPEED_MPS, path)
            return "" if ok else MoveTo.Result.REASON_NAV_ABORTED
        if seg.mode == NAV_GOAL:
            m.clear_target()
            goal.nav = NavGoalDrive(self._space, self._recovery.p.robot_radius_m, target, t,
                                    (m.x, m.y))
            self._take_segment_abort(t, goal, seg, math.dist((m.x, m.y), target))
            self._recovery.restart()  # 예전 goal 끝처럼: 후진을 멈추고 물체 자리는 이 조각의 시작
            self._recovery.watch(t, self._faults.obstacle(t), True, m)  # 앞 물체면 여기서 선다
            return ""
        path = list(seg.path)
        # 연결 검사는 실물처럼 새로 시작하는 줄만 한다. 앞 조각에 이은 조각은 하지 않는다
        if not joined and self._path_misses_me(t, path):
            return MoveTo.Result.REASON_PATH_NOT_CONNECTED
        goal.line = PathWatch(self._recovery.p.robot_radius_m)
        m.set_target(*target, via=path[1:-1], start=path[0] if path else None)
        self._take_segment_abort(t, goal, seg, m.remaining())
        self._recovery.restart()  # 예전 goal 끝처럼: 후진을 멈추고 물체 자리는 이 조각의 시작
        self._recovery.watch(t, self._faults.obstacle(t), True, m)  # 앞 물체면 여기서 선다
        return ""

    def _take_segment_abort(self, t: float, goal: _Goal, seg: Seg, remaining: float) -> None:
        """Nav2 중단 고장은 조각 끝 노드로 잡는다. 이미 잡은 것이 있으면 그대로 둔다."""
        if goal.abort_at is None and not goal.aborting:
            target_id, goal.target_id = goal.target_id, seg.end_node
            self._take_nav_abort(t, goal, remaining)
            goal.target_id = target_id

    def _segment_step(self, t: float, dt: float, goal: _Goal):
        if goal.seg_mode == RETREAT:
            out = self._retreat_move(t, dt, goal)
        else:
            out = self._move_step(t, dt, goal)
        if out is not None and out[0] and goal.abort_at is not None:
            # 아직 안 난 Nav2 중단은 그 조각의 것이다. 돌려주고 다음 조각에서 다시 잡는다
            self._faults.give_back_nav_abort(goal.abort_from)
            goal.abort_at, goal.abort_from = None, None
        return out
