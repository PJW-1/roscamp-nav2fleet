"""order 모델의 goal 하나(계약 0.11-draft 제안의 로봇 쪽 규칙): base 조각을 쉬지 않고 잇달아 달린다.

- 조각 사이에서 관제를 기다리지 않는다. 조각 하나는 FollowRun, NavGoalRun, RetreatRun 가운데
  하나로 달린다(make가 만든다).
- base가 끝나고 horizon이 남으면 base 끝에 서서(waiting_at_base_end) update를 기다린다.
- update는 이음 조건 넷(order_id, 다음 update id, 이음 지점 = base 끝, seq가 이어짐)이 모두 맞을
  때만 줄 뒤에 붙인다. 붙이면 goal id가 새 goal의 것으로 바뀐다.
- 통신이 끊겨도 이미 받은 base 끝까지 간다(세계가 run을 계속 돌린다). 장애물 앞에서는 선다.
"""
from __future__ import annotations

from .runs import MOVE, Result, Run


def refusal(run, cmd) -> str:
    """update를 붙이면 안 되는 이유. 붙여도 되면 빈 문자열. run은 OrderRun이거나 None."""
    if run is None:
        return "no running order"
    if cmd.order_id != run.order_id:
        return "order_id differs"
    if cmd.update_id != run.update + 1:
        return "order_update_id is not the next one"
    if cmd.stitch_node != (run.last.end_node if run.last else ""):
        return "stitch_node_id is not the end of the base"
    if cmd.base and run.last and cmd.base[0].seq_id != run.last.seq_id + 1:
        return "seq_id does not continue"
    return ""


class OrderRun(Run):
    kind = MOVE

    def __init__(self, cmd, make):
        """make(segment, final, goal_id) -> Run: 조각 하나를 달리는 run."""
        super().__init__(cmd.goal_id)
        self.order_id, self.update = cmd.order_id, 0
        self.queue = list(cmd.base)
        self.horizon = tuple(cmd.horizon)
        self.last = cmd.base[-1] if cmd.base else None  # base의 마지막 조각
        self.make = make
        self.sub: Run | None = None  # 달리는 조각의 run
        self.current = None  # 달리는 조각
        self.done_seq = 0
        self.waiting = False  # base 끝에 서서 update를 기다린다
        self.told = None  # 마지막으로 관제에 알린 (done_seq, waiting, goal_id)

    @property
    def blocked(self) -> bool:
        return self.sub is not None and self.sub.blocked

    def stitch(self, cmd) -> int:
        """update를 붙이고, 넘겨준 이전 goal id를 돌려준다."""
        old, self.goal_id, self.update = self.goal_id, cmd.goal_id, cmd.update_id
        self.queue += list(cmd.base)
        self.horizon = tuple(cmd.horizon)
        if cmd.base:
            self.last = cmd.base[-1]
            self.waiting = False
        return old

    def done(self, success: bool, reason: str = "") -> Result:
        return Result(self.kind, self.goal_id, success, reason, self.done_seq)

    def step(self, ctx, robot) -> Result | None:
        self.started = True
        if self.sub is None:
            if not self.queue:
                if self.horizon:
                    self.waiting = True
                    return None
                return self.done(True)
            self.current = self.queue.pop(0)
            final = not self.queue and not self.horizon
            self.sub = self.make(self.current, final, self.goal_id)
        result = self.sub.step(ctx, robot)
        if result is None:
            return None
        self.sub = None
        if not result.success:
            return self.done(False, result.reason)
        self.done_seq = self.current.seq_id
        self.waiting = not self.queue and bool(self.horizon)
        if not self.queue and not self.horizon:
            return self.done(True)
        return None
