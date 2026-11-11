"""시험용 로봇이 order 하나를 다루는 법: 계약 0.11-draft(제안)의 로봇 쪽 규칙을 작게 옮겼다.

base 조각을 차례로 달리고, base가 끝나면 선다. horizon이 남아 있으면 update를 기다리고, 없으면
order가 끝난다. update는 이음 조건 넷이 모두 맞을 때만 줄 뒤에 붙인다.
"""
from __future__ import annotations


class OrderRun:
    def __init__(self, cmd):
        self.id, self.update, self.goal_id = cmd.order_id, 0, cmd.goal_id
        self.queue = list(cmd.base)
        self.horizon = cmd.horizon
        self.last = cmd.base[-1] if cmd.base else None  # base의 마지막 조각
        self.current = None  # 달리는 조각
        self.done_seq = 0
        self.waiting = False  # base 끝에 서서 update를 기다린다
        self.told = None  # 마지막으로 보낸 피드백

    def refusal(self, cmd) -> str:
        """update를 붙이면 안 되는 이유. 붙여도 되면 빈 문자열."""
        if cmd.order_id != self.id:
            return "order_id differs"
        if cmd.update_id != self.update + 1:
            return "order_update_id is not the next one"
        if cmd.stitch_node != (self.last.end_node if self.last else ""):
            return "stitch_node_id is not the end of the base"
        if cmd.base and self.last and cmd.base[0].seq_id != self.last.seq_id + 1:
            return "seq_id does not continue"
        return ""

    def stitch(self, cmd) -> int:
        """update를 붙이고, 넘겨준 이전 goal id를 돌려준다."""
        old, self.goal_id, self.update = self.goal_id, cmd.goal_id, cmd.update_id
        self.queue += list(cmd.base)
        self.horizon = cmd.horizon
        if cmd.base:
            self.last = cmd.base[-1]
            self.waiting = False
        return old

    def next_segment(self):
        """다음에 달릴 조각. 줄이 비었으면 None."""
        if self.current is None and self.queue:
            self.current = self.queue.pop(0)
        return self.current

    def finish_segment(self) -> None:
        self.done_seq, self.current = self.current.seq_id, None
        self.waiting = not self.queue and bool(self.horizon)

    @property
    def finished(self) -> bool:
        return self.current is None and not self.queue and not self.horizon
