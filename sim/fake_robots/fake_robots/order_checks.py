"""주문 검사: 배정과 출고의 순서."""
from __future__ import annotations


def _out_of_order(sequence: list[str], rank: dict[str, int]) -> list[str]:
    """순위가 더 뒤인 항목 다음에 온 항목, 또는 순위가 아예 없는 항목."""
    wrong = []
    highest = -1
    for item in sequence:
        position = rank.get(item)
        if position is None or position < highest:
            wrong.append(item)
        else:
            highest = position
    return wrong


class OrderChecks:
    def fifo_problems(self) -> list[str]:
        """등록 순서로 배정하고 배정 순서로 출고한다. 재큐된 주문은 맨 앞으로 돌아가
        다음에 배정돼야 한다. 재큐 없이 두 번 배정되면 안 된다."""
        rank = {o: i for i, o in enumerate(self.registered)}
        problems = []
        open_bound: set[str] = set()  # 배정 후 재큐되지 않은 것(최종 여부 무관)
        requeued: set[str] = set()  # 다시 배정을 기다리는 것
        seen: set[str] = set()
        first_binds = []
        last_bind: dict[str, int] = {}
        for index, (kind, order) in enumerate(self.order_log):
            if kind == "REQUEUED":
                open_bound.discard(order)
                requeued.add(order)
                continue
            if order in open_bound:
                problems.append(f"{order} bound again without a requeue")
            elif requeued and order != min(requeued, key=lambda o: rank.get(o, len(rank))):
                problems.append(f"{order} bound ahead of requeued {sorted(requeued)}")
            open_bound.add(order)
            requeued.discard(order)
            last_bind[order] = index
            if order not in seen:
                seen.add(order)
                first_binds.append(order)
        problems += [f"{o} bound out of registration order"
                     for o in _out_of_order(first_binds, rank)]
        problems += [f"{o} shipped out of bound order"
                     for o in _out_of_order(self.ship_log, last_bind)]
        return problems
