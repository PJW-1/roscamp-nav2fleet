"""주문과 팔 검사: 배정과 출고의 순서, PLACING을 안 보인 팔, 두 팔이 겹친 시각."""
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

    def arms_never_placing(self) -> list[str]:
        """출고된 주문에 필요했는데 PLACING을 한 번도 안 보인 팔. 그 상태가 없으면
        겹침 검사가 데이터 없이 통과한다. 팔이 둘 필요했을 때만 의미가 있다."""
        stock = self.scenario.stock
        items = {o.id: o.items for o in self.scenario.orders}
        needed = {arm for order in self.ship_log for kind in items.get(order, {})
                  for arm in self._placing if stock.get(arm, {}).get(kind, 0) > 0}
        return sorted(needed - self._placing_seen) if len(needed) > 1 else []

    def dual_arm_overlap_list(self) -> list[tuple[float, float]]:
        """작업대 팔 둘이 동시에 PLACING이던 시각. 맞닿기만 한 것은 세지 않는다."""
        latest = max(self._arm_stamp.values(), default=0.0)
        spans = {arm: [*done, *([(self._placing_open[arm], latest)]
                                if self._placing_open[arm] is not None else [])]
                 for arm, done in self._placing.items()}
        arms = sorted(spans)
        out = []
        for i, a in enumerate(arms):
            for b in arms[i + 1:]:
                for start_a, end_a in spans[a]:
                    for start_b, end_b in spans[b]:
                        if start_a < end_b and start_b < end_a:
                            out.append((max(start_a, start_b), min(end_a, end_b)))
        return out
