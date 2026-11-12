"""판정 결과: 주문이 다 끝났는지, 시나리오가 끝났는지, 기대와 비교한 요약."""
from __future__ import annotations


class Outcome:
    @property
    def all_final(self) -> bool:
        return all(order in self.final for order in self.registered)


    def _complete(self) -> bool:
        return self.all_final and self._all_parked()

    def finished(self, t: float) -> bool:
        return t >= self.scenario.timeout_s or self._complete()

    def summary(self, t: float) -> dict:
        ex = self.scenario.expect
        counts = self.final_counts
        fifo = self.fifo_problems()
        overlaps = self.dual_arm_overlap_list()
        missing = [c for c in ex.events_include if c not in self.events_seen]
        never_placing = self.arms_never_placing()
        allowed_errors = {*ex.events_include, *ex.event_counts}
        unexpected = [c for c in self.error_codes if c not in allowed_errors]
        timed_out = not self._complete()

        def exact(value, expected):
            return {"value": value, "expected": expected,
                    "ok": expected is None or value == expected}

        def at_least(value, minimum):
            return {"value": value, "minimum": minimum, "ok": value >= minimum}

        checks = {
            "all_orders_finished": {"value": self.all_final, "expected": ex.all_orders_finished,
                                    "ok": self.all_final == ex.all_orders_finished},
            "orders_shipped": exact(counts["SHIPPED"], ex.orders_shipped),
            "orders_partial": exact(counts["PARTIAL"], ex.orders_partial),
            "orders_held": exact(counts["HELD"], ex.orders_held),
            "orders_shipped_min": at_least(counts["SHIPPED"], ex.orders_shipped_min),
            "fifo_violations": exact(len(fifo), ex.fifo_violations),
            "dual_arm_overlaps": {
                "value": len(overlaps), "expected": ex.dual_arm_overlaps,
                "arms_never_placing": never_placing,
                "ok": len(overlaps) == ex.dual_arm_overlaps and not never_placing},
            "zone_holder_max": exact(self.max_occupancy, ex.zone_holder_max),
            "released_without_position_check": exact(self.release_violations,
                                                     ex.released_without_position_check),
            "unheld_zone_pose": exact(self.unheld_violations, ex.unheld_zone_pose),
            "released_while_off_lane": exact(self.off_lane_releases, ex.released_while_off_lane),
            "path_not_connected": exact(self.path_not_connected, ex.path_not_connected),
            "robot_idle": exact(self.robot_idle, ex.robot_idle),
            "direct_returns_min": at_least(self.direct_returns, ex.direct_returns_min),
            "parked_while_backlog": exact(self.parked_while_backlog, ex.parked_while_backlog),
            "shipped_before_operator": at_least(self.shipped_before_operator,
                                                ex.shipped_before_operator_min),
            "detours_around_isolated": at_least(self.detours_around_isolated,
                                                ex.detours_around_isolated_min),
            "timed_out": exact(timed_out, ex.timed_out),
            "bound_after_reinstate_min": at_least(self.bound_after_reinstate,
                                                  ex.bound_after_reinstate_min),
            "shipped_after_reinstate_min": at_least(self.shipped_after_reinstate,
                                                    ex.shipped_after_reinstate_min),
            "unexpected_errors": {"value": len(unexpected), "expected": ex.unexpected_errors,
                                  "codes": unexpected,
                                  "ok": len(unexpected) == ex.unexpected_errors},
            "order_results": {
                "value": {o: self.final.get(o) for o in ex.order_results},
                "expected": ex.order_results,
                "ok": all(self.final.get(o) == r for o, r in ex.order_results.items())},
            "events_include": {"missing": missing, "ok": not missing},
            "event_counts": {
                "value": {c: self.event_counts.get(c, 0) for c in ex.event_counts},
                "expected": ex.event_counts,
                "ok": all(self.event_counts.get(c, 0) == n for c, n in ex.event_counts.items())},
        }
        return {
            "pass": all(c["ok"] for c in checks.values()),
            "orders_total": len(self.registered),
            "orders_shipped": counts["SHIPPED"],
            "orders_partial": counts["PARTIAL"],
            "orders_held": counts["HELD"],
            "duration_s": float(t),
            "timed_out": timed_out,
            "violations": [*self.violations, *fifo,
                           *(f"unexpected ERROR event {c}" for c in unexpected),
                           *(f"{a} never showed PLACING" for a in never_placing),
                           *(f"arm1 and arm2 both PLACING at {start:.2f}-{end:.2f}"
                             for start, end in overlaps)],
            "min_release_distance_m": min((r[3] for r in self.release_rows), default=None),
            "checks": checks,
        }
