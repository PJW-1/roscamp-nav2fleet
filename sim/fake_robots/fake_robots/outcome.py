"""판정 결과: 주문이 다 끝났는지, 시나리오가 끝났는지, 기대와 비교한 요약."""
from __future__ import annotations


class Outcome:
    @property
    def all_final(self) -> bool:
        return all(order in self.final for order in self.registered)


    def _result_of(self, order: str):
        """끝난 결과, 아직 끝나지 않았으면 보류 중인 것은 HELD."""
        return self.final.get(order) or ("HELD" if order in self.held else None)

    def _complete(self) -> bool:
        return self.all_final and self._all_parked()

    def finished(self, t: float) -> bool:
        return t >= self.scenario.timeout_s or self._complete()

    def summary(self, t: float) -> dict:
        ex = self.scenario.expect
        counts = self.final_counts
        fifo = self.fifo_problems()
        missing = [c for c in ex.events_include if c not in self.events_seen]
        allowed_errors = {*ex.events_include, *ex.event_counts}
        unexpected = [c for c in self.error_codes if c not in allowed_errors]
        timed_out = not self._complete()
        while_error, after_resolved = self.dispatch_checks()

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
            "orders_held": exact(len(self.held), ex.orders_held),
            "orders_canceled": exact(counts["CANCELED"], ex.orders_canceled),
            "order_reported_after_final": exact(len(self.order_report_problems), 0),
            "robots_isolated": exact(self.robots_isolated, ex.robots_isolated),
            "orders_shipped_min": at_least(counts["SHIPPED"], ex.orders_shipped_min),
            "fifo_violations": exact(len(fifo), ex.fifo_violations),
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
            "fill_dispatch_while_arm_error": exact(while_error, ex.fill_dispatch_while_arm_error),
            "fill_dispatch_after_resolved_min": at_least(after_resolved,
                                                         ex.fill_dispatch_after_resolved_min),
            "unexpected_errors": {"value": len(unexpected), "expected": ex.unexpected_errors,
                                  "codes": unexpected,
                                  "ok": len(unexpected) == ex.unexpected_errors},
            "order_results": {
                "value": {o: self._result_of(o) for o in ex.order_results},
                "expected": ex.order_results,
                "ok": all(self._result_of(o) == r for o, r in ex.order_results.items())},
            "hold_reasons": {
                "value": {o: self.hold_reasons.get(o) for o in ex.hold_reasons},
                "expected": ex.hold_reasons,
                "ok": all(self.hold_reasons.get(o) == r for o, r in ex.hold_reasons.items())},
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
            "orders_held": len(self.held),
            "orders_canceled": counts["CANCELED"],
            "duration_s": float(t),
            "timed_out": timed_out,
            "violations": [*self.violations, *fifo,
                           *(f"unexpected ERROR event {c}" for c in unexpected)],
            "min_release_distance_m": min((r[3] for r in self.release_rows), default=None),
            "checks": checks,
        }
