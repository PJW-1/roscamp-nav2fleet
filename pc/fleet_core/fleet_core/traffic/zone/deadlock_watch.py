# ---- 대기 순환 감지 ----
from __future__ import annotations

from fleet_core.traffic.deadlock import find_cycle


class DeadlockWatch:
    def __init__(self, params, grants, wants, robots, emit):
        self.params = params
        self.grants = grants
        self._wants = wants
        self._robots = robots
        self.emit = emit
        self.cycle: tuple = ()
        self.since = 0.0
        self.acted = False

    # ---- 대기 그래프의 순환을 찾아 유예가 지나면 돌려주기 ----
    def due(self, now: float) -> tuple:
        graph = {}
        for name, want in self._wants.items():
            if not self._robots[name].available or self._robots[name].watch.redock.key:
                continue
            blockers = self.grants.blockers(name, want.zones, want.nodes, want.lanes)
            robots = {robot for group in blockers.values() for robot in group}
            if robots:
                graph[name] = robots
        cycle = tuple(find_cycle(graph) or ())
        if not cycle:
            self.cycle = ()
        elif set(cycle) != set(self.cycle):
            self.cycle, self.since, self.acted = cycle, now, False
            self.emit("WARN", "DEADLOCK_SEEN", f"wait cycle {' -> '.join(cycle)}")
        elif not self.acted and now - self.since >= self.params.deadlock_grace_s:
            return cycle
        return ()
