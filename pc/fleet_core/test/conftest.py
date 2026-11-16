"""fleet_core 시험 공통: 가짜 교통 층, 틱 순서 비교용 기록."""
import json
import os
import sys
import types

import pytest
from fleet_core.traffic.modes import ZONE_CONTROLLER


class FakeZoneTraffic:
    """3단계 ZoneTraffic 자리의 가짜. 받은 인자만 남긴다."""

    def __init__(self, site, params, robots, clock, next_goal_id):
        self.args = (site, params, robots, clock, next_goal_id)
        self.robots = list(robots)


@pytest.fixture
def zone_layer(monkeypatch):
    """fleet_core.traffic.zone.controller 자리에 가짜를 둔다."""
    module = types.ModuleType(ZONE_CONTROLLER)
    module.ZoneTraffic = FakeZoneTraffic
    monkeypatch.setitem(sys.modules, ZONE_CONTROLLER, module)
    return module


# ---- 틱 순서 비교용 기록: ZONE_TICK_TRACE=<파일>이면 시험마다 Harness 기록을 JSON으로 남긴다 ----
_TRACE = os.environ.get("ZONE_TICK_TRACE", "")
_traced: dict = {}
_worlds: list = []

if _TRACE:
    import zone_harness

    _harness_init = zone_harness.Harness.__init__

    def _init_and_keep(self, *args, **kwargs):
        _harness_init(self, *args, **kwargs)
        _worlds.append(self)

    zone_harness.Harness.__init__ = _init_and_keep

    @pytest.hookimpl(hookwrapper=True)
    def pytest_runtest_call(item):
        _worlds.clear()
        yield
        _traced[item.nodeid] = [
            {"order": w.tick_order, "log": w.log, "problems": w.problems,
             "results": [[r.goal_id, r.success, r.robot_id, r.reason] for r in w.results],
             "events": [[e.level, e.code, e.robot, e.detail] for e in w.events],
             "commands": [repr(c) for c in w.commands]}
            for w in _worlds]

    def pytest_sessionfinish(session):
        with open(_TRACE, "w") as f:
            json.dump(_traced, f)
