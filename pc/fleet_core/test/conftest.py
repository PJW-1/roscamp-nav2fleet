"""fleet_core 시험 공통: 기다리는 시험 건너뛰기(fleet_core/pending.py)와 가짜 교통 층."""
import sys
import types

import pytest
from fleet_core.pending import skip_waiting_module
from fleet_core.traffic.modes import ZONE_CONTROLLER


def pytest_pycollect_makemodule(module_path, parent):
    return skip_waiting_module(module_path, parent)


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
