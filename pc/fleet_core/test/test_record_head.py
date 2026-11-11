"""기록 첫 줄: 로봇, 파라미터, 지도 본문. 재생은 지도를 그 본문으로 다시 만든다.

교통 층은 가짜(conftest.py의 zone_layer)를 쓴다. 지도도 FleetManager가 읽는 칸만 있는
가짜다. 지도 본문이 없으면 기록 파일을 열기 전에 거부한다.
"""
import json
from dataclasses import asdict
from types import SimpleNamespace

import pytest
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from stations import STATIONS

ROBOTS = ["pinky1", "pinky2", "pinky3"]


def stub_site(source_text):
    return SimpleNamespace(stations=STATIONS, bounds=lambda: (0.0, 0.0, 1.98, 0.99),
                           source_text=source_text)


def test_a_map_without_its_text_is_refused_before_anything_is_written(zone_layer):
    core = FleetManager(stub_site(""), load_params(), ROBOTS, lambda: 0.0)
    with pytest.raises(ValueError, match="text of the map"):
        core.check_recordable()
    written = []
    with pytest.raises(ValueError, match="text of the map"):
        core.record_to(written.append)
    assert written == []


def test_the_first_line_holds_the_robots_the_params_and_the_map_text(zone_layer):
    params = load_params()
    core = FleetManager(stub_site("nodes: {}\n"), params, ROBOTS, lambda: 0.0)
    core.check_recordable()
    written = []
    core.record_to(written.append)
    assert json.loads(written[0]) == {"recorded": True, "robots": ROBOTS,
                                      "params": asdict(params), "map": "nodes: {}\n"}
