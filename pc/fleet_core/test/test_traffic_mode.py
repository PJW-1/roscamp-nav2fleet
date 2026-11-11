"""traffic_mode는 이제 zones 하나다. 교통 층은 3단계에서 traffic/zone/controller.py로 온다.

그 모듈이 없는 동안 build_traffic은 무엇이 빠졌는지 말하는 오류를 낸다. 있으면 지금과 같은
인자(site, params, robots, clock, next_goal_id)로 ZoneTraffic을 만든다.
"""
from dataclasses import asdict, replace

import pytest
import yaml
from fleet_core.common.params import DEFAULT_PARAMS, FleetParams, load_params
from fleet_core.traffic import modes
from fleet_core.traffic.modes import MODES, ZONE_CONTROLLER, ZONES, NoZoneTraffic, build_traffic

ROBOTS = ["pinky1", "pinky2", "pinky3"]


def test_zones_is_the_only_mode():
    assert MODES == (ZONES,) == ("zones",)


def test_the_shipped_params_pick_zones():
    assert load_params().traffic_mode == "zones"
    assert yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))["traffic_mode"] == "zones"


def test_params_without_the_value_pick_zones():
    fields = asdict(load_params())
    del fields["traffic_mode"]
    assert FleetParams(**fields).traffic_mode == "zones"


def test_without_the_zone_layer_the_error_says_what_is_missing():
    assert ZONE_CONTROLLER == "fleet_core.traffic.zone.controller"
    with pytest.raises(NoZoneTraffic, match=r"needs the zone traffic layer \(stage 3\).*"
                                            r"fleet_core\.traffic\.zone\.controller"):
        build_traffic(object(), load_params(), ROBOTS, lambda: 0.0, None)


def test_the_zone_layer_gets_the_same_arguments_as_before(zone_layer):
    site, params, clock, next_id = object(), load_params(), (lambda: 0.0), (lambda: 1)
    traffic = build_traffic(site, params, ROBOTS, clock, next_id)
    assert type(traffic).__name__ == "FakeZoneTraffic"
    assert traffic.args == (site, params, ROBOTS, clock, next_id)


def test_an_unknown_mode_is_refused_with_the_known_ones(zone_layer):
    for old in ("lanes", "freefloor", "roads"):
        with pytest.raises(ValueError, match="zones"):
            build_traffic(object(), replace(load_params(), traffic_mode=old), ROBOTS,
                          lambda: 0.0, None)


def test_an_import_error_inside_the_zone_layer_is_not_hidden(monkeypatch):
    """모듈은 있는데 그 안의 임포트가 틀렸으면 그 오류가 그대로 보여야 한다."""
    def broken(name):
        raise ModuleNotFoundError("No module named 'numpy'", name="numpy")
    monkeypatch.setattr(modes.importlib, "import_module", broken)
    with pytest.raises(ModuleNotFoundError, match="numpy"):
        build_traffic(object(), load_params(), ROBOTS, lambda: 0.0, None)
