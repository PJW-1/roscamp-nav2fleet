"""지도는 2단계의 fleet_core/traffic/fleet_map.py에서 온다. 이 가지에는 아직 없다.

파일이나 기록에서 지도를 읽는 곳은 traffic/site.py를 거친다. 그 모듈이 없는 동안은 무엇이
빠졌는지 말하는 오류를 내고, 있으면 그 모듈에 넘긴다. 지도를 만들어 내지 않는다.
"""
import sys
import types

import pytest
from fleet_core.traffic import site
from fleet_core.traffic.site import FLEET_MAP_MODULE, NoFleetMap, load_site, site_from_text


def test_without_the_fleet_map_reading_a_site_says_what_is_missing():
    assert FLEET_MAP_MODULE == "fleet_core.traffic.fleet_map"
    for read in (lambda: load_site(), lambda: load_site("x.yaml"), lambda: site_from_text("a")):
        with pytest.raises(NoFleetMap, match=r"needs a fleet map \(stage 2\).*fleet_map"):
            read()


def test_with_the_fleet_map_the_reads_go_to_it(monkeypatch):
    module = types.ModuleType(FLEET_MAP_MODULE)
    module.load_fleet_map = lambda path=None: ("file", path)
    module.fleet_map_from_text = lambda text: ("text", text)
    monkeypatch.setitem(sys.modules, FLEET_MAP_MODULE, module)
    assert load_site() == ("file", None)
    assert load_site("d2.yaml") == ("file", "d2.yaml")
    assert site_from_text("nodes: {}") == ("text", "nodes: {}")


def test_an_import_error_inside_the_fleet_map_is_not_hidden(monkeypatch):
    def broken(name):
        raise ModuleNotFoundError("No module named 'shapely'", name="shapely")
    monkeypatch.setattr(site.importlib, "import_module", broken)
    with pytest.raises(ModuleNotFoundError, match="shapely"):
        load_site()
