"""파일이나 기록에서 지도를 읽는 곳은 traffic/site.py를 거치고, 그 지도는 구역 지도다."""
from fleet_core.fleet_map import FleetMap
from fleet_core.traffic.site import load_site, site_from_text


def test_load_site_reads_the_default_layout_or_a_named_one():
    assert isinstance(load_site(), FleetMap)
    assert load_site().name == "arena_d2"
    assert load_site("c").name == "arena_c"


def test_a_site_is_rebuilt_from_the_text_kept_in_a_record():
    site = load_site("d")
    again = site_from_text(site.source_text)
    assert again.name == "arena_d"
    assert again.links() == site.links()
    assert again.source_text == site.source_text
