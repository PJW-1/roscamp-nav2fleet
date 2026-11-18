"""매장 상황이 개발 배치 d2와 실제 경기장 v7에서 서고 끝까지 돈다."""
import pytest
from fake_robots.world.situations import build
from fake_robots.world.store_situations import STORE_NAMES, applies
from fake_robots.world.table import HEADER, run_situation
from fleet_core.fleet_map import load_fleet_map

STORE_LAYOUTS = ("d2", "v7")


def _inside_a_shelf(site, x, y):
    return any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in site.shelves)


@pytest.mark.parametrize("layout", STORE_LAYOUTS)
@pytest.mark.parametrize("name", STORE_NAMES)
def test_every_store_situation_builds_on_the_store_layouts(name, layout):
    site = load_fleet_map(layout)
    if not applies(name, layout, site):
        pytest.skip(f"{name} does not apply to {layout}")
    s = build(name, layout)
    assert s.title and s.timeout_s > 0 and s.injections
    bx0, by0, bx1, by1 = site.bounds()
    for obstacle in s.injections:
        assert bx0 < obstacle.x < bx1 and by0 < obstacle.y < by1
        assert site.zone_at(obstacle.x, obstacle.y) is not None
        assert not _inside_a_shelf(site, obstacle.x, obstacle.y)


def test_only_the_layouts_with_a_corridor_run_the_corridor_box():
    assert not applies("corridor_box", "d2", load_fleet_map("d2"))
    assert applies("corridor_box", "v7", load_fleet_map("v7"))
    assert applies("corridor_box", "d", load_fleet_map("d"))


@pytest.mark.parametrize("layout, node", [("d2", "X_W"), ("d", "X_W"), ("c", "X_W"),
                                          ("v7", "W_X")])
def test_the_person_at_the_dock_stands_on_the_pass_point_by_the_work_dock(layout, node):
    [person] = build("person_at_dock", layout).injections
    assert (person.x, person.y) == load_fleet_map(layout).node_pose(node)


def test_the_corridor_box_on_v7_sits_in_the_south_corridor():
    site = load_fleet_map("v7")
    [box] = build("corridor_box", "v7").injections
    (ax, ay), (bx, by) = site.node_pose("E-3"), site.node_pose("Q2")
    assert (box.x, box.y) == pytest.approx(((ax + bx) / 2, (ay + by) / 2))
    assert site.zone_at(box.x, box.y) == "HALL"


@pytest.mark.parametrize("name", STORE_NAMES)
def test_every_store_situation_runs_to_an_outcome_on_v7(name):
    outcome = run_situation(name, "v7", 1)
    assert len(outcome.row) == len(HEADER)
    assert outcome.row[1:3] == ("v7", 1)
