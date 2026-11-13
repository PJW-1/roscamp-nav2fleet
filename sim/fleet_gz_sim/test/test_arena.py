"""경기장 생성(arena, arena_maps, arena_world): 구역 지도에서 월드, 지도, keepout 마스크."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

pytest.importorskip("fleet_core")
from fleet_core.fleet_map import NAV_GOAL, SHARED, load_fleet_map  # noqa: E402
from fleet_gz_sim import arena, arena_maps, arena_world  # noqa: E402

PKG = Path(__file__).resolve().parents[1]


@pytest.fixture(params=arena.KNOWN_LAYOUTS)
def site(request):
    return load_fleet_map(request.param)


def test_committed_worlds_and_maps_match_the_generator(tmp_path, site):
    written = arena.generate(tmp_path, site)
    assert len(written) == 7
    for name in written:
        assert (tmp_path / name).read_bytes() == (PKG / name).read_bytes(), name


def test_every_layout_has_its_files_named_after_the_fleet_map(site):
    for name in arena.outputs(site.name):
        assert (PKG / name).is_file(), name


def test_setup_installs_worlds_maps_scenarios_and_no_sites_folder():
    text = (PKG / "setup.py").read_text(encoding="utf-8")
    for folder in ("worlds", "maps", "scenarios", "obstacles"):
        assert f"share/fleet_gz_sim/{folder}" in text
        assert any((PKG / folder).iterdir()), folder
    assert "sites" not in text


def _value(grid, x, y, resolution, site):
    i, j = arena_maps.cell_of(x, y, resolution, site)
    return grid[j][i]


@pytest.mark.parametrize("resolution", [arena_maps.COARSE_M, arena_maps.FINE_M])
def test_nodes_are_free_and_walls_and_shelves_are_occupied(site, resolution):
    grid = arena_maps.occupancy(resolution, site)
    for node in site.nodes.values():
        assert not _value(grid, node.x, node.y, resolution, site), node.id
    x0, y0, x1, y1 = site.bounds()
    for x, y in ((x0 - 0.02, (y0 + y1) / 2), (x1 + 0.02, (y0 + y1) / 2),
                 ((x0 + x1) / 2, y0 - 0.02), ((x0 + x1) / 2, y1 + 0.02)):
        assert _value(grid, x, y, resolution, site)
    for sx0, sy0, sx1, sy1 in site.shelves:
        assert _value(grid, (sx0 + sx1) / 2, (sy0 + sy1) / 2, resolution, site)


def test_shelves_are_thin_and_tall_enough_for_the_lidar(site):
    root = ET.fromstring((PKG / "worlds" / f"{site.name}.sdf").read_text(encoding="utf-8"))
    shelves = [c for c in root.iter("collision") if c.get("name").startswith("shelf_")]
    assert len(shelves) == len(site.shelves) > 0
    for shelf in shelves:
        sx, sy, sz = (float(v) for v in shelf.find("geometry/box/size").text.split())
        # D 계열은 지시서의 1.5 cm, v7은 사용자 치수(진열대 0.04, 칸막이 0.02)
        thin = 0.04 if site.name == "arena_v7" else 0.015
        assert min(sx, sy) <= thin + 1e-9
        if site.name != "arena_v7":
            assert min(sx, sy) == pytest.approx(0.015)
        assert sz >= arena.MIN_SHELF_HEIGHT_M > arena.LIDAR_Z_M


def test_a_shelf_lower_than_the_lidar_needs_is_refused():
    low = load_fleet_map("d2")
    low.shelf_height_m = 0.10
    with pytest.raises(arena.ArenaError, match="shelf_height_m"):
        arena.shelves(low)


def test_world_walls_enclose_the_inner_floor(site):
    root = ET.fromstring(arena_world.world_sdf(site))
    assert root.find("world").get("name") == arena.WORLD
    boxes = {c.get("name"): c for c in root.iter("collision")}
    x0, _, x1, _ = site.bounds()
    west = float(boxes["wall_west"].find("pose").text.split()[0])
    east = float(boxes["wall_east"].find("pose").text.split()[0])
    assert west == pytest.approx(x0 - arena.WALL_M / 2)
    assert east == pytest.approx(x1 + arena.WALL_M / 2)


def test_every_world_steps_physics_at_4_ms(site):
    root = ET.fromstring((PKG / "worlds" / f"{site.name}.sdf").read_text(encoding="utf-8"))
    assert float(root.find("world/physics/max_step_size").text) == 0.004


def test_fixed_paths_are_painted_without_collision(site):
    root = ET.fromstring(arena_world.world_sdf(site))
    assert [v for v in root.iter("visual") if v.get("name").startswith("path_")]
    assert not [c for c in root.iter("collision") if c.get("name").startswith("path_")]
    names = [v.get("name") for v in root.iter("visual") if v.get("name").startswith("path_")]
    assert len(names) == len(set(names)), "a bent path paints each piece under its own name"


def test_map_yaml_names_its_image_and_the_world_frame(site):
    for suffix, resolution in (("", 0.05), ("_fine", 0.01), ("_keepout", 0.01)):
        text = (PKG / "maps" / f"{site.name}{suffix}.yaml").read_text(encoding="utf-8")
        assert f"image: {site.name}{suffix}.pgm" in text
        assert f"resolution: {resolution}" in text
        assert "origin: [-0.15, -0.15, 0.0]" in text


# ---- keepout ----

def test_every_zone_that_is_not_shared_is_keepout(site):
    grid = arena_maps.keepout(arena_maps.FINE_M, site)
    pad = arena_maps.KEEPOUT_PAD_M
    for zone in site.zones.values():
        for x0, y0, x1, y1 in zone.rects:
            if zone.kind == SHARED and min(x1 - x0, y1 - y0) < 2 * pad:
                continue  # a shared sliver (beside the shelf) lies inside the pad by design
            inside = _value(grid, (x0 + x1) / 2, (y0 + y1) / 2, arena_maps.FINE_M, site)
            assert inside == (zone.kind != SHARED), zone.id


def test_keepout_is_widened_by_half_the_footprint(site):
    pad = arena_maps.KEEPOUT_PAD_M
    grid = arena_maps.keepout(arena_maps.FINE_M, site)
    x0, y0, x1, _ = site.zones["J_W" if "J_W" in site.zones else "J_E"].rects[0]
    cx = (x0 + x1) / 2
    assert _value(grid, cx, y0 - pad + 0.005, arena_maps.FINE_M, site)
    assert not _value(grid, cx, y0 - pad - 0.015, arena_maps.FINE_M, site)


def test_shared_nodes_and_their_footprint_stay_out_of_the_keepout(site):
    """A robot waiting on a shared node, or a Nav2 goal ending there, sits outside the keepout."""
    half = 0.06
    nav_ends = {end for link in site.links() if link.via == NAV_GOAL for end in (link.a, link.b)}
    for node in site.nodes.values():
        # 고정 경로로만 드나드는 지점(v7 W_X: WORK_DOCK과 J_E 사이 띠)은 Nav2가 계획하지 않는다
        if site.zones[node.zone].kind != SHARED or node.id not in nav_ends:
            continue
        for x0, y0, x1, y1 in arena_maps.keepout_rects(site):
            assert not (x0 - half < node.x < x1 + half and y0 - half < node.y < y1 + half), \
                (node.id, (x0, y0, x1, y1))


def test_straight_nav_goal_links_run_clear_of_the_keepout(site):
    """The straight line of every nav_goal link stays out of the keepout, so the planner has a
    way inside the shared zones."""
    grid = arena_maps.keepout(arena_maps.FINE_M, site)
    for link in site.links():
        if link.via != NAV_GOAL:
            continue
        (ax, ay), (bx, by) = link.points[0], link.points[-1]
        steps = max(2, int(math.dist((ax, ay), (bx, by)) / 0.01))
        hits = [i for i in range(steps + 1)
                if _value(grid, ax + (bx - ax) * i / steps, ay + (by - ay) * i / steps,
                          arena_maps.FINE_M, site)]
        assert not hits, f"{link.a}->{link.b}"


def test_a_thin_shelf_off_the_grid_still_fills_the_cells_it_touches():
    site = load_fleet_map("d2")
    grid = arena_maps.occupancy(arena_maps.COARSE_M, site)
    x0, y0, x1, y1 = site.shelves[0]
    assert _value(grid, (x0 + x1) / 2, y0 + 0.001, arena_maps.COARSE_M, site)
    assert _value(grid, (x0 + x1) / 2, y1 - 0.001, arena_maps.COARSE_M, site)


def test_boxes_are_walls_and_shelves_above_the_lidar(site):
    boxes = arena.obstacles(site)
    assert {box.kind for box in boxes} == {"wall", "shelf"}
    assert all(box.height_m > arena.LIDAR_Z_M for box in boxes)
