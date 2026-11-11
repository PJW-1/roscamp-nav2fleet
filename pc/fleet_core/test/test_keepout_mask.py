"""keepout 마스크: 구역 지도에서, 주어진 지도의 크기와 원점에 맞춰 만든다."""
import subprocess
import sys
from pathlib import Path

import pytest
import yaml
from fleet_core.fleet_map import load_fleet_map
from fleet_core.fleet_map.keepout import keepout_rects, mask, mask_yaml, pgm, pgm_size

REPO = Path(__file__).resolve().parents[3]
SIM_MAPS = REPO / "sim" / "fleet_gz_sim" / "maps"
ORIGIN, RESOLUTION = (-0.15, -0.15), 0.01
SIZE = (228, 129)  # 1.98 x 0.99 m 경기장에 둘레 0.15 m


def cell(grid, x, y, origin=ORIGIN, resolution=RESOLUTION):
    return grid[int((y - origin[1]) / resolution)][int((x - origin[0]) / resolution)]


def test_the_lane_and_the_docks_are_closed_and_the_yard_is_open():
    grid = mask(load_fleet_map("d2"), *SIZE, RESOLUTION, ORIGIN)
    assert cell(grid, 1.0, 0.89), "express"
    assert cell(grid, 0.18, 0.89) and cell(grid, 0.40, 0.89), "dock and junction"
    assert not cell(grid, 1.0, 0.40) and not cell(grid, 0.13, 0.15), "yard"


def test_the_closed_area_is_wider_by_the_body_half_width():
    site = load_fleet_map("d2")
    grid = mask(site, *SIZE, RESOLUTION, ORIGIN)
    assert cell(grid, 0.20, 0.79 - 0.055), "inside the pad below the dock zone"
    assert not cell(grid, 0.20, 0.79 - 0.075), "outside the pad"
    bare = mask(site, *SIZE, RESOLUTION, ORIGIN, pad=0.0)
    assert not cell(bare, 0.20, 0.79 - 0.055)
    assert len(keepout_rects(site, 0.0)) == len(keepout_rects(site, 0.06))


def test_the_mask_follows_the_origin_and_the_size_of_the_map_it_is_made_for():
    site = load_fleet_map("d2")
    origin = (-1.0, -0.5)
    grid = mask(site, 400, 250, 0.01, origin)
    assert (len(grid), len(grid[0])) == (250, 400)
    assert cell(grid, 1.0, 0.89, origin) and not cell(grid, 1.0, 0.40, origin)
    assert not cell(grid, -0.5, -0.2, origin), "outside the arena nothing is closed"


def test_the_picture_is_a_pgm_with_the_top_row_first():
    grid = [[False, True], [False, False], [True, True]]  # 줄은 아래부터
    data = pgm(grid)
    assert data.startswith(b"P5\n2 3\n255\n") and pgm_size(data) == (2, 3)
    assert data[-6:] == bytes([0, 0, 254, 254, 254, 0]), "closed cells are black"


@pytest.mark.parametrize("layout", ["d2", "d", "c"])
def test_it_matches_the_mask_the_simulation_ships(layout):
    source = SIM_MAPS / f"arena_{layout}_keepout.yaml"
    if not source.exists():
        pytest.skip("the simulation maps are not in this tree")
    info = yaml.safe_load(source.read_text(encoding="utf-8"))
    shipped = (SIM_MAPS / info["image"]).read_bytes()
    grid = mask(load_fleet_map(layout), *pgm_size(shipped), info["resolution"],
                tuple(info["origin"][:2]))
    assert pgm(grid) == shipped


def test_the_yaml_is_what_map_server_reads():
    info = yaml.safe_load(mask_yaml("mask.pgm", 0.01, (-0.15, -0.15)))
    assert info == {"image": "mask.pgm", "mode": "trinary", "resolution": 0.01,
                    "origin": [-0.15, -0.15, 0.0], "negate": 0, "occupied_thresh": 0.65,
                    "free_thresh": 0.25}


def test_the_tool_writes_a_mask_next_to_a_map(tmp_path):
    (tmp_path / "room.pgm").write_bytes(b"P5\n# made by a mapper\n300 200\n255\n" + bytes(60000))
    (tmp_path / "room.yaml").write_text(
        "image: room.pgm\nresolution: 0.01\norigin: [-0.5, -0.4, 0.0]\n", encoding="utf-8")
    done = subprocess.run(
        [sys.executable, str(REPO / "tools" / "make_keepout.py"), "--layout", "d2",
         "--map", str(tmp_path / "room.yaml")], capture_output=True, text=True, check=False)
    assert done.returncode == 0, done.stderr
    info = yaml.safe_load((tmp_path / "room_keepout.yaml").read_text(encoding="utf-8"))
    assert info["image"] == "room_keepout.pgm" and info["origin"] == [-0.5, -0.4, 0.0]
    assert pgm_size((tmp_path / "room_keepout.pgm").read_bytes()) == (300, 200)
