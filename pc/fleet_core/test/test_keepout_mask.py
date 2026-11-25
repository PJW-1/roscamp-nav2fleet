"""keepout 마스크: 구역 지도에서, 주어진 지도의 크기와 원점에 맞춰 만든다."""
import subprocess
import sys
from pathlib import Path

import pytest
import yaml
from fleet_core.fleet_map import KNOWN_LAYOUTS, load_fleet_map
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


@pytest.mark.parametrize("layout", KNOWN_LAYOUTS)
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


# ---- 구역이 아닌 keepout 사각형(keepout_extra) ----
V7_GAP = (0.99, 0.3475, 1.25, 0.3875)  # v7 남쪽 진열대 사이 건넘 틈
# 칸막이 3과 그 둘레는 없다
V7_DIVIDER_KEEPS = ((0.19, 0.0, 0.31, 0.20), (0.45, 0.0, 0.57, 0.20))


def test_keepout_extra_closes_its_rectangle_without_padding_and_is_no_zone():
    site = load_fleet_map("v7")
    assert set(site.keepout_extra) == set(V7_DIVIDER_KEEPS)
    # 덧대지 않는다: 목적은 계획 차단이고, 덧대면 남쪽 마당이 좁아진다
    for rect in V7_DIVIDER_KEEPS:
        assert rect in keepout_rects(site) and rect in keepout_rects(site, 0.0)
    grid = mask(site, *SIZE, RESOLUTION, ORIGIN)
    # 칸막이 2 둘레: 안은 닫히고 바로 위(y 0.20 너머)와 옆 칸은 열린다
    assert cell(grid, 0.46, 0.12) and cell(grid, 0.565, 0.195)
    assert not cell(grid, 0.51, 0.205) and not cell(grid, 0.64, 0.11)
    # 칸막이 3이 있던 자리는 열린다
    for x in (0.72, 0.77, 0.855):
        assert not cell(grid, x, 0.12) and not cell(grid, x, 0.195), x
    # 구역이 아니다: 그 자리는 여전히 HALL이고 구역 목록과 정원은 그대로다
    assert site.zone_at(0.80, 0.15) == "HALL"
    assert set(site.zones) == {"HALL", "WORK_DOCK", "J_E", "EXPRESS", "OUT_DOCK"}


def test_v7_the_crossing_gap_stays_open_as_a_shortcut():
    """남쪽 진열대 사이 건넘 틈은 지름길이라 막지 않는다: 마스크의 틈 안 칸이 닫히지 않고 그 자리는 HALL이다."""
    site = load_fleet_map("v7")
    assert V7_GAP not in site.keepout_extra
    assert V7_GAP not in keepout_rects(site) and V7_GAP not in keepout_rects(site, 0.0)
    grid = mask(site, *SIZE, RESOLUTION, ORIGIN)
    x0, y0, x1, y1 = V7_GAP
    shut = [(round(x, 3), round(y, 3))
            for x in [x0 + 0.005 + 0.01 * k for k in range(int(round((x1 - x0) / 0.01)))]
            for y in [y0 + 0.005 + 0.01 * k for k in range(int(round((y1 - y0) / 0.01)))]
            if cell(grid, x, y)]
    assert shut == []
    # 서는 로봇이 어느 구역에도 들지 않는 일이 없다: 틈은 HALL이다
    for x in (1.0, 1.12, 1.24):
        for y in (0.355, 0.3675, 0.38):
            assert site.zone_at(x, y) == "HALL", (x, y)


def test_a_dock_left_by_nav_goal_is_open_along_its_way_out():
    """v7의 W는 WORK_DOCK keepout 한가운데라, W -> M_E Nav2 goal이 계획되도록 W에서 넓은 곳까지 곧은 선 둘레(몸 반폭)를 연다. 도크의 나머지는 막혀 있다."""
    from fleet_core.fleet_map.keepout import keepout_openings
    site = load_fleet_map("v7")
    (x0, y0, x1, y1), = keepout_openings(site)
    # 곧은 선이 덧댄 keepout(x 1.72)을 나서는 y까지의 상자에 몸 반폭 0.06을 두르고 덧댄 도크 상자 안으로 자른다
    assert (x0, y0, x1, y1) == pytest.approx((1.72, 0.435, 1.92, 0.5908), abs=1e-3)
    grid = mask(site, *SIZE, RESOLUTION, ORIGIN)
    for x, y in ((1.88, 0.495), (1.80, 0.51), (1.73, 0.53), (1.88, 0.45), (1.88, 0.58)):
        assert not cell(grid, x, y), (x, y)
    for x, y in ((1.88, 0.62), (1.88, 0.40), (1.96, 0.495), (1.75, 0.65), (1.75, 0.36)):
        assert cell(grid, x, y), (x, y)
    assert cell(grid, 1.88, 0.75), "the J_E keepout stays"
    assert keepout_openings(load_fleet_map("d2")) == []


def test_an_opening_clears_only_its_own_docks_keepout():
    """도크 밖 열린 상자가 다른 keepout_extra 사각형과 겹쳐도 그 사각형은 닫혀 있다."""
    from fleet_core.fleet_map import fleet_map_from_text
    text = load_fleet_map("v7").source_text.replace(
        "keepout_extra: [[", "keepout_extra: [[1.80, 0.46, 1.84, 0.50], [", 1)
    site = fleet_map_from_text(text)
    grid = mask(site, *SIZE, RESOLUTION, ORIGIN)
    assert cell(grid, 1.82, 0.48)
    assert not cell(grid, 1.75, 0.55) and not cell(grid, 1.88, 0.58)


def test_layouts_without_keepout_extra_keep_their_masks():
    for layout in ("d2", "d", "c", "d2_shelf"):
        assert load_fleet_map(layout).keepout_extra == ()


def test_a_keepout_extra_outside_the_floor_is_refused():
    from fleet_core.fleet_map import FleetMapError, fleet_map_from_text
    text = load_fleet_map("v7").source_text.replace(
        "keepout_extra: [[", "keepout_extra: [[1.9, 0.9, 2.1, 1.0], [", 1)
    assert "2.1, 1.0" in text
    with pytest.raises(FleetMapError, match="keepout_extra"):
        fleet_map_from_text(text)


def test_the_tool_refuses_a_turned_map(tmp_path):
    (tmp_path / "room.pgm").write_bytes(b"P5\n30 20\n255\n" + bytes(600))
    (tmp_path / "room.yaml").write_text(
        "image: room.pgm\nresolution: 0.01\norigin: [-0.5, -0.4, 0.3]\n", encoding="utf-8")
    done = subprocess.run(
        [sys.executable, str(REPO / "tools" / "make_keepout.py"), "--layout", "d2",
         "--map", str(tmp_path / "room.yaml")], capture_output=True, text=True, check=False)
    assert done.returncode == 2 and "yaw" in done.stderr
    assert not (tmp_path / "room_keepout.yaml").exists(), "no mask for a grid it cannot match"
