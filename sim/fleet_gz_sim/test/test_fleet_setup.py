"""gz.launch.py가 배치와 시나리오에서 정하는 것: 경기장 파일, Pode 자리, 돌릴 수 없는 시나리오."""
import math
from pathlib import Path

import pytest
import yaml

pytest.importorskip("fleet_core")
pytest.importorskip("fake_robots")
from fleet_core.fleet_map import KNOWN_LAYOUTS, load_fleet_map  # noqa: E402
from fleet_gz_sim import fleet_setup  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
SCENARIOS = PKG / "scenarios"


def write(tmp_path, data, name="s.yaml") -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


@pytest.mark.parametrize("layout", KNOWN_LAYOUTS)
def test_each_layout_has_its_arena_files(layout):
    found = fleet_setup.arena_for(layout, PKG)
    assert found.layout == layout
    for path in (found.world, found.nav_map, found.nav_map_fine, found.keepout_mask):
        assert Path(path).is_file() and f"arena_{layout}" in Path(path).name


def test_an_unknown_layout_or_missing_files_are_refused(tmp_path):
    with pytest.raises(fleet_setup.ScenarioUnfit, match="layout"):
        fleet_setup.arena_for("warehouse", PKG)
    with pytest.raises(fleet_setup.ScenarioUnfit, match="python3 -m fleet_gz_sim.arena"):
        fleet_setup.arena_for("d2", tmp_path)


@pytest.mark.parametrize("variant", ["a", "b", "ab", "a1"])
def test_a_nav_map_variant_swaps_only_the_costmap_map(variant):
    """가운데 통로 갈래: 5 cm 지도만 바뀐다. 1 cm 지도(AMCL)와 keepout 마스크는 그대로."""
    base = fleet_setup.arena_for("v7", PKG)
    found = fleet_setup.arena_for("v7", PKG, variant)
    assert Path(found.nav_map) == PKG / "maps" / f"arena_v7_{variant}.yaml"
    assert Path(found.nav_map).is_file()
    assert found.world == base.world and found.nav_map_fine == base.nav_map_fine
    assert found.keepout_mask == base.keepout_mask and found.layout == "v7"


def test_v7_uses_the_2p5_cm_map_by_default_and_base_gives_the_5_cm_map():
    """v7 기본은 갈래 a(2.5 cm). base는 5 cm 지도(비교용)."""
    assert fleet_setup.arena_for("v7", PKG) == fleet_setup.arena_for("v7", PKG, "")
    assert Path(fleet_setup.arena_for("v7", PKG).nav_map) == PKG / "maps" / "arena_v7_a.yaml"
    assert Path(fleet_setup.arena_for("v7", PKG, "base").nav_map) == PKG / "maps" / "arena_v7.yaml"


def test_other_layouts_keep_their_plain_map():
    assert Path(fleet_setup.arena_for("d2", PKG).nav_map).name == "arena_d2.yaml"
    assert fleet_setup.arena_for("d2", PKG, "base") == fleet_setup.arena_for("d2", PKG)


def test_an_unknown_nav_map_variant_or_a_layout_without_it_is_refused():
    with pytest.raises(fleet_setup.ScenarioUnfit, match="nav_map_variant"):
        fleet_setup.arena_for("v7", PKG, "c")
    # d2에 갈래를 주면 고칠 수 없는 생성 명령을 권하지 않는다
    with pytest.raises(fleet_setup.ScenarioUnfit, match="only for layout v7") as err:
        fleet_setup.arena_for("d2", PKG, "b")
    assert "python3 -m" not in str(err.value)


def test_the_test_layout_has_a_three_robot_gazebo_scenario():
    data = fleet_setup.read_scenario(SCENARIOS / "gz_d2_shelf.yaml")
    assert data["map"] == "d2_shelf"
    assert len(data["orders"]) == 8 and len(data["robots"]) == 3
    fleet_setup.check_fits(data, "gz", "d2_shelf")
    site = load_fleet_map("d2_shelf")
    assert list(fleet_setup.start_nodes(site, data).values()) == list(site.parking)


@pytest.mark.parametrize("layout", ["d2", "d", "c"])
def test_the_gazebo_scenarios_run_three_robots_with_8_orders_and_one_robot(layout):
    site = load_fleet_map(layout)
    three = fleet_setup.read_scenario(SCENARIOS / f"gz_{layout}.yaml")
    one = fleet_setup.read_scenario(SCENARIOS / f"gz_{layout}_one.yaml")
    assert three["map"] == one["map"] == layout
    assert len(three["orders"]) == 8 and len(three["robots"]) == 3
    assert len(one["robots"]) == 1
    for data in (three, one):
        fleet_setup.check_fits(data, "gz", layout)
        starts = fleet_setup.start_nodes(site, data)
        assert list(starts.values()) == list(site.parking[:len(starts)])


def test_robots_start_on_their_nodes_facing_the_node_yaw():
    site = load_fleet_map("d2")
    poses = fleet_setup.start_poses(site, {"pode1": "P-3", "pode2": "N_W"})
    assert poses["pode1"] == pytest.approx((1.85, 0.15, math.pi / 2))
    assert poses["pode2"] == pytest.approx((0.35, 0.65, 0.0))  # N_W has no yaw


def test_robots_without_a_start_get_the_free_parking_nodes_in_order():
    site = load_fleet_map("d2")
    data = {"robots": {"pode1": {}, "pode2": {"start": "P-1"}, "pode3": None}}
    assert fleet_setup.start_nodes(site, data) == {"pode1": "P-2", "pode2": "P-1",
                                                   "pode3": "P-3"}


@pytest.mark.parametrize("robots, words", [
    ({"pode1": {"start": "P-9"}}, "not a node"),
    ({"pode1": {"start": "P-1"}, "pode2": {"start": "P-1"}}, "one node"),
    ({}, "no robots"),
    ({f"pode{i}": {} for i in range(1, 5)}, "no parking node left"),
])
def test_bad_starts_are_refused(robots, words):
    with pytest.raises(fleet_setup.ScenarioUnfit, match=words):
        fleet_setup.start_nodes(load_fleet_map("d2"), {"robots": robots})


def test_a_scenario_for_another_layout_is_refused():
    data = fleet_setup.read_scenario(SCENARIOS / "gz_c.yaml")
    with pytest.raises(fleet_setup.ScenarioUnfit, match="layout 'c'"):
        fleet_setup.check_fits(data, "gz_c.yaml", "d2")


def test_a_scenario_without_a_map_is_for_v7():
    assert fleet_setup.DEFAULT_LAYOUT == "v7"
    assert fleet_setup.scenario_layout({}) == "v7"
    with pytest.raises(fleet_setup.ScenarioUnfit):
        fleet_setup.scenario_layout({"map": "warehouse"})


def test_a_scenario_with_robot_faults_is_refused():
    data = {"map": "d2", "robots": {"pode1": {}},
            "faults": [{"type": "stuck", "robot": "pode1", "at_s": 5}]}
    with pytest.raises(fleet_setup.ScenarioUnfit, match="stuck"):
        fleet_setup.check_fits(data, "s.yaml", "d2")


def test_a_low_battery_runs_in_the_arena_the_stand_in_reports_it(tmp_path):
    data = {"map": "d2", "robots": {"pode1": {}}, "dock": {"duration_s": 4.5},
            "faults": [{"type": "low_battery", "robot": "pode1", "at_s": 5,
                        "duration_s": 30}]}
    fleet_setup.check_fits(data, "s.yaml", "d2")
    assert fleet_setup.has_low_battery(data)
    inputs = fleet_setup.stand_in_inputs(write(tmp_path, data))
    assert inputs.dock_s == 4.5 and inputs.faults[0]["robot"] == "pode1"
    assert not fleet_setup.has_low_battery({"robots": {"pode1": {}}})


def test_a_random_low_battery_is_seen_with_the_given_seed():
    data = {"seed": 3, "robots": {"pode1": {}},
            "random": {"low_battery": {"from_s": 10, "to_s": 20, "charge_s": 30}}}
    assert fleet_setup.has_low_battery(data, seed=11)


def test_a_scenario_is_a_name_in_the_shares_in_order_or_a_file(tmp_path):
    first, second = tmp_path / "a", tmp_path / "b"
    for folder in (first, second):
        (folder / "scenarios").mkdir(parents=True)
        (folder / "scenarios" / "x.yaml").write_text("map: d2\n", encoding="utf-8")
    assert fleet_setup.scenario_path("x", first, second) == first / "scenarios" / "x.yaml"
    path = write(tmp_path, {"map": "d2"})
    assert fleet_setup.scenario_path(str(path), first) == path
    with pytest.raises(fleet_setup.ScenarioUnfit, match="nowhere"):
        fleet_setup.scenario_path("missing", first)


def test_an_obstacle_schedule_is_off_a_share_name_or_a_file(tmp_path):
    from fleet_gz_sim.obstacles import ScheduleError
    assert fleet_setup.obstacles_path("", PKG) == ""
    assert fleet_setup.obstacles_path("d2_yard_express", PKG).endswith("d2_yard_express.yaml")
    path = write(tmp_path, {"obstacles": []}, "o.yaml")
    assert fleet_setup.obstacles_path(str(path), PKG) == str(path)
    with pytest.raises(ScheduleError, match="nowhere"):
        fleet_setup.obstacles_path("roam", PKG)


def test_narrow_strips_are_the_dock_junction_and_lane_rectangles_not_the_hall():
    """스캔 거르개가 받을 좁은 띠: 넓은 구역(shared)이 아닌 구역의 사각형."""
    site = load_fleet_map("d2")
    strips = fleet_setup.narrow_strips(site)
    # 경기장 벽에 닿은 변은 band(0.05)만큼 벽 너머로 넓힌다: 벽 너머로 잰 잡음 점도 띠 안에 든다
    assert strips.split("; ") == [
        "-0.05,0.79,0.3,1.04", "0.3,0.79,0.5,1.04", "0.5,0.79,1.48,1.04", "1.48,0.79,1.68,1.04",
        "1.68,0.79,2.03,1.04",
        "0.45,0.725,0.55,0.84", "1.43,0.725,1.53,0.84"]


def test_narrow_strips_add_a_box_around_each_shelf_end_face():
    """진열대 끝면마다 band만큼 사방으로 넓힌 사각형을 더한다(후진 모퉁이)."""
    site = load_fleet_map("d2")
    strips = fleet_setup.narrow_strips(site, band=0.03).split("; ")
    assert strips[-2:] == ["0.47,0.745,0.53,0.82", "1.45,0.745,1.51,0.82"]
    # v7은 시뮬 진열대(sim_shelves)로 그린다
    v7 = fleet_setup.narrow_strips(load_fleet_map("v7"), band=0.03).split("; ")
    assert "0.62,0.705,0.68,0.805" in v7 and "1.53,0.705,1.59,0.805" in v7
    # 세로 진열대는 위아래 끝면에 더한다.
    assert fleet_setup.shelf_end_boxes([(1.0, 0.2, 1.02, 0.6)], 0.05) == [
        (0.95, 0.15, 1.07, 0.25), (0.95, 0.55, 1.07, 0.65)]
