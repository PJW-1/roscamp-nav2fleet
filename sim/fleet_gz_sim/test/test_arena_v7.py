"""배치 v7의 Gazebo 경기장과 시나리오: 월드 치수, 지도, keepout, gz_v7(_random)."""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

pytest.importorskip("fleet_core")
pytest.importorskip("fake_robots")
from fleet_core.fleet_map import load_fleet_map  # noqa: E402
from fleet_gz_sim import arena_maps, fleet_setup  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
BOXES = [(0.68, 0.77, 1.56, 0.81), (0.68, 0.365, 0.99, 0.405), (1.25, 0.365, 1.56, 0.405),
         (0.24, 0.0, 0.26, 0.10), (0.50, 0.0, 0.52, 0.10), (0.76, 0.0, 0.78, 0.10)]


@pytest.fixture(scope="module")
def site():
    return load_fleet_map("v7")


def _boxes(path):
    root = ET.fromstring(path.read_text(encoding="utf-8"))
    out = []
    for part in root.iter("collision"):
        if not part.get("name").startswith("shelf_"):
            continue
        pose = [float(v) for v in part.find("pose").text.split()]
        size = [float(v) for v in part.find("geometry/box/size").text.split()]
        out.append((round(pose[0] - size[0] / 2, 4), round(pose[1] - size[1] / 2, 4),
                    round(pose[0] + size[0] / 2, 4), round(pose[1] + size[1] / 2, 4)))
    return out


def test_v7_world_has_the_shelves_and_dividers(site):
    found = fleet_setup.arena_for("v7", PKG)
    assert sorted(_boxes(Path(found.world))) == sorted(BOXES)


def test_v7_charging_stalls_and_express_are_open_in_the_fine_map(site):
    grid = arena_maps.occupancy(arena_maps.FINE_M, site)

    def free(x, y):
        i, j = arena_maps.cell_of(x, y, arena_maps.FINE_M, site)
        return not grid[j][i]
    for x in (0.12, 0.38, 0.64):         # 칸 한가운데
        assert free(x, 0.05)
    assert not free(0.25, 0.05) and not free(0.51, 0.05) and not free(0.77, 0.05)
    assert free(1.0, 0.85) and free(1.0, 0.95)   # 급행 0.18 안


def test_v7_standing_nodes_are_outside_the_keepout(site):
    grid = arena_maps.keepout(arena_maps.FINE_M, site)
    for name in ("N_O", "N_E", "Q1", "Q2", "O_Q", "P-1", "P-2", "P-3"):
        x, y = site.node_pose(name)
        i, j = arena_maps.cell_of(x, y, arena_maps.FINE_M, site)
        assert not grid[j][i], name


@pytest.mark.parametrize("name", ["gz_v7", "gz_v7_random"])
def test_v7_scenarios_fit_the_layout(site, name):
    data = fleet_setup.read_scenario(PKG / "scenarios" / f"{name}.yaml")
    assert data["map"] == "v7"
    assert len(data["orders"]) == 8 and len(data["robots"]) == 3
    for seed in (None, 4):
        fleet_setup.check_fits(data, name, "v7", seed)
    starts = fleet_setup.start_nodes(site, data)
    assert list(starts.values()) == ["P-3", "P-2", "P-1"]


def test_v7_random_scenario_has_the_random_block():
    data = fleet_setup.read_scenario(PKG / "scenarios" / "gz_v7_random.yaml")
    assert data["random"] == {"order_gap_s": [5, 40],
                              "low_battery": {"from_s": 20, "to_s": 150, "charge_s": 30}}
    assert fleet_setup.has_low_battery(data, 4)


@pytest.mark.parametrize("seed", [None, 4])
def test_v7_random_scenario_expands_to_one_low_battery_per_robot_and_spread_orders(seed):
    from fake_robots.scenario import expand_random
    data = fleet_setup.read_scenario(PKG / "scenarios" / "gz_v7_random.yaml")
    out = expand_random(data, seed)
    low = [f for f in out["faults"] if f["type"] == "low_battery"]
    assert sorted(f["robot"] for f in low) == ["pinky1", "pinky2", "pinky3"]
    assert all(20 <= f["at_s"] <= 150 and f["duration_s"] == 30 for f in low)
    times = [0.0] + [order["at_s"] for order in out["orders"]]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert len(gaps) == 8 and all(5 - 0.05 <= gap <= 40 + 0.05 for gap in gaps), gaps


def test_v7_committed_maps_are_the_generator_output(tmp_path, site):
    from fleet_gz_sim import arena
    written = arena.generate(tmp_path, site)
    for name in written:
        assert (tmp_path / name).read_bytes() == (PKG / name).read_bytes(), name
    for suffix, resolution in (("", arena_maps.COARSE_M), ("_fine", arena_maps.FINE_M),
                               ("_keepout", arena_maps.FINE_M)):
        text = (PKG / "maps" / f"arena_v7{suffix}.yaml").read_text(encoding="utf-8")
        assert f"image: arena_v7{suffix}.pgm\n" in text
        assert f"resolution: {resolution}\n" in text
        assert "origin: [-0.15, -0.15, 0.0]\n" in text
