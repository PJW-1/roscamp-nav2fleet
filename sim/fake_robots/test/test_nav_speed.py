"""가짜 로봇의 최고 속도는 단일 출처(pode_nav_config/params/nav_speed.yaml)를 읽고, 로봇마다
배율(1 이하)을 곱한다. ROS가 필요 없다."""
import random
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from fake_robots.nav_speed import SCALE_RANGE, nav_speed_mps
from fake_robots.scenario import ScenarioError, parse_scenario
from fake_robots.world import Imperfections
from fake_robots.world.runs import jittered
from fleet_core.fleet_map import load_fleet_map
from world_helpers import follow, nav, quiet_world

REPO = Path(__file__).parents[3]
SOURCE = REPO / "pode" / "pode_nav_config" / "params" / "nav_speed.yaml"
TOP = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))["nav_speed_mps"]
SCENARIOS = Path(__file__).parents[1] / "scenarios"


@pytest.fixture(scope="module")
def site():
    return load_fleet_map("d2")


def scenario(robot):
    return {"seed": 1, "speed_factor": 1.0, "timeout_s": 100,
            "stock": {"arm_left": {"A": 2}, "arm_right": {"C": 1}},
            "orders": [{"at_s": 0, "id": "O-1", "items": {"A": 1}}],
            "robots": {"pode1": {"start": "P-1", **robot}}}


def test_the_top_speed_is_read_from_the_one_source():
    assert nav_speed_mps() == TOP


def test_a_bad_source_value_is_refused(tmp_path):
    bad = tmp_path / "nav_speed.yaml"
    bad.write_text("nav_speed_mps: 0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="nav_speed_mps"):
        nav_speed_mps(bad)


def test_the_scale_range_is_the_isaac_scripted_robot_range_below_one():
    assert SCALE_RANGE == (0.67, 1.0)


# ---- 시나리오 로봇(ROS 가짜 노드) ----
def test_a_scenario_robot_drives_at_its_scale_of_the_top_speed(site):
    assert parse_scenario(scenario({}), site).robots["pode1"].speed_mps == TOP
    cfg = parse_scenario(scenario({"speed_scale": 0.8}), site).robots["pode1"]
    assert (cfg.speed_scale, cfg.speed_mps) == (0.8, pytest.approx(0.8 * TOP))


@pytest.mark.parametrize("scale", [1.01, 0, -0.5, True, "fast", float("nan")])
def test_a_scale_above_one_or_not_positive_is_refused(site, scale):
    with pytest.raises(ScenarioError, match="speed_scale"):
        parse_scenario(scenario({"speed_scale": scale}), site)


def test_an_absolute_speed_is_refused_so_no_file_can_outrun_the_source(site):
    with pytest.raises(ScenarioError, match="speed_scale"):
        parse_scenario(scenario({"speed_mps": 0.2}), site)


def test_the_shipped_scenarios_scale_their_robots_inside_the_range():
    low, high = SCALE_RANGE
    for path in sorted(SCENARIOS.glob("*.yaml")):
        robots = yaml.safe_load(path.read_text(encoding="utf-8"))["robots"]
        for name, raw in robots.items():
            assert "speed_mps" not in raw, (path.name, name)
            assert low <= raw.get("speed_scale", 1.0) <= high, (path.name, name)


# ---- 머리 없는 구역 세계(situations) ----
def test_world_robots_drive_at_the_top_speed_whatever_the_fleet_believes():
    world = quiet_world(robots={"pode1": "P-1", "pode2": "P-3"},
                        overrides={"nav_speed_mps": 0.2, "follow_speed_mps": 0.2})
    world.send(nav(world, "pode1", 1, "N_E"))
    world.send(follow(world, "pode2", 2, ["P-3", "N_E"]))
    world.run(0.2)
    assert world.robots["pode1"].run.speed == TOP
    assert world.robots["pode2"].run.speed == TOP


def test_each_world_robot_draws_a_scale_inside_the_range():
    low, high = SCALE_RANGE
    rng = random.Random(7)
    from fake_robots.world.robot import profile_for
    scales = [profile_for(rng, Imperfections()).speed_factor for _ in range(2000)]
    assert low <= min(scales) < low + 0.02 and high - 0.02 < max(scales) <= high


def test_step_jitter_never_takes_a_robot_above_the_top_speed():
    ctx = SimpleNamespace(imperfect=Imperfections(), rng=random.Random(3), top_speed_mps=TOP)
    steps = [jittered(ctx, TOP) for _ in range(2000)]
    assert max(steps) == TOP and min(steps) < TOP
    slow = [jittered(ctx, 0.5 * TOP) for _ in range(2000)]
    assert max(slow) > 0.5 * TOP


def test_a_fast_retreat_command_is_held_to_the_top_speed_even_without_jitter():
    world = quiet_world()
    from fleet_core.common.commands import RetreatCmd
    world.send(RetreatCmd("pode1", 1, 0.3, 2 * TOP))
    world.run(0.2)
    assert jittered(world, world.robots["pode1"].run.speed) == TOP


def test_the_ros_fake_backs_up_no_faster_than_the_top_speed():
    pytest.importorskip("fleet_interfaces.action")
    from fake_robots.motion import Motion
    from fake_robots.pode_goal import _Goal
    from fake_robots.pode_retreat import PodeRetreat

    class Pode(PodeRetreat):
        _motion = Motion(1.0, 0.5)

    goal = _Goal.__new__(_Goal)
    assert Pode()._begin_retreat(0.0, goal, 0.3, 2 * TOP, [])
    assert goal.speed == TOP
