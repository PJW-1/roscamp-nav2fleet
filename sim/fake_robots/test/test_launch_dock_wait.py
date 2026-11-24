"""fakes.launch.py가 실물 로봇용 도킹 대역을 띄우는 조건."""
import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip("launch_ros")

from launch import LaunchContext  # noqa: E402
from launch.utilities import perform_substitutions  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

FAKES = Path(__file__).parents[1] / "launch" / "fakes.launch.py"
SCENARIO = Path(__file__).parents[1] / "scenarios" / "real_two_orders.yaml"


def _module():
    spec = importlib.util.spec_from_file_location("fakes_launch", FAKES)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _setup(**over):
    context = LaunchContext()
    context.launch_configurations.update({
        "scenario": str(SCENARIO), "fake_robots": "", "fake_arms": "arm_left arm_right arm_ship",
        "fake_central": "true", "use_sim_clock": "false", "result_dir": "/tmp/x",
        "monitor": "false", "map_file": "v7", "contract_file": "", "params_file": "",
        "dock_wait_robots": "", **over})
    return _module()._setup(context)


def _executables(actions):
    return [a.node_executable for a in actions if isinstance(a, Node)]


def _text(value):
    """launch가 치환 묶음으로 바꿔 둔 키와 값을 되돌린다. 값은 launch_ros처럼 YAML로 읽는다."""
    if isinstance(value, (list, tuple)) and value and hasattr(value[0], "perform"):
        return yaml.safe_load(perform_substitutions(LaunchContext(), list(value)))
    return value


def _params(actions, executable):
    node = next(a for a in actions if isinstance(a, Node) and a.node_executable == executable)
    merged = {}
    for p in node._Node__parameters:
        merged.update({_text(k): _text(v) for k, v in p.items()})
    return merged


def test_declared_and_off_by_default():
    declared = {a.name: a.default_value for a in _module().generate_launch_description().entities
                if hasattr(a, "default_value")}
    assert "dock_wait_robots" in declared
    assert "real_dock_wait_node" not in _executables(_setup())


def test_real_robots_get_the_dock_wait_node():
    actions = _setup(dock_wait_robots="pode1 pode2")
    assert _executables(actions).count("real_dock_wait_node") == 1
    params = _params(actions, "real_dock_wait_node")
    assert params["robots"] == "pode1 pode2"
    assert params["use_sim_time"] is False
    assert params["map_file"] == "v7"
    assert params["scenario_file"] == str(SCENARIO)


def test_refused_with_sim_clock():
    with pytest.raises(RuntimeError, match="use_sim_clock"):
        _setup(dock_wait_robots="pode1", use_sim_clock="true")


def test_refused_for_a_faked_robot():
    with pytest.raises(RuntimeError, match="pode2"):
        _setup(dock_wait_robots="pode1 pode2", fake_robots="pode2")


def test_none_means_no_fake_robots():
    """ros2 launch는 빈 값(fake_robots:=)을 받지 않는다. none이 '가짜 로봇 없음'이다."""
    actions = _setup(fake_robots="none", dock_wait_robots="pode1 pode2")
    assert "fake_pode_node" not in _executables(actions)
    assert _executables(actions).count("real_dock_wait_node") == 1
