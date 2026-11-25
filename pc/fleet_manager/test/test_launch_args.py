"""파일 지정 인자가 가짜 노드에도 전달된다."""
import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch_ros")

from ament_index_python.packages import (  # noqa: E402
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchContext  # noqa: E402
from launch.actions import IncludeLaunchDescription  # noqa: E402
from launch.utilities import (  # noqa: E402
    normalize_to_list_of_substitutions,
    perform_substitutions,
)

PC_LAUNCH = Path(__file__).parents[3] / "bringup" / "iponic_bringup" / "launch" / "pc.launch.py"
FILES = {"map_file": "/m.yaml", "params_file": "/p.yaml", "contract_file": "/c.yaml"}


def test_file_overrides_are_forwarded_to_the_fake_launch():
    try:
        get_package_share_directory("fake_robots")
    except PackageNotFoundError:
        pytest.skip("fake_robots is not built in this workspace")
    spec = importlib.util.spec_from_file_location("pc_launch", PC_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        "use_fake": "true", "use_sim_clock": "true", "scenario": "basic_orders",
        "robots": "pode1", "fake_robots": "pode1", "fake_arms": "arm_left", "fake_central": "true",
        "result_dir": "/tmp/x", "record": "", **FILES,
    })
    include = next(a for a in module._setup(context) if isinstance(a, IncludeLaunchDescription))

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    forwarded = {text(k): text(v) for k, v in include.launch_arguments}
    assert {k: forwarded.get(k) for k in FILES} == FILES


def test_order_arguments_become_node_parameters_only_when_given():
    spec = importlib.util.spec_from_file_location("pc_launch", PC_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    declared = {a.name for a in module.generate_launch_description().entities
                if hasattr(a, "default_value")}
    assert {"use_order_model", "lookahead_s"} <= declared
    assert module._order_params(lambda name: "") == {}
    given = {"use_order_model": "true", "lookahead_s": "3.5"}
    assert module._order_params(given.get) == {"use_order_model": True, "lookahead_s": 3.5}
    assert module._order_params({"use_order_model": "false"}.get) == {"use_order_model": False}


def test_real_test_arguments_are_declared_and_forwarded():
    """감시 노드를 끄고 PC 도킹 대역을 켜는 인자가 fakes.launch.py까지 간다."""
    try:
        get_package_share_directory("fake_robots")
    except PackageNotFoundError:
        pytest.skip("fake_robots is not built in this workspace")
    spec = importlib.util.spec_from_file_location("pc_launch", PC_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    declared = {a.name: a.default_value for a in module.generate_launch_description().entities
                if hasattr(a, "default_value")}
    assert {"monitor", "dock_wait_robots"} <= set(declared)
    context = LaunchContext()
    context.launch_configurations.update({
        "use_fake": "true", "use_sim_clock": "false", "scenario": "basic_orders",
        "robots": "pode1 pode2", "fake_robots": "none", "fake_arms": "arm_left arm_right arm_ship",
        "fake_central": "true", "result_dir": "/tmp/x", "record": "", "map_file": "v7",
        "params_file": "", "contract_file": "", "monitor": "false",
        "dock_wait_robots": "pode1 pode2",
    })
    include = next(a for a in module._setup(context) if isinstance(a, IncludeLaunchDescription))

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    forwarded = {text(k): text(v) for k, v in include.launch_arguments}
    assert forwarded["monitor"] == "false"
    assert forwarded["dock_wait_robots"] == "pode1 pode2"
    assert forwarded["fake_robots"] == "none"
    doc = PC_LAUNCH.read_text(encoding="utf-8")
    # 빈 값은 ros2 launch가 거부한다
    assert "fake_robots:= " not in doc and "fake_robots:=none" in doc
