"""pode_fsm.launch.py arguments. Needs the launch package, not a running ROS graph."""
import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
# The package's own launch/ folder can shadow a missing ROS launch package as an empty namespace
pytest.importorskip("launch.actions")

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "pode_fsm.launch.py"


def defaults() -> dict:
    spec = importlib.util.spec_from_file_location("pode_fsm_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = launch.LaunchContext()
    out = {}
    for action in module.generate_launch_description().entities:
        if isinstance(action, launch.actions.DeclareLaunchArgument):
            out[action.name] = launch.utilities.perform_substitutions(
                context, action.default_value)
    return out


def test_lock_boundary_paths_use_the_through_controller_by_default():
    """pode_nav_config loads FollowPathThrough next to FollowPath; "" switches it off."""
    assert defaults()["through_controller_id"] == "FollowPathThrough"


def test_the_horizon_cap_defaults_to_the_fsm_params():
    """A slower robot (a smaller Nav2 desired_linear_vel) passes its speed here: RPP takes the
    lock-boundary limit as its speed, so the limit must never be above the robot's speed."""
    from pode_fsm.core import FsmParams
    assert float(defaults()["horizon_cap_mps"]) == FsmParams().horizon_cap_mps


def test_no_motion_is_off_by_default():
    """설계 4.5: 동작 없음은 실행 인자 no_motion 하나로 켠다. 기본은 꺼짐(실주행)."""
    assert defaults()["no_motion"] == "false"


def test_the_no_motion_argument_reaches_the_node_parameter():
    text = LAUNCH.read_text(encoding="utf-8")
    assert '"no_motion": ParameterValue(LaunchConfiguration("no_motion"), value_type=bool)' \
        in text
