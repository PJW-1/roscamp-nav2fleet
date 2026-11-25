"""pode_fsm.launch.py 인자. launch 패키지는 필요하고 실행 중인 ROS 그래프는 필요 없다."""
import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
# 패키지 자체 launch/ 폴더가 없는 ROS launch 패키지를 빈 namespace로 가릴 수 있다
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
    """pode_nav_config가 FollowPath 옆에 FollowPathThrough를 올리고, 빈 값이면 끈다."""
    assert defaults()["through_controller_id"] == "FollowPathThrough"


def test_the_horizon_cap_defaults_to_the_fsm_params():
    """더 느린 로봇은 자기 속도를 여기 넘긴다(RPP가 잠금 경계 제한을 속도로 쓰므로 제한이 로봇 속도보다 높으면 안 된다)."""
    from pode_fsm.core import FsmParams
    assert float(defaults()["horizon_cap_mps"]) == FsmParams().horizon_cap_mps


def test_no_motion_is_off_by_default():
    """설계 4.5: 동작 없음은 실행 인자 no_motion 하나로 켠다. 기본은 꺼짐(실주행)."""
    assert defaults()["no_motion"] == "false"


def test_the_no_motion_argument_reaches_the_node_parameter():
    text = LAUNCH.read_text(encoding="utf-8")
    assert '"no_motion": ParameterValue(LaunchConfiguration("no_motion"), value_type=bool)' \
        in text
