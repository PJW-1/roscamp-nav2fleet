"""traffic/api.py의 TrafficApi가 FleetManager와 ROS 껍데기가 교통 층에 부르는 것을 모두 적는다.

3단계의 ZoneTraffic은 이 목록을 채우면 manager.py를 고치지 않고 끼워진다.
"""
import ast
import importlib
import typing
from pathlib import Path

import fleet_core
import pytest
from fleet_core.traffic.api import TrafficApi

PACKAGE = Path(fleet_core.__file__).parent
FLEET_MANAGER_NODE = (Path(__file__).resolve().parents[2] / "fleet_manager" / "fleet_manager"
                      / "fleet_manager_node.py")
# 지운 레인 제어기의 공개 메서드(1da1e61 traffic/controller.py)와 FleetManager가 더 부르는 것
LANE_CONTROLLER_API = {
    "on_robot_state", "on_run_mode", "on_dispatch", "on_dispatch_cancel", "on_release",
    "on_robot_fault", "on_move_result", "on_goal_rejected", "on_goal_not_sent", "on_goal_lost",
    "on_retreat_result", "on_dock_result", "robots", "everyone_placed", "running", "available",
    "tick", "tick_begin", "tick_progress", "tick_pick", "step", "take_output", "zones_view",
    "robot_view", "status_view",
}
ALSO_CALLED = {"zones", "confirm_clear", "clear_reason", "reinstate", "reinstate_refusal"}
OPTIONAL = {"station_left"}  # FleetManager가 있을 때만 채운다(hasattr)
KEPT = ("mission", "selector", "deadlock", "modes", "api", "site", "zone.controller")


def declared() -> set[str]:
    return {name for name in vars(TrafficApi) if not name.startswith("_")} | set(
        typing.get_type_hints(TrafficApi))


def used_on(path: Path, owner: str) -> set[str]:
    """`<owner>.traffic.<이름>` 꼴로 쓰는 이름."""
    found = set()
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Attribute)
                and node.value.attr == "traffic"):
            base = node.value.value
            if (isinstance(base, ast.Name) and base.id == owner) or (
                    isinstance(base, ast.Attribute) and base.attr == owner):
                found.add(node.attr)
    return found


def test_the_api_holds_the_public_methods_of_the_removed_lane_controller():
    assert LANE_CONTROLLER_API | ALSO_CALLED <= declared()


def test_everything_the_manager_calls_on_traffic_is_in_the_api():
    used = used_on(PACKAGE / "manager.py", "self")
    assert used, "the walk found nothing, so it would pass anything"
    assert used - OPTIONAL <= declared()


@pytest.mark.skipif(not FLEET_MANAGER_NODE.is_file(), reason="only in the source tree")
def test_everything_the_ros_shell_calls_on_traffic_is_in_the_api():
    used = used_on(FLEET_MANAGER_NODE, "core")
    assert used and used <= declared()


def test_every_api_member_says_what_the_caller_expects():
    for name in vars(TrafficApi):
        member = getattr(TrafficApi, name)
        if not name.startswith("_") and callable(member) or isinstance(member, property):
            assert (member.__doc__ or "").strip(), name


@pytest.mark.parametrize("name", KEPT)
def test_the_kept_traffic_modules_import_on_their_own(name):
    importlib.import_module(f"fleet_core.traffic.{name}")
