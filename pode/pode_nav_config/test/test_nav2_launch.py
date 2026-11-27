"""nav2.launch.py가 use_composition에 따라 노드 아홉을 프로세스 아홉 또는 컨테이너 하나로 띄우는지 본다.

프로세스는 띄우지 않고 launch의 조건과 namespace 범위로 액션을 훑는다. ROS 2(launch_ros)와 설치된 pode_nav_config가 필요하다."""
import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
launch_ros = pytest.importorskip("launch_ros")
from launch_ros.actions import ComposableNodeContainer, Node  # noqa: E402
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request  # noqa: E402
from rclpy.parameter import parameter_value_to_python  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
NAMES = ["map_server", "map_server_fine", "amcl", "controller_server", "planner_server",
         "behavior_server", "bt_navigator", "lifecycle_manager_localization",
         "lifecycle_manager_navigation"]
PLUGINS = {
    "map_server": "nav2_map_server::MapServer",
    "map_server_fine": "nav2_map_server::MapServer",
    "amcl": "nav2_amcl::AmclNode",
    "controller_server": "nav2_controller::ControllerServer",
    "planner_server": "nav2_planner::PlannerServer",
    "behavior_server": "behavior_server::BehaviorServer",
    "bt_navigator": "nav2_bt_navigator::BtNavigator",
    "lifecycle_manager_localization": "nav2_lifecycle_manager::LifecycleManager",
    "lifecycle_manager_navigation": "nav2_lifecycle_manager::LifecycleManager",
}


def _launch_description():
    spec = importlib.util.spec_from_file_location("nav2_launch", PKG / "launch" / "nav2.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _walk(entities, context, found):
    """조건을 따지며 액션을 방문한다. Node는 실행하지 않고 그때의 context로 펼친다."""
    for entity in entities:
        condition = getattr(entity, "condition", None)
        if condition is not None and not condition.evaluate(context):
            continue
        if isinstance(entity, launch.actions.TimerAction):
            # 타이머는 펼치지 않고 그대로 모은다
            found.append((entity, []))
            continue
        if isinstance(entity, Node):
            entity._perform_substitutions(context)
            requests = []
            if isinstance(entity, ComposableNodeContainer):
                descriptions = entity._ComposableNodeContainer__composable_node_descriptions
                requests = [get_composable_node_load_request(d, context) for d in descriptions
                            if d.condition() is None or d.condition().evaluate(context)]
            found.append((entity, requests))
            continue
        _walk(entity.visit(context) or [], context, found)


def _expand(use_composition, use_sim_time="true", keepout_mask=None, vision_obstacles=None,
            avoidance=None, narrow_strips=None, peer_obstacles=None, peer_robots=None,
            peer_node=None, plan_keep="false", **more):
    # 길 유지는 기본 켬이지만, 그것을 보는 시험 말고는 끈 채로 센다(None이면 launch 기본값)
    context = launch.LaunchContext()
    context.launch_configurations.update({
        "use_composition": use_composition, "use_sim_time": use_sim_time,
        "map": "/maps/coarse.yaml", "map_fine": "/maps/fine.yaml",
        "initial_x": "0.35", "initial_y": "1.55", "initial_yaw": "1.5708",
    })
    if keepout_mask is not None:
        context.launch_configurations["keepout_mask"] = keepout_mask
    if vision_obstacles is not None:
        context.launch_configurations["vision_obstacles"] = vision_obstacles
    if avoidance is not None:
        context.launch_configurations["avoidance_params_file"] = avoidance
    if narrow_strips is not None:
        context.launch_configurations["narrow_strips"] = narrow_strips
    if peer_obstacles is not None:
        context.launch_configurations["peer_obstacles"] = peer_obstacles
    if peer_robots is not None:
        context.launch_configurations["peer_robots"] = peer_robots
    if peer_node is not None:
        context.launch_configurations["peer_node"] = peer_node
    if plan_keep is not None:
        context.launch_configurations["plan_keep"] = plan_keep
    context.launch_configurations.update(more)
    found = []
    _walk(_launch_description().entities, context, found)
    return context, found


def _params(request):
    return {p.name: parameter_value_to_python(p.value) for p in request.parameters}


def test_without_composition_nine_nodes_run_as_today():
    _, found = _expand("false")
    nodes = [node for node, _ in found]
    assert not any(isinstance(n, ComposableNodeContainer) for n in nodes)
    assert [n.node_name for n in nodes] == [f"/pode1/{name}" for name in NAMES]
    executables = [n.node_executable for n in nodes]
    assert executables == ["map_server", "map_server", "amcl", "controller_server",
                           "planner_server", "behavior_server", "bt_navigator",
                           "lifecycle_manager", "lifecycle_manager"]
    for node in nodes[:7]:
        assert node.expanded_remapping_rules == [("/tf", "tf"), ("/tf_static", "tf_static")]


def test_with_composition_one_container_loads_the_nine_nodes():
    _, found = _expand("true")
    assert len(found) == 1
    container, requests = found[0]
    assert isinstance(container, ComposableNodeContainer)
    assert container.node_package == "rclcpp_components"
    assert container.node_executable == "component_container_isolated"
    assert container.node_name == "/pode1/nav2_container"
    assert [r.node_name for r in requests] == NAMES
    assert {r.node_name: r.plugin_name for r in requests} == PLUGINS
    for request in requests:
        assert request.node_namespace == "/pode1"
        assert _params(request)["use_sim_time"] is True
    for request in requests[:7]:  # lifecycle manager는 프로세스로 띄울 때처럼 TF를 쓰지 않는다
        assert list(request.remap_rules) == ["/tf:=tf", "/tf_static:=tf_static"]
    # 컨테이너 안에서 만드는 하위 노드(costmap)를 위해 컨테이너 프로세스에도 같은 remap을 준다
    assert container.expanded_remapping_rules == [("/tf", "tf"), ("/tf_static", "tf_static")]


def test_composed_nodes_get_the_same_parameters_as_the_processes():
    _, [(_, requests)] = _expand("true")
    params = {r.node_name: _params(r) for r in requests}
    # 파라미터 파일의 namespace 뿌리 키(pode1:)가 노드마다 맞아 들어가야 한다
    assert params["controller_server"]["controller_plugins"] == [
        "FollowPath", "FollowPathThrough", "FollowPathReverse", "FollowPathGoal"]
    assert params["amcl"]["map_topic"] == "map_fine"
    assert params["map_server"]["yaml_filename"] == "/maps/coarse.yaml"
    assert params["map_server_fine"]["yaml_filename"] == "/maps/fine.yaml"
    assert params["amcl"]["set_initial_pose"] is True
    assert (params["amcl"]["initial_pose.x"], params["amcl"]["initial_pose.y"],
            params["amcl"]["initial_pose.yaw"]) == (0.35, 1.55, 1.5708)
    assert params["bt_navigator"]["default_nav_to_pose_bt_xml"].endswith(
        "navigate_to_pose_w_replanning_and_recovery_goal.xml")
    loc, nav = params["lifecycle_manager_localization"], params["lifecycle_manager_navigation"]
    assert loc["node_names"] == ["map_server", "map_server_fine", "amcl"]
    assert nav["node_names"] == ["controller_server", "planner_server", "behavior_server",
                                 "bt_navigator"]
    assert loc["autostart"] is True and nav["autostart"] is True


def _timers(found):
    return [e for e, _ in found if isinstance(e, launch.actions.TimerAction)]


def _seconds(context, timer):
    # TimerAction이 기다릴 때 계산하는 그대로
    from launch.utilities import type_utils
    return type_utils.perform_typed_substitution(context, timer.period, float)


@pytest.mark.parametrize("delay", [None, "0", "0.0"])
def test_navigation_comes_up_with_localization_by_default(delay):
    more = {} if delay is None else {"navigation_delay_s": delay}
    _, found = _expand("true", **more)
    assert not _timers(found)
    params = {r.node_name: _params(r) for r in found[0][1]}
    assert params["lifecycle_manager_navigation"]["autostart"] is True


@pytest.mark.parametrize("autostart", ["true", "True", "1"])
def test_navigation_autostart_takes_every_true_spelling(autostart):
    _, found = _expand("true", autostart=autostart)
    params = {r.node_name: _params(r) for r in found[0][1]}
    assert params["lifecycle_manager_navigation"]["autostart"] is True


def test_the_delay_is_fixed_when_the_robot_s_launch_is_read():
    """include로 넘긴 인자는 밖으로 새어 나가 다음 로봇의 값으로 바뀐다. 타이머는 제 값을 지킨다."""
    context, found = _expand("true", navigation_delay_s="40")
    [timer] = _timers(found)
    context.launch_configurations["navigation_delay_s"] = "0.0"
    context.launch_configurations["namespace"] = "pode3"
    assert _seconds(context, timer) == 40.0
    [call] = timer.actions
    name = launch.utilities.perform_substitutions(context, call.cmd[3])
    assert name == "/pode1/lifecycle_manager_navigation/manage_nodes"


def test_the_late_startup_call_has_a_valid_name_without_a_namespace():
    context, found = _expand("false", namespace="", navigation_delay_s="5")
    [timer] = _timers(found)
    [call] = timer.actions
    name = launch.utilities.perform_substitutions(context, call.cmd[3])
    assert name == "/lifecycle_manager_navigation/manage_nodes"


@pytest.mark.parametrize("composition", ["false", "true"])
def test_a_navigation_delay_brings_up_only_the_driving_side_later(composition):
    """시험용: 위치(AMCL)는 먼저 뜨고 controller·planner·bt_navigator만 늦게 켠다."""
    context, found = _expand(composition, navigation_delay_s="40")
    if composition == "true":
        params = {r.node_name: _params(r) for r in found[0][1]}
        loc, nav = params["lifecycle_manager_localization"], params["lifecycle_manager_navigation"]
    else:
        import yaml

        def given(name):
            # launch_ros가 쓴 파라미터 파일(node_names가 tuple이라 safe_load로는 못 읽는다)
            [node] = [n for n, _ in found if isinstance(n, Node) and n.node_name == name]
            [path] = [p for p, is_file in node._Node__expanded_parameter_arguments if is_file]
            [body] = yaml.unsafe_load(Path(path).read_text(encoding="utf-8")).values()
            return body["ros__parameters"]
        loc = given("/pode1/lifecycle_manager_localization")
        nav = given("/pode1/lifecycle_manager_navigation")
    assert loc["autostart"] is True and nav["autostart"] is False
    [timer] = _timers(found)
    assert _seconds(context, timer) == 40.0
    [call] = timer.actions
    cmd = [launch.utilities.perform_substitutions(context, part) for part in call.cmd]
    assert cmd[:3] == ["ros2", "service", "call"]
    assert cmd[3] == "/pode1/lifecycle_manager_navigation/manage_nodes"
    assert cmd[4] == "nav2_msgs/srv/ManageLifecycleNodes" and "command: 0" in cmd[5]


def test_composed_use_sim_time_follows_the_argument():
    _, [(_, requests)] = _expand("true", use_sim_time="false")
    assert all(_params(r)["use_sim_time"] is False for r in requests)


# ---- keepout(급행 레인과 도크 구역을 Nav2 goal 경로에서 닫음) -----------------------

KEEPOUT_NAMES = ["keepout_mask_server", "costmap_filter_info_server",
                 "lifecycle_manager_keepout"]
MASK = "/maps/keepout_mask.yaml"


def _param_files(node):
    """프로세스가 받는 파라미터 파일(launch_ros는 dict도 파일로 쓴다)."""
    import yaml
    out = {}
    for path, is_file in node._Node__expanded_parameter_arguments:
        if is_file:
            data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
            for key, value in data.items():
                out.setdefault(key, {}).update(value) if isinstance(value, dict) else None
    return out


def _global_costmap(node):
    return _param_files(node)["pode1"]["global_costmap"]["global_costmap"]["ros__parameters"]


@pytest.mark.parametrize("mask", [None, ""])
def test_without_a_keepout_mask_nothing_changes(mask):
    _, found = _expand("false", keepout_mask=mask)
    nodes = [node for node, _ in found]
    assert [n.node_name for n in nodes] == [f"/pode1/{name}" for name in NAMES]
    planner = nodes[NAMES.index("planner_server")]
    assert "filters" not in _global_costmap(planner)


def test_a_keepout_mask_adds_the_filter_to_the_global_costmap_and_starts_its_servers():
    _, found = _expand("false", keepout_mask=MASK)
    nodes = {n.node_name.rsplit("/", 1)[1]: n for n, _ in found}
    assert list(nodes) == NAMES + KEEPOUT_NAMES
    glob = _global_costmap(nodes["planner_server"])
    assert glob["filters"] == ["keepout_filter"]
    assert glob["keepout_filter"]["filter_info_topic"] == "/pode1/costmap_filter_info"
    assert glob["plugins"] == ["static_layer", "obstacle_layer", "inflation_layer"]
    local = _param_files(nodes["controller_server"])["pode1"]["local_costmap"]
    assert "filters" not in local["local_costmap"]["ros__parameters"], "FollowPath unaffected"
    info = _param_files(nodes["costmap_filter_info_server"])["pode1"][
        "costmap_filter_info_server"]["ros__parameters"]
    assert info["mask_topic"] == "/pode1/keepout_filter_mask"
    assert nodes["costmap_filter_info_server"].node_executable == "costmap_filter_info_server"
    assert nodes["keepout_mask_server"].node_executable == "map_server"
    assert nodes["keepout_mask_server"].expanded_remapping_rules == [
        ("/tf", "tf"), ("/tf_static", "tf_static")]


def test_the_keepout_servers_are_brought_up_by_their_own_lifecycle_manager():
    _, [(_, requests)] = _expand("true", keepout_mask=MASK)
    params = {r.node_name: _params(r) for r in requests}
    assert [r.node_name for r in requests] == NAMES + KEEPOUT_NAMES
    assert params["lifecycle_manager_keepout"]["node_names"] == [
        "keepout_mask_server", "costmap_filter_info_server"]
    assert params["lifecycle_manager_keepout"]["autostart"] is True
    assert params["keepout_mask_server"]["yaml_filename"] == MASK
    assert params["keepout_mask_server"]["topic_name"] == "keepout_filter_mask"
    plugins = {r.node_name: r.plugin_name for r in requests}
    assert plugins["costmap_filter_info_server"] == "nav2_map_server::CostmapFilterInfoServer"
    assert plugins["keepout_mask_server"] == "nav2_map_server::MapServer"


def test_the_composed_container_carries_the_keepout_filter_for_the_costmap_sub_node():
    _, [(container, _)] = _expand("true", keepout_mask=MASK)
    assert _global_costmap(container)["filters"] == ["keepout_filter"]
    _, [(container, _)] = _expand("true")
    assert "filters" not in _global_costmap(container)


# ---- vision 장애물(회피 담당의 점구름 자리) -----------------------------------------

def _obstacle_layers(found):
    nodes = [node for node, _ in found]
    layers = {}
    for name, which in (("controller_server", "local"), ("planner_server", "global")):
        costmap = _param_files(nodes[NAMES.index(name)])["pode1"][f"{which}_costmap"]
        layers[which] = costmap[f"{which}_costmap"]["ros__parameters"]["obstacle_layer"]
    return layers


@pytest.mark.parametrize("value", [None, "false"])
def test_without_vision_obstacles_the_costmaps_read_only_the_scan(value):
    _, found = _expand("false", vision_obstacles=value)
    for layer in _obstacle_layers(found).values():
        assert layer["observation_sources"] == "scan" and "vision" not in layer


def test_vision_obstacles_adds_the_point_cloud_to_both_costmaps():
    _, found = _expand("false", vision_obstacles="true")
    for layer in _obstacle_layers(found).values():
        assert layer["observation_sources"] == "scan vision"
        assert layer["vision"]["topic"] == "/pode1/vision_obstacles"
        assert layer["vision"]["data_type"] == "PointCloud2"
        assert layer["vision"]["expected_update_rate"] == 0.0, "a silent topic stops nothing"
        assert layer["scan"]["topic"] == "/pode1/scan", "the scan stays as it was"


def test_vision_obstacles_and_keepout_work_together():
    _, found = _expand("false", keepout_mask=MASK, vision_obstacles="true")
    planner = [node for node, _ in found][NAMES.index("planner_server")]
    glob = _global_costmap(planner)
    assert glob["filters"] == ["keepout_filter"]
    assert glob["obstacle_layer"]["observation_sources"] == "scan vision"


# ---- 설정의 주인 ---------------------------------------------------------------------

def _avoidance_file() -> dict:
    import yaml
    path = PKG.parent / "pode_avoidance" / "params" / "avoidance.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_the_goal_controller_comes_from_the_avoidance_file():
    _, found = _expand("false")
    controller = [node for node, _ in found][NAMES.index("controller_server")]
    server = _param_files(controller)["pode1"]["controller_server"]["ros__parameters"]
    theirs = _avoidance_file()["follow_path_goal"]
    assert server["FollowPathGoal"]["plugin"] == theirs["plugin"]
    assert server["FollowPathGoal"]["desired_linear_vel"] == theirs["desired_linear_vel"]
    assert server["FollowPath"]["plugin"], "the user's controllers are there as before"


def test_an_avoidance_file_with_a_user_owned_key_stops_the_launch(tmp_path):
    import yaml
    bad = tmp_path / "avoidance.yaml"
    bad.write_text(yaml.safe_dump({**_avoidance_file(), "local_costmap": {"width": 9}}),
                   encoding="utf-8")
    with pytest.raises(ValueError, match="local_costmap"):
        _expand("false", avoidance=str(bad))


# ---- 스캔 거르개 ---------------------------------------------------------------------

STRIPS = "0.0,0.79,0.30,0.99; 0.30,0.79,0.50,0.99"


@pytest.mark.parametrize("strips", [None, ""])
def test_without_narrow_strips_the_costmaps_read_the_raw_scan_and_no_filter_runs(strips):
    _, found = _expand("false", narrow_strips=strips)
    assert [n.node_name for n, _ in found] == [f"/pode1/{name}" for name in NAMES]
    for layer in _obstacle_layers(found).values():
        assert layer["scan"]["topic"] == "/pode1/scan"


@pytest.mark.parametrize("composition", ["false", "true"])
def test_narrow_strips_start_the_filter_and_only_the_costmaps_read_its_scan(composition):
    _, found = _expand(composition, narrow_strips=STRIPS)
    nodes = [n for n, _ in found]
    [scan_filter] = [n for n in nodes if n.node_name == "/pode1/scan_filter"]
    assert scan_filter.node_package == "pode_scan_filter"
    assert scan_filter.node_executable == "scan_filter"
    assert scan_filter.expanded_remapping_rules == [("/tf", "tf"), ("/tf_static", "tf_static")]
    given = "".join(Path(path).read_text(encoding="utf-8")
                    for path, is_file in scan_filter._Node__expanded_parameter_arguments if is_file)
    assert "wall_band_m: 0.05" in given and STRIPS in given
    if composition == "false":
        layers = _obstacle_layers(found)
        for layer in layers.values():
            assert layer["scan"]["topic"] == "/pode1/scan_filtered"
        amcl = _param_files(nodes[NAMES.index("amcl")])["pode1"]["amcl"]["ros__parameters"]
        assert amcl["scan_topic"] == "scan", "AMCL keeps the raw scan"
    else:
        container = nodes[0]
        costmap = _param_files(container)["pode1"]["local_costmap"]["local_costmap"]
        assert costmap["ros__parameters"]["obstacle_layer"]["scan"]["topic"] == (
            "/pode1/scan_filtered")


# ---- 다른 로봇의 몸을 costmap에(peer 층, 임시: 주행 시험용) ----------------------------------

def _costmaps(found):
    nodes = [node for node, _ in found]
    out = {}
    for name, which in (("controller_server", "local"), ("planner_server", "global")):
        costmap = _param_files(nodes[NAMES.index(name)])["pode1"][f"{which}_costmap"]
        out[which] = costmap[f"{which}_costmap"]["ros__parameters"]
    return out


@pytest.mark.parametrize("value", [None, "false"])
def test_without_peer_obstacles_nothing_changes(value):
    _, found = _expand("false", peer_obstacles=value)
    assert [n.node_name for n, _ in found] == [f"/pode1/{name}" for name in NAMES]
    for costmap in _costmaps(found).values():
        assert "peer_layer" not in costmap["plugins"] and "peer_layer" not in costmap


def test_peer_obstacles_adds_the_layer_to_both_costmaps_and_starts_the_node():
    _, found = _expand("false", peer_obstacles="true", peer_robots="pode1,pode2,pode3")
    nodes = [n for n, _ in found]
    [peer] = [n for n in nodes if n.node_name == "/pode1/peer_obstacles"]
    assert peer.node_package == "pode_peer_obstacles"
    assert peer.node_executable == "peer_obstacles"
    given = "".join(Path(path).read_text(encoding="utf-8")
                    for path, is_file in peer._Node__expanded_parameter_arguments if is_file)
    assert "robot_id: pode1" in given and "pode1,pode2,pode3" in given
    for costmap in _costmaps(found).values():
        at = costmap["plugins"].index("peer_layer")
        assert costmap["plugins"][at + 1] == "inflation_layer"
        assert costmap["peer_layer"]["peers"]["topic"] == "/pode1/peer_obstacles"
        assert costmap["peer_layer"]["sweep"]["topic"] == "/pode1/peer_sweep"
        assert costmap["obstacle_layer"]["observation_sources"] == "scan"


def test_the_composed_container_carries_the_peer_layer_for_the_costmap_sub_nodes():
    _, found = _expand("true", peer_obstacles="true", peer_robots="pode1,pode2")
    container = found[0][0]
    for which in ("local", "global"):
        costmap = _param_files(container)["pode1"][f"{which}_costmap"][f"{which}_costmap"]
        assert "peer_layer" in costmap["ros__parameters"]["plugins"]
    assert [n.node_name for n, _ in found][1:] == ["/pode1/peer_obstacles"]


def test_peer_obstacles_works_with_keepout_and_the_scan_filter():
    _, found = _expand("false", keepout_mask=MASK, narrow_strips=STRIPS, peer_obstacles="true",
                       peer_robots="pode1,pode2")
    costmaps = _costmaps(found)
    assert costmaps["global"]["filters"] == ["keepout_filter"]
    assert costmaps["global"]["plugins"] == ["static_layer", "obstacle_layer", "peer_layer",
                                             "inflation_layer"]
    assert costmaps["local"]["obstacle_layer"]["scan"]["topic"] == "/pode1/scan_filtered"
    names = [n.node_name for n, _ in found]
    assert "/pode1/scan_filter" in names and "/pode1/peer_obstacles" in names


def test_peer_obstacles_without_the_robot_list_stops_the_launch():
    with pytest.raises(ValueError, match="peer_robots"):
        _expand("false", peer_obstacles="true")


def test_the_layer_alone_can_be_asked_for_when_another_process_sends_the_bodies():
    _, found = _expand("false", peer_obstacles="true", peer_node="false")
    assert [n.node_name for n, _ in found] == [f"/pode1/{name}" for name in NAMES]
    for costmap in _costmaps(found).values():
        assert "peer_layer" in costmap["plugins"]


def test_peer_node_alone_starts_nothing():
    _, found = _expand("false", peer_node="true", peer_robots="pode1,pode2")
    assert [n.node_name for n, _ in found] == [f"/pode1/{name}" for name in NAMES]


@pytest.mark.parametrize("name", ["peer_obstacles", "peer_node"])
@pytest.mark.parametrize("value", ["1", "yes", "ture", ""])
def test_a_peer_switch_that_is_not_true_or_false_stops_the_launch(name, value):
    """층을 넣는 쪽과 노드를 띄우는 쪽이 같은 글자를 다르게 읽으면 하나만 켜진다: 둘 다 멈춘다."""
    given = {"peer_obstacles": "true", "peer_node": "true", "peer_robots": "pode1,pode2"}
    given[name] = value
    # 조건 안에서 읽힌 스위치는 launch가 SubstitutionFailure로 싸서 낸다
    from launch.substitutions.substitution_failure import SubstitutionFailure
    with pytest.raises((ValueError, SubstitutionFailure), match=f"{name} must be true or false"):
        _expand("false", **given)


def test_the_peer_switches_take_capital_letters():
    _, found = _expand("false", peer_obstacles="True", peer_node="TRUE",
                       peer_robots="pode1,pode2")
    assert "/pode1/peer_obstacles" in [n.node_name for n, _ in found]
    for costmap in _costmaps(found).values():
        assert "peer_layer" in costmap["plugins"]


# ---- 회피 패키지 바꾸기(avoidance_package) ------------------------------------------------------

AVOIDANCE_ARGS = ["namespace", "use_sim_time", "log_level", "peer_obstacles", "peer_node",
                  "peer_robots", "vision_obstacles", "vision_stand_in"]
# 회피 담당의 패키지 흉내: 인자를 모두 기본값 없이 받는다(nav2.launch.py가 다 넘겨야 뜬다)
TEAM_LAUNCH = """
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMES = {names!r}


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name) for name in NAMES),
        Node(package="team_avoidance", executable="team_vision", name="team_vision",
             parameters=[{{name: LaunchConfiguration(name) for name in NAMES}}]),
    ])
"""
TEAM_GOAL = {"plugin": "nav2_mppi_controller::MPPIController", "vx_max": 0.12}


def _team_package(tmp_path, monkeypatch, peer_layer=None):
    import os

    import yaml
    prefix = tmp_path / "team_prefix"
    share = prefix / "share" / "team_avoidance"
    (share / "params").mkdir(parents=True)
    (share / "launch").mkdir()
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    (index / "team_avoidance").write_text("", encoding="utf-8")
    theirs = {"follow_path_goal": TEAM_GOAL,
              "vision_obstacles": {**_avoidance_file()["vision_obstacles"],
                                   "obstacle_max_range": 3.0}}
    if peer_layer is not None:
        theirs["peer_layer"] = peer_layer
    (share / "params" / "avoidance.yaml").write_text(yaml.safe_dump(theirs), encoding="utf-8")
    (share / "launch" / "avoidance.launch.py").write_text(
        TEAM_LAUNCH.format(names=AVOIDANCE_ARGS), encoding="utf-8")
    monkeypatch.setenv("AMENT_PREFIX_PATH",
                       f"{prefix}{os.pathsep}{os.environ.get('AMENT_PREFIX_PATH', '')}")


def _controller_server(found):
    controller = [node for node, _ in found][NAMES.index("controller_server")]
    return _param_files(controller)["pode1"]["controller_server"]["ros__parameters"]


def _given(node):
    """프로세스가 받는 파라미터를 이름으로 찾는다(launch_ros가 쓰는 노드 키와 상관없이)."""
    import yaml
    params = {}
    for path, is_file in node._Node__expanded_parameter_arguments:
        if is_file:
            for value in yaml.safe_load(Path(path).read_text(encoding="utf-8")).values():
                params.update(value["ros__parameters"])
    return params


def test_the_default_avoidance_package_is_the_temporary_one():
    _, found = _expand("false")
    assert _controller_server(found)["FollowPathGoal"] == _avoidance_file()["follow_path_goal"]


def test_another_avoidance_package_swaps_the_goal_controller_and_the_nodes(tmp_path, monkeypatch):
    _team_package(tmp_path, monkeypatch)
    _, found = _expand("false", avoidance_package="team_avoidance", vision_obstacles="true")
    assert _controller_server(found)["FollowPathGoal"] == TEAM_GOAL
    assert _controller_server(found)["FollowPath"]["plugin"], "the user's controllers stay"
    for layer in _obstacle_layers(found).values():
        assert layer["vision"]["obstacle_max_range"] == 3.0
    [team] = [n for n, _ in found if n.node_name == "/pode1/team_vision"]
    given = _given(team)
    assert set(given) >= set(AVOIDANCE_ARGS)
    assert given["namespace"] == "pode1"
    assert given["vision_obstacles"] is True and given["peer_obstacles"] is False
    assert "/pode1/peer_obstacles" not in [n.node_name for n, _ in found]


def test_an_avoidance_params_file_still_wins_over_the_package(tmp_path, monkeypatch):
    import yaml
    _team_package(tmp_path, monkeypatch)
    mine = tmp_path / "mine.yaml"
    mine.write_text(yaml.safe_dump({"follow_path_goal": {"plugin": "mine::Goal"}}),
                    encoding="utf-8")
    _, found = _expand("false", avoidance=str(mine), avoidance_package="team_avoidance")
    assert _controller_server(found)["FollowPathGoal"] == {"plugin": "mine::Goal"}
    assert "/pode1/team_vision" in [n.node_name for n, _ in found], "nodes stay the package's"


def test_peer_obstacles_with_a_package_without_the_peer_layer_stops_the_launch(
        tmp_path, monkeypatch):
    _team_package(tmp_path, monkeypatch)
    with pytest.raises(ValueError, match="peer_layer"):
        _expand("false", avoidance_package="team_avoidance", peer_obstacles="true",
                peer_node="false")


def test_the_peer_layer_comes_from_the_chosen_package(tmp_path, monkeypatch):
    layer = {**_avoidance_file()["peer_layer"], "enabled": False}
    _team_package(tmp_path, monkeypatch, peer_layer=layer)
    _, found = _expand("false", avoidance_package="team_avoidance", peer_obstacles="true",
                       peer_robots="pode1,pode2")
    for costmap in _costmaps(found).values():
        assert costmap["peer_layer"]["enabled"] is False
    [team] = [n for n, _ in found if n.node_name == "/pode1/team_vision"]
    assert _given(team)["peer_robots"] == "pode1,pode2"


def test_an_unknown_avoidance_package_stops_the_launch():
    from launch.substitutions.substitution_failure import SubstitutionFailure
    with pytest.raises((LookupError, SubstitutionFailure, ValueError)):
        _expand("false", avoidance_package="no_such_avoidance_package")


# ---- 임시 비전 대역(pode_avoidance, 시뮬레이션 전용) ----------------------------------------

@pytest.mark.parametrize("value", [None, "false"])
def test_without_the_vision_stand_in_nothing_changes(value):
    more = {} if value is None else {"vision_stand_in": value}
    _, found = _expand("false", **more)
    assert [n.node_name for n, _ in found] == [f"/pode1/{name}" for name in NAMES]


def test_the_vision_stand_in_starts_its_node_and_turns_the_vision_source_on():
    _, found = _expand("false", vision_stand_in="true")
    [stand_in] = [n for n, _ in found if n.node_name == "/pode1/vision_stand_in"]
    assert stand_in.node_package == "pode_avoidance"
    assert stand_in.node_executable == "vision_stand_in"
    for layer in _obstacle_layers(found).values():
        assert layer["observation_sources"] == "scan vision"
        assert layer["vision"]["topic"] == "/pode1/vision_obstacles"


@pytest.mark.parametrize("name", ["vision_obstacles", "vision_stand_in"])
def test_a_vision_switch_that_is_not_true_or_false_stops_the_launch(name):
    from launch.substitutions.substitution_failure import SubstitutionFailure
    with pytest.raises((ValueError, SubstitutionFailure), match=f"{name} must be true or false"):
        _expand("false", **{name: "ture"})


# ---- 길 유지(pode_plan_keep): 기본 켬(11-26 실물 확인 뒤) ----

def _plan_keep(found):
    [node] = [n for n, _ in found if n.node_name == "/pode1/plan_keep"]
    return node


def _respawn(node, context):
    from launch.utilities.type_utils import perform_typed_substitution
    for owner in ("ExecuteLocal", "ExecuteProcess"):
        if hasattr(node, f"_{owner}__respawn"):
            value = getattr(node, f"_{owner}__respawn")
            return value if isinstance(value, bool) else perform_typed_substitution(
                context, value, bool)
    raise AssertionError("launch keeps respawn somewhere else")


def _bt_xml(found):
    navigator = [n for n, _ in found if n.node_name == "/pode1/bt_navigator"]
    if navigator:
        import yaml
        trees = []
        for path, is_file in navigator[0]._Node__expanded_parameter_arguments:
            if not is_file:
                continue
            for value in yaml.safe_load(Path(path).read_text(encoding="utf-8")).values():
                given = value.get("ros__parameters", {}) if isinstance(value, dict) else {}
                trees += [given["default_nav_to_pose_bt_xml"]] if (
                    "default_nav_to_pose_bt_xml" in given) else []
        [tree] = trees
        return tree
    [(_, requests)] = [(n, r) for n, r in found if r]
    [request] = [r for r in requests if r.node_name == "bt_navigator"]
    return _params(request)["default_nav_to_pose_bt_xml"]


@pytest.mark.parametrize("composition", ["false", "true"])
def test_plan_keep_false_runs_no_node_and_the_default_tree_plans(composition):
    _, found = _expand(composition, plan_keep="false")
    assert not [n for n, _ in found if n.node_name == "/pode1/plan_keep"]
    assert _bt_xml(found).endswith("bt/navigate_to_pose_w_replanning_and_recovery_goal.xml")


@pytest.mark.parametrize("composition", ["false", "true"])
def test_by_default_the_plan_keep_node_runs_with_the_keep_tree(composition):
    context, found = _expand(composition, plan_keep=None)
    node = _plan_keep(found)
    assert node.node_package == "pode_plan_keep" and node.node_executable == "plan_keep"
    assert _respawn(node, context) is True
    assert _bt_xml(found).endswith("bt/navigate_to_pose_w_replanning_and_recovery_goal_keep.xml")


def test_plan_keep_node_false_leaves_the_node_to_another_process():
    _, found = _expand("false", plan_keep="true", plan_keep_node="false")
    assert [n.node_name for n, _ in found] == [f"/pode1/{name}" for name in NAMES]
    assert _bt_xml(found).endswith("_keep.xml")


@pytest.mark.parametrize("composition", ["false", "true"])
def test_plan_keep_true_starts_the_node_and_switches_to_the_keep_tree(composition):
    _, found = _expand(composition, plan_keep="true")
    assert _plan_keep(found).node_package == "pode_plan_keep"
    assert _bt_xml(found).endswith("bt/navigate_to_pose_w_replanning_and_recovery_goal_keep.xml")


def test_a_given_tree_wins_over_the_plan_keep_choice():
    _, found = _expand("false", bt_xml="/trees/mine.xml")
    assert _bt_xml(found) == "/trees/mine.xml"


@pytest.mark.parametrize("name", ["plan_keep", "plan_keep_node"])
def test_a_plan_keep_switch_that_is_not_true_or_false_stops_the_launch(name):
    from launch.substitutions.substitution_failure import SubstitutionFailure
    more = {"plan_keep": "true"} if name == "plan_keep_node" else {}
    with pytest.raises((ValueError, SubstitutionFailure), match=f"{name} must be true or false"):
        _expand("false", **{**more, name: "ture"})
