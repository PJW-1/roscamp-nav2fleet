"""one_pode.launch.py가 무엇을 띄우기 전에 격리를 검사하는지 본다."""
import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
pytest.importorskip("rclpy")

LAUNCH = Path(__file__).parents[1] / "launch" / "one_pode.launch.py"
URDF = Path(__file__).parents[1] / "urdf" / "pode_sim.urdf.xacro"
FAST_DDS = ("ROS_STATIC_PEERS", "RMW_IMPLEMENTATION", "ROS_DISCOVERY_SERVER",
            "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
            "FASTDDS_ENVIRONMENT_FILE", "FASTDDS_BUILTIN_TRANSPORTS")


@pytest.fixture
def one(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location("one_pode_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv("ROS_DOMAIN_ID", "44")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    for name in FAST_DDS:
        monkeypatch.delenv(name, raising=False)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(module, "get_rmw_implementation_identifier", lambda: "rmw_fastrtps_cpp")
    return module


def run(module, **overrides):
    """ros2 launch처럼 인자 기본값을 넣고(overrides로 바꾸고) OpaqueFunction을 실행한다."""
    context = launch.LaunchContext()
    *args, start = module.generate_launch_description().entities
    for action in args:
        action.visit(context)
    context.launch_configurations.update(overrides)
    return start.execute(context)


def test_the_default_layout_is_the_real_arena_v7(one):
    assert one.ARGS["layout"][0] == "v7"
    assert one.ARGS["start"][0] == ""  # 비면 drive_check 기본 계획의 시작 지점


def spawned_at(one, monkeypatch, **overrides):
    """pode()에 넘긴 (x, y, yaw)."""
    need("pinky_description", "pode_nav_config", "pode_fsm")
    from fleet_gz_sim import launch_parts
    seen = []
    monkeypatch.setattr(launch_parts, "pode", lambda ns, x, y, yaw, **kw: seen.append((x, y, yaw))
                        or [])
    run(one, **overrides)
    [pose] = seen
    return pose


def test_without_layout_and_start_the_pode_stands_where_drive_check_starts(one, monkeypatch):
    from fleet_core.fleet_map import load_fleet_map
    from fleet_gz_sim.stage2_plan import pick_layout_plan, start_node
    layout, plan = pick_layout_plan(None, None)
    site = load_fleet_map(layout)
    x, y, _ = spawned_at(one, monkeypatch)
    assert (x, y) == site.node_pose("O_IN")
    # drive_check의 시작 검사(START_M 0.15 m)를 통과한다
    assert math.dist((x, y), site.node_pose(start_node(plan))) <= 0.15


def test_d2_without_start_stands_on_p1(one, monkeypatch):
    from fleet_core.fleet_map import load_fleet_map
    x, y, _ = spawned_at(one, monkeypatch, layout="d2")
    assert (x, y) == load_fleet_map("d2").node_pose("P-1")


@pytest.mark.parametrize("layout, node", [("v7", "O_IN"), ("d2", "P-1"), ("d", "P-1"),
                                          ("c", "P-1"), ("d2_shelf", "P-1")])
def test_without_start_every_layout_stands_on_its_plan_start_with_its_yaw(
        one, monkeypatch, layout, node):
    import fleet_core
    import yaml
    from fleet_gz_sim.stage2_plan import pick_layout_plan, start_node
    assert start_node(pick_layout_plan(layout, None)[1]) == node
    # 정답은 지도 yaml 원문에서 바로 읽는다(launch가 쓰는 start_poses를 거치지 않게)
    data = Path(fleet_core.__file__).parent / "data" / f"fleet_map_{layout}.yaml"
    raw = yaml.safe_load(data.read_text())["nodes"][node]
    want = (raw["x"], raw["y"], math.radians(raw.get("yaw_deg", 0.0)))
    assert spawned_at(one, monkeypatch, layout=layout) == pytest.approx(want)


def test_an_explicit_start_wins(one, monkeypatch):
    from fleet_core.fleet_map import load_fleet_map
    x, y, _ = spawned_at(one, monkeypatch, start="P-2")
    assert (x, y) == load_fleet_map("v7").node_pose("P-2")
    x, y, _ = spawned_at(one, monkeypatch, layout="d2", start="P-3")
    assert (x, y) == load_fleet_map("d2").node_pose("P-3")


def test_the_fixed_line_check_is_off_unless_asked(one, monkeypatch):
    need("pinky_description", "pode_nav_config", "pode_fsm")
    from fleet_gz_sim import launch_parts
    seen = {}
    monkeypatch.setattr(launch_parts, "pode", lambda *a, **kw: seen.update(kw) or [])
    run(one)
    assert seen["fixed_line_check"] is False


@pytest.mark.parametrize("text, off", [("true", False), ("false", True)])
def test_the_fixed_line_check_switch_reaches_pode(one, monkeypatch, text, off):
    need("pinky_description", "pode_nav_config", "pode_fsm")
    from fleet_gz_sim import launch_parts
    seen = {}
    monkeypatch.setattr(launch_parts, "pode", lambda *a, **kw: seen.update(kw) or [])
    run(one, fixed_line_collision_check=text)
    assert seen["fixed_line_check"] is not off
    with pytest.raises(ValueError, match="fixed_line_collision_check"):
        run(one, fixed_line_collision_check="no")


def test_domain_6_starts_nothing(one, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "6")
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID"):
        run(one)


def test_other_rmw_starts_nothing(one, monkeypatch):
    monkeypatch.setattr(one, "get_rmw_implementation_identifier", lambda: "rmw_cyclonedds_cpp")
    with pytest.raises(RuntimeError, match="rmw_cyclonedds_cpp"):
        run(one)


def need(*packages):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    for package in packages:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            pytest.skip(f"{package} is not installed")


def environment(actions) -> dict:
    return {"".join(s.perform(None) for s in a.name): "".join(s.perform(None) for s in a.value)
            for a in actions if type(a).__name__ == "SetEnvironmentVariable"}


def test_isolated_launch_keeps_gazebo_transport_on_this_computer(one, monkeypatch):
    need("pinky_description", "pode_nav_config", "pode_fsm")
    monkeypatch.delenv("GZ_IP", raising=False)
    monkeypatch.delenv("GZ_PARTITION", raising=False)
    actions = run(one)
    env = environment(actions)
    assert env["GZ_IP"] == "127.0.0.1" and env["GZ_PARTITION"] == "fleet_gz_44"
    kinds = [type(a).__name__ for a in actions]
    assert "ExecuteProcess" in kinds and "IncludeLaunchDescription" in kinds


def test_gazebo_ip_from_the_shell_is_not_used(one, monkeypatch):
    """셸에 LAN 주소가 GZ_IP로 남아 있어도 Gazebo는 이 컴퓨터 안에서만 통신한다."""
    need("pinky_description", "pode_nav_config", "pode_fsm")
    monkeypatch.setenv("GZ_IP", "192.168.0.10")
    assert environment(run(one))["GZ_IP"] == "127.0.0.1"


def test_one_bridge_carries_the_clock_and_the_robot(one, monkeypatch):
    """gz.launch.py와 같은 조각을 쓴다. 한 대일 때도 브리지 프로세스는 하나다."""
    need("pinky_description", "pode_nav_config", "pode_fsm")
    from fleet_gz_sim import launch_parts
    made = []
    monkeypatch.setattr(launch_parts, "Node", lambda *a, **kw: made.append(kw) or kw)
    run(one)
    [bridge] = [kw for kw in made if kw["executable"] == "parameter_bridge"]
    topics = [a.split("@")[0] for a in bridge["arguments"]]
    assert topics.count("/clock") == 1 and "/model/pode1/cmd_vel" in topics


def test_joint_states_cover_every_moving_joint():
    """빠진 관절은 TF가 없어 RViz 로봇 모델이 오류가 된다(캐스터 두 개가 빠졌었다)."""
    xacro = pytest.importorskip("xacro")
    need("pinky_description")
    root = ET.fromstring(xacro.process_file(str(URDF), mappings={"name": "pode1"}).toxml())
    moving = {j.get("name") for j in root.iter("joint") if j.get("type") != "fixed"}
    plugin = next(p for p in root.iter("plugin")
                  if p.get("name") == "gz::sim::systems::JointStatePublisher")
    listed = {e.text for e in plugin.iter("joint_name")}
    assert moving and (not listed or moving <= listed)


# ---- 가운데 통로 갈래: gz.launch.py의 nav_map_variant와 같은 뜻 ----
def nav_arguments(actions) -> dict:
    """nav2.launch.py include의 인자(map_server는 map, AMCL은 map_fine을 읽는다)."""
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    context = launch.LaunchContext()

    def text(subs):
        return perform_substitutions(context, normalize_to_list_of_substitutions(subs))
    found = [{text(k): text(v) for k, v in a.launch_arguments} for a in actions
             if type(a).__name__ == "IncludeLaunchDescription"]
    [nav] = [args for args in found if "map_fine" in args]  # nav2.launch.py만 map_fine을 받는다
    return nav


@pytest.mark.parametrize("variant", ["a", "b", "ab", "a1"])
def test_nav_map_variant_changes_only_the_costmap_map(one, variant):
    need("pinky_description", "pode_nav_config", "pode_fsm")
    nav = nav_arguments(run(one, layout="v7", start="O_IN", nav_map_variant=variant))
    assert nav["map"].endswith(f"maps/arena_v7_{variant}.yaml")
    assert Path(nav["map"]).is_file()
    assert nav["map_fine"].endswith("maps/arena_v7_fine.yaml")
    assert nav["keepout_mask"].endswith("maps/arena_v7_keepout.yaml")


def test_nav_map_variant_is_empty_by_default_and_v7_only(one):
    need("pinky_description", "pode_nav_config", "pode_fsm")
    from fleet_gz_sim.fleet_setup import ScenarioUnfit
    assert one.ARGS["nav_map_variant"][0] == ""
    # 비면 v7은 2.5 cm 지도, base는 5 cm 지도
    assert nav_arguments(run(one, layout="v7", start="O_IN"))["map"].endswith(
        "maps/arena_v7_a.yaml")
    assert nav_arguments(run(one, layout="v7", start="O_IN", nav_map_variant="base"))[
        "map"].endswith("maps/arena_v7.yaml")
    with pytest.raises(ScenarioUnfit, match="only for layout v7"):
        run(one, layout="d2", nav_map_variant="b")
    with pytest.raises(ScenarioUnfit, match="nav_map_variant"):
        run(one, layout="v7", start="O_IN", nav_map_variant="x")


def test_the_corridor_box_schedule_runs_with_one_pode_on_o_in(one):
    need("pinky_description", "pode_nav_config", "pode_fsm")
    actions = run(one, layout="v7", start="O_IN", obstacles="v7_corridor_box")
    assert [a for a in actions if type(a).__name__ == "Node"
            and a.node_executable == "sim_obstacles"]


# ---- 길 유지(pode_plan_keep): 한 대는 nav2.launch.py가 노드를 띄운다 ----
@pytest.mark.parametrize("text, keep", [(None, True), ("true", True), ("false", False)])
def test_the_plan_keep_switch_reaches_pode(one, monkeypatch, text, keep):
    need("pinky_description", "pode_nav_config", "pode_fsm")
    from fleet_gz_sim import launch_parts
    seen = {}
    monkeypatch.setattr(launch_parts, "pode", lambda *a, **kw: seen.update(kw) or [])
    run(one, **({} if text is None else {"plan_keep": text}))
    assert seen["plan_keep"] is keep
    assert seen.get("plan_keep_node", True) is True
    with pytest.raises(ValueError, match="plan_keep"):
        run(one, plan_keep="no")


def test_one_pode_passes_plan_keep_to_nav2(one):
    need("pinky_description", "pode_nav_config", "pode_fsm")
    nav = nav_arguments(run(one))
    assert (nav["plan_keep"], nav["plan_keep_node"]) == ("true", "true")
    nav = nav_arguments(run(one, plan_keep="false"))
    assert (nav["plan_keep"], nav["plan_keep_node"]) == ("false", "true")
