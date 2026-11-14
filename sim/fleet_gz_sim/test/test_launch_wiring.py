"""gz.launch.py: 격리 검사를 먼저 하고, 배치(layout)의 월드·지도·keepout으로 핑키 3대와 진짜 관제,
가짜 central·팔을 잇는 것을 본다."""
import importlib.util
import math
from pathlib import Path

import pytest
import yaml

launch = pytest.importorskip("launch")
rclpy = pytest.importorskip("rclpy")

LAUNCH = Path(__file__).parents[1] / "launch" / "gz.launch.py"
FAST_DDS = ("ROS_STATIC_PEERS", "RMW_IMPLEMENTATION", "ROS_DISCOVERY_SERVER",
            "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
            "FASTDDS_ENVIRONMENT_FILE", "FASTDDS_BUILTIN_TRANSPORTS")
ROBOTS = ["pinky1", "pinky2", "pinky3"]
STARTS = {"pinky1": (0.13, 0.15, math.pi / 2), "pinky2": (0.38, 0.15, math.pi / 2),
          "pinky3": (1.85, 0.15, math.pi / 2)}  # gz_d2: P-1, P-2, P-3


@pytest.fixture
def gz(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location("gz_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv("ROS_DOMAIN_ID", "44")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    for name in FAST_DDS:
        monkeypatch.delenv(name, raising=False)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(module, "get_rmw_implementation_identifier", lambda: "rmw_fastrtps_cpp")
    return module


def run(gz, **overrides):
    """ros2 launch처럼 인자 기본값을 넣고(overrides로 바꾸고) OpaqueFunction을 실행한다."""
    context = launch.LaunchContext()
    *args, start = gz.generate_launch_description().entities
    for action in args:
        action.visit(context)
    context.launch_configurations.update(overrides)
    return start.execute(context)


def test_the_rmw_comes_from_rclpy():
    spec = importlib.util.spec_from_file_location("gz_launch_plain", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.get_rmw_implementation_identifier is rclpy.get_rmw_implementation_identifier


def test_launch_reads_the_current_environment(gz, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "6")
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID"):
        run(gz)


def test_launch_checks_the_loaded_rmw(gz, monkeypatch):
    monkeypatch.setattr(gz, "get_rmw_implementation_identifier", lambda: "rmw_cyclonedds_cpp")
    with pytest.raises(RuntimeError, match="rmw_cyclonedds_cpp"):
        run(gz)


def test_launch_checks_the_working_directory(gz, tmp_path):
    (tmp_path / "DEFAULT_FASTRTPS_PROFILES.xml").write_text("<dds/>")
    with pytest.raises(RuntimeError, match="DEFAULT_FASTRTPS_PROFILES.xml"):
        run(gz)


def test_the_launch_takes_a_layout_and_no_site_argument(gz):
    assert gz.ARGS["layout"][0] == "d2"
    assert "site" not in gz.ARGS and "traffic_mode" not in gz.ARGS
    # 마당 goal의 충돌 검사는 끌 수 없다. 고정 경로 셋은 fixed_line_collision_check로 정한다
    assert "nav_collision_check" not in gz.ARGS


# -- 배선: launch 동작 대신 만든 인자를 기록해 본다 ------------------------------------------

KINDS = ("Node", "IncludeLaunchDescription", "PythonLaunchDescriptionSource", "ExecuteProcess",
         "GroupAction", "SetRemap", "RegisterEventHandler", "OnProcessExit")


class Made:
    def __init__(self, kind, args, kwargs):
        self.kind, self.args, self.kw = kind, args, kwargs

    def flat(self):
        yield self
        if self.kind == "GroupAction":
            for child in self.args[0]:
                yield from (child.flat() if isinstance(child, Made) else [child])


def need(*packages):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    for package in packages:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            pytest.skip(f"{package} is not installed")


@pytest.fixture
def made(gz, monkeypatch, tmp_path):
    from fleet_gz_sim import launch_parts
    need("pinky_description", "pinky_nav_config", "pinky_fsm", "fake_robots", "fleet_manager",
         "fleet_gz_sim")
    monkeypatch.setattr(launch_parts.tempfile, "gettempdir", lambda: str(tmp_path))
    for module in (gz, launch_parts):
        for kind in KINDS:
            if hasattr(module, kind):
                monkeypatch.setattr(module, kind, lambda *a, _k=kind, **kw: Made(_k, a, kw))
    return gz


def flat(actions):
    for a in actions:
        yield from (a.flat() if isinstance(a, Made) else [a])


def nodes(actions, executable=None):
    return [a for a in flat(actions) if isinstance(a, Made) and a.kind == "Node"
            and (executable is None or a.kw["executable"] == executable)]


def params(node) -> dict:
    out = {}
    for p in node.kw.get("parameters", []):
        out.update(p)
    return out


def includes(actions, launch_file):
    return [a for a in flat(actions) if isinstance(a, Made) and a.kind == "IncludeLaunchDescription"
            and str(a.args[0].args[0]).endswith(launch_file)]


def launch_args(include) -> dict:
    return dict(include.kw["launch_arguments"])


def after_wait(actions):
    [on_exit] = [a.args[0].kw["on_exit"] for a in flat(actions) if isinstance(a, Made)
                 and a.kind == "RegisterEventHandler" and callable(a.args[0].kw["on_exit"])]
    return on_exit(type("Exit", (), {"returncode": 0})(), None)


def scenario(name="gz_d2") -> str:
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory("fleet_gz_sim")) / "scenarios" / f"{name}.yaml")


def test_three_pinkys_start_on_the_parking_nodes_in_order(made):
    actions = run(made)
    spawned = {}
    for create in nodes(actions, "create"):
        a = create.kw["arguments"]
        spawned[a[a.index("-name") + 1]] = tuple(float(a[a.index(k) + 1])
                                                 for k in ("-x", "-y", "-Y"))
    assert spawned.keys() == STARTS.keys()
    for robot, pose in STARTS.items():
        assert spawned[robot] == pytest.approx(pose)
    navs = {launch_args(i)["namespace"]: launch_args(i)
            for i in includes(actions, "nav2.launch.py")}
    for robot, nav in navs.items():  # AMCL 시작 자세가 놓은 자리와 같다
        assert tuple(float(nav[k]) for k in ("initial_x", "initial_y", "initial_yaw")) == \
            pytest.approx(spawned[robot])


def test_the_test_layout_has_its_default_scenario_and_it_names_that_layout():
    path = Path(scenario("gz_d2_shelf"))
    assert path.is_file()
    assert yaml.safe_load(path.read_text(encoding="utf-8"))["map"] == "d2_shelf"


@pytest.mark.parametrize("layout", ["d2", "d", "c", "d2_shelf"])
def test_each_layout_runs_its_world_maps_and_keepout_mask(made, layout):
    actions = run(made, layout=layout)
    [server] = [a for a in flat(actions) if isinstance(a, Made) and a.kind == "ExecuteProcess"
                and "-s" in a.kw["cmd"]]
    assert server.kw["cmd"][-1].endswith(f"worlds/arena_{layout}.sdf")
    navs = [launch_args(i) for i in includes(actions, "nav2.launch.py")]
    assert len(navs) == 3
    for nav in navs:
        assert nav["map"].endswith(f"maps/arena_{layout}.yaml")
        assert nav["map_fine"].endswith(f"maps/arena_{layout}_fine.yaml")
        assert nav["keepout_mask"].endswith(f"maps/arena_{layout}_keepout.yaml")
        assert Path(nav["keepout_mask"]).is_file()


def test_keepout_can_be_left_off_for_a_comparison(made):
    navs = [launch_args(i) for i in includes(run(made, keepout="false"), "nav2.launch.py")]
    assert [nav["keepout_mask"] for nav in navs] == ["", "", ""]


def test_a_scenario_for_another_layout_starts_nothing(made):
    from fleet_gz_sim.fleet_setup import ScenarioUnfit
    with pytest.raises(ScenarioUnfit, match="layout"):
        run(made, layout="c", scenario="gz_d2")


def test_an_unknown_layout_starts_nothing(made):
    from fleet_gz_sim.fleet_setup import ScenarioUnfit
    with pytest.raises(ScenarioUnfit, match="layout"):
        run(made, layout="lane_arena")


def _controllers(path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return data["controller_server"]["ros__parameters"]


def test_every_controller_checks_collisions_when_asked_and_keeps_the_speed_cap(made):
    from fleet_gz_sim.launch_parts import CONTROLLERS
    for speed in ("", "0.15"):
        made_with = run(made, nav_max_speed=speed, fixed_line_collision_check="true")
        for nav in includes(made_with, "nav2.launch.py"):
            controllers = _controllers(launch_args(nav)["params_file"])
            for name in CONTROLLERS:
                assert controllers[name]["use_collision_detection"] is True, name
            forward = [controllers[n]["desired_linear_vel"] for n in CONTROLLERS
                       if n != "FollowPathReverse"]
            assert forward == [float(speed or 0.12)] * 3
            assert controllers["FollowPathReverse"]["desired_linear_vel"] == 0.05


@pytest.mark.parametrize("asked", [{}, {"fixed_line_collision_check": "false"}])
def test_the_fixed_line_check_is_off_for_every_robot_unless_asked(made, asked):
    # 사용자 결정 2026-11-13: 켠 세 대 경기가 끝나지 않아 기본을 끔으로 둔다
    navs = includes(run(made, **asked), "nav2.launch.py")
    assert len(navs) == 3
    for nav in navs:
        controllers = _controllers(launch_args(nav)["params_file"])
        assert [controllers[n]["use_collision_detection"] for n in
                ("FollowPath", "FollowPathThrough", "FollowPathReverse", "FollowPathGoal")] == [
            False, False, False, True]
    with pytest.raises(ValueError, match="fixed_line_collision_check"):
        run(made, fixed_line_collision_check="off")


def test_nav_overrides_lays_a_yaml_file_over_every_robots_nav2_params(made, tmp_path):
    """주행 성능 조정(2026-11-14): 파일을 고치지 않고 Nav2 값을 바꿔 Gazebo 판을 비교한다."""
    extra = tmp_path / "faster.yaml"
    extra.write_text(yaml.safe_dump({"controller_server": {"ros__parameters": {
        "FollowPathGoal": {"approach_velocity_scaling_dist": 0.2}}}}), encoding="utf-8")
    navs = includes(run(made, nav_overrides=str(extra)), "nav2.launch.py")
    assert len(navs) == 3
    for nav in navs:
        goal = _controllers(launch_args(nav)["params_file"])["FollowPathGoal"]
        assert goal["approach_velocity_scaling_dist"] == 0.2
        assert goal["use_collision_detection"] is True, "overrides never turn the check off"
    plain = includes(run(made), "nav2.launch.py")[0]
    assert _controllers(launch_args(plain)["params_file"])["FollowPathGoal"][
        "approach_velocity_scaling_dist"] == 0.6
    with pytest.raises(FileNotFoundError):
        run(made, nav_overrides=str(tmp_path / "missing.yaml"))


MPPI = "nav2_mppi_controller::MPPIController"
SWAP = {"controller_server": {"ros__parameters": {"FollowPathGoal": {"plugin": MPPI}}}}
DOTTED = {"controller_server": {"ros__parameters": {"FollowPathGoal.plugin": MPPI}}}
LISTED = {"controller_server": {"ros__parameters": {"FollowPath": [{"plugin": MPPI}]}}}
NEW_LISTED = {"controller_server": {"ros__parameters": {"Other": {"steps": [{"plugin": MPPI}]}}}}


@pytest.mark.parametrize("text", ["", "just words", "- 1\n- 2", "[]", yaml.safe_dump(SWAP),
                                  yaml.safe_dump(DOTTED), "controller_server: null",
                                  "controller_server: []",
                                  "controller_server:\n  ros__parameters: null",
                                  yaml.safe_dump(LISTED), yaml.safe_dump(NEW_LISTED),
                                  "controller_server:\n  ros__parameters:\n    FollowPath: 3",
                                  "amcl: 5"])
def test_nav_overrides_must_be_a_mapping_and_never_swaps_a_plugin(made, tmp_path, text):
    """Codex 2026-11-14: an empty file quietly meant nothing, a list crashed in the merge, and a
    plugin swap would slip past the RPP speed cap and collision check. Second review: a plugin
    inside a list was not looked at, and a value that replaced a whole mapping (a controller
    given as a list or a number) passed the check and crashed later in the launch."""
    extra = tmp_path / "bad.yaml"
    extra.write_text(text, encoding="utf-8")
    with pytest.raises(ValueError, match="nav_overrides"):
        run(made, nav_overrides=str(extra))


@pytest.mark.parametrize("bad", ["0.25", "0.1", "fast", "nan"])
def test_a_speed_outside_the_arena_range_stops_the_launch(made, bad):
    with pytest.raises(ValueError, match="nav_max_speed"):
        run(made, nav_max_speed=bad)


def test_the_speed_cap_also_caps_the_fsm_speed_limit_of_every_robot(made):
    caps = [launch_args(i)["horizon_cap_mps"]
            for i in includes(run(made, nav_max_speed="0.14"), "pinky_fsm.launch.py")]
    assert caps == ["0.14"] * 3


def test_gazebo_time_and_every_pinky_share_one_bridge(made):
    [bridge] = nodes(run(made), "parameter_bridge")
    topics = [str(x).split("@")[0] for x in bridge.kw["arguments"]]
    assert topics.count("/clock") == 1
    for robot in ROBOTS:
        for name in ("scan", "odom", "tf", "joint_states", "cmd_vel", "truth"):
            assert topics.count(f"/model/{robot}/{name}") == 1


def test_the_fleet_runs_the_layout_map_after_every_pinky_is_ready(made):
    actions = run(made, layout="d")
    [wait] = nodes(actions, "wait_ready")
    assert params(wait)["robots"] == ROBOTS
    assert not nodes(actions, "fleet_manager_node")  # 기다린 뒤에 띄운다
    fleet = after_wait(run(made, layout="d", scenario=scenario("gz_d")))
    [manager] = nodes(fleet, "fleet_manager_node")
    assert params(manager)["map_file"] == "d" and params(manager)["robots"] == ROBOTS
    for node in nodes(fleet):
        if "map_file" in params(node):
            assert params(node)["map_file"] == "d", node.kw["executable"]
    fakes = sorted(n.kw["executable"] for n in nodes(fleet) if n.kw["package"] == "fake_robots")
    assert fakes == sorted(["fake_workcell_node", "fake_vision_node", "fake_vision_node",
                            "fake_arm_node", "fake_central_node", "scenario_monitor_node"])


def test_the_fleet_params_must_run_the_zone_traffic(gz, tmp_path):
    from fleet_core.common.params import DEFAULT_PARAMS
    values = yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))
    assert values["traffic_mode"] == "zones"
    assert gz.params_for("", str(tmp_path)) == ""
    values["traffic_mode"] = "lanes"
    other = tmp_path / "lanes.yaml"
    other.write_text(yaml.safe_dump(values), encoding="utf-8")
    with pytest.raises((ValueError, TypeError)):
        gz.params_for(str(other), str(tmp_path / "out"))


def test_battery_low_pct_goes_into_a_params_copy(gz, tmp_path):
    copy = gz.params_for("", str(tmp_path / "out"), "35")
    assert yaml.safe_load(Path(copy).read_text(encoding="utf-8"))["battery_low_pct"] == 35.0
    with pytest.raises(ValueError, match="battery_low_pct"):
        gz.params_for("", str(tmp_path / "out"), "120")


def test_the_real_pinky_fsm_answers_dock_by_default(made):
    actions = run(made)
    assert not [a for a in flat(actions) if isinstance(a, Made) and a.kind == "SetRemap"]
    [stand_in] = nodes(actions, "sim_stand_in")
    assert params(stand_in)["dock_stub"] is False and params(stand_in)["map_file"] == "d2"


def test_sim_dock_stub_moves_the_pinky_fsm_dock_aside(made):
    actions = run(made, sim_dock_stub="true")
    remaps = [a.args for a in flat(actions) if isinstance(a, Made) and a.kind == "SetRemap"]
    assert sorted(remaps) == [(f"/{r}/dock", f"/{r}/dock_fsm") for r in ROBOTS]
    [stand_in] = nodes(actions, "sim_stand_in")
    assert params(stand_in)["dock_stub"] is True


@pytest.mark.parametrize("name", ["sim_dock_stub", "fleet_check", "gui", "monitor",
                                  "nav_composition", "keepout"])
def test_a_misspelt_switch_stops_the_launch(made, name):
    with pytest.raises(ValueError, match=name):
        run(made, **{name: "ture"})


def test_obstacles_take_a_link_schedule_checked_against_the_layout(made):
    fleet = after_wait(run(made, obstacles="d2_yard_express"))
    [node] = nodes(fleet, "sim_obstacles")
    assert params(node)["obstacles_file"].endswith("obstacles/d2_yard_express.yaml")
    assert params(node)["map_file"] == "d2"
    from fleet_gz_sim.obstacles import ScheduleError
    with pytest.raises(ScheduleError, match="no_such_schedule"):
        run(made, obstacles="no_such_schedule")


@pytest.mark.parametrize("text, value", [("true", True), ("false", False), ("", None)])
def test_use_order_model_reaches_the_fleet_manager_only_when_given(made, text, value):
    [manager] = nodes(after_wait(run(made, use_order_model=text)), "fleet_manager_node")
    assert params(manager).get("use_order_model") == value


def test_a_use_order_model_typo_stops_the_launch(made):
    with pytest.raises(ValueError, match="use_order_model"):
        run(made, use_order_model="yes")


def test_one_rviz_shows_the_whole_fleet_by_default(made):
    actions = run(made)
    [view] = nodes(actions, "fleet_view")
    assert params(view)["robots"] == ROBOTS and params(view)["map_file"] == "d2"
    closed = run(made, rviz="none")
    assert not nodes(closed, "fleet_view") and not nodes(closed, "rviz2")


NAV2_NODES = 12  # pinky_nav_config with keepout: 노드 열, lifecycle manager 셋을 넉넉히
LOCALHOST_PARTICIPANTS = 32  # Fast DDS LOCALHOST 탐색에서 서로 보이는 참여자 수(실측)
SPARE = 6  # launch 프로세스, RViz, 사람이 띄우는 ros2 명령 몇 개


def test_ros_process_budget_leaves_room_under_the_localhost_limit(made):
    actions = run(made, obstacles="d2_yard_express")
    fleet = after_wait(actions)
    count = len(nodes(actions)) + len(nodes(fleet))
    for nav in includes(actions, "nav2.launch.py"):
        count += 1 if launch_args(nav)["use_composition"] == "true" else NAV2_NODES
    count += len(includes(actions, "pinky_fsm.launch.py"))
    assert count + SPARE <= LOCALHOST_PARTICIPANTS


def test_the_fleet_record_is_kept_in_the_result_folder_only_when_asked(made, tmp_path):
    """tools/triage and tools/avoid_regression.py read the record of a Gazebo game."""
    [manager] = nodes(after_wait(run(made)), "fleet_manager_node")
    assert not manager.kw.get("arguments")
    out = tmp_path / "result"
    [manager] = nodes(after_wait(run(made, fleet_record="true", result_dir=str(out))),
                      "fleet_manager_node")
    assert manager.kw["arguments"] == ["--record", str(out / "fleet_record.jsonl")]
    with pytest.raises(ValueError, match="fleet_record"):
        run(made, fleet_record="yes")


def test_the_bodies_of_other_robots_stay_out_of_the_costmaps_when_switched_off(made):
    actions = run(made, peer_obstacles="false")
    [stand_in] = nodes(actions, "sim_stand_in")
    assert params(stand_in)["peer_obstacles"] is False
    for nav in includes(actions, "nav2.launch.py"):
        assert launch_args(nav)["peer_obstacles"] == "false"


@pytest.mark.parametrize("asked", [{}, {"peer_obstacles": "true"}])
def test_peer_obstacles_adds_the_layer_to_every_robot_and_the_stand_in_sends_the_bodies(
        made, asked):
    # 기본이 켜짐이다(사용자 결정 2026-11-13, 작업 기록 11절)
    before = len(nodes(run(made, peer_obstacles="false")))
    actions = run(made, **asked)
    navs = includes(actions, "nav2.launch.py")
    assert len(navs) == 3
    for nav in navs:
        assert launch_args(nav)["peer_obstacles"] == "true"
        assert launch_args(nav)["peer_node"] == "false", "no extra process per robot"
    [stand_in] = nodes(actions, "sim_stand_in")
    assert params(stand_in)["peer_obstacles"] is True
    assert params(stand_in)["robots"] == ["pinky1", "pinky2", "pinky3"]
    assert len(nodes(actions)) == before, "no new process: the DDS participant budget is full"
    with pytest.raises(ValueError, match="peer_obstacles"):
        run(made, peer_obstacles="on")


# ---- 회피 패키지와 임시 비전 대역 ------------------------------------------------------------

def test_every_robot_gets_the_avoidance_package_and_no_vision_stand_in_by_default(made):
    assert made.ARGS["avoidance_package"][0] == "pinky_avoidance"
    assert made.ARGS["vision_stand_in"][0] == "false"
    navs = includes(run(made), "nav2.launch.py")
    assert len(navs) == 3
    for nav in navs:
        assert launch_args(nav)["avoidance_package"] == "pinky_avoidance"
        assert launch_args(nav)["vision_stand_in"] == "false"


def test_the_vision_stand_in_can_be_turned_on_for_every_robot(made):
    navs = includes(run(made, vision_stand_in="true"), "nav2.launch.py")
    assert [launch_args(nav)["vision_stand_in"] for nav in navs] == ["true"] * 3
    with pytest.raises(ValueError, match="vision_stand_in"):
        run(made, vision_stand_in="on")


def test_an_unknown_avoidance_package_stops_the_launch(made):
    from ament_index_python.packages import PackageNotFoundError
    with pytest.raises(PackageNotFoundError):
        run(made, avoidance_package="no_such_avoidance_package")
