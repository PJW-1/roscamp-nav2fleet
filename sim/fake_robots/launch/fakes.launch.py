"""가짜 로봇, 작업셀, 팔, 중앙 대역, 시뮬 시계, 시나리오 모니터를 띄운다.

  ros2 launch fake_robots fakes.launch.py scenario:=basic_orders result_dir:=/tmp/run1
  ros2 launch fake_robots fakes.launch.py fake_robots:="pode2 pode3"   # pode1은 실물
  ros2 launch fake_robots fakes.launch.py fake_arms:="arm_ship"          # 작업셀은 실물
  ros2 launch fake_robots fakes.launch.py fake_robots:=none use_sim_clock:=false monitor:=false \
      scenario:=real_two_orders dock_wait_robots:="pode1 pode2"      # 실물 두 대, PC 도킹 대역
  ros2 launch fake_robots fakes.launch.py fake_robots:=none use_sim_clock:=false monitor:=false \
      scenario:=real_three_orders dock_wait_robots:="pode1 pode2 pode3"   # 실물 세 대, PC 도킹 대역

fake_arms: arm_left와 arm_right는 함께 줘야 가짜 작업셀이 뜨고(fill_basket 서버 하나가 둘을 맡는다),
arm_ship은 가짜 출고 팔이다. none은 가짜 팔 없음이다(대소문자 무시).
fake_robots:=none은 가짜 로봇 없음이다(ros2 launch는 빈 값을 받지 않는다).
dock_wait_robots: 실물 로봇의 도킹을 PC가 대신 받는다. 로봇은 DOCK_STUB=1로 떠야 하고, 시뮬 시계나 가짜 로봇과는 같이 못 쓴다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from fake_robots.common import split_fake_arms
from fake_robots.dock_wait import dock_wait_problems
from fake_robots.scenario import scenario_map_file
from fleet_core.traffic.site import load_site
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ALL_ROBOTS = "pode1 pode2 pode3"
ALL_ARMS = "arm_left arm_right arm_ship"
CLOCK_START_DELAY_S = 5.0  # 모든 노드가 뜨고 서로를 찾는 데 쓰는 실제 시간
CLOCK_RATE_HZ = 50.0  # 틱마다 모든 노드가 깬다. speed factor 5에서 시뮬 시간 0.1초


def _truthy(text: str) -> bool:
    return text.strip().lower() in ("1", "true", "yes", "on")


def _robot_list(text: str) -> list[str]:
    """빈칸으로 나눈 로봇 이름. none은 없음이다."""
    return [] if text.strip().lower() == "none" else text.split()


def _scenario_path(value: str) -> str:
    if os.path.isfile(value):
        return os.path.abspath(value)
    share = get_package_share_directory("fake_robots")
    path = os.path.join(share, "scenarios", value if value.endswith(".yaml") else value + ".yaml")
    if not os.path.isfile(path):
        raise RuntimeError(f"scenario {value!r} is neither a file nor in {share}/scenarios")
    return path


def _setup(context):
    arg = {name: LaunchConfiguration(name).perform(context) for name in (
        "scenario", "fake_robots", "fake_arms", "fake_central", "use_sim_clock", "result_dir",
        "monitor", "map_file", "contract_file", "params_file")}
    arg["dock_wait_robots"] = LaunchConfiguration("dock_wait_robots", default="").perform(context)
    scenario = _scenario_path(arg["scenario"])
    if not arg["map_file"]:
        arg["map_file"] = scenario_map_file(scenario)  # 시나리오가 고른 지도, 없으면 기본
    sim_clock = _truthy(arg["use_sim_clock"])
    fakes = _robot_list(arg["fake_robots"])
    dock_robots = arg["dock_wait_robots"].split()
    if dock_robots:  # 노드를 하나도 만들기 전에 막는다
        found = dock_wait_problems(dock_robots, fakes, sim_clock=sim_clock)
        if found:
            raise RuntimeError("dock_wait_robots refused:\n  " + "\n  ".join(found))
    common = {"use_sim_time": sim_clock, "scenario_file": scenario}
    if arg["map_file"]:
        common["map_file"] = arg["map_file"]
    if arg["contract_file"]:
        common["contract_file"] = arg["contract_file"]

    def node(executable, name, **extra):
        return Node(package="fake_robots", executable=executable, name=name, output="screen",
                    parameters=[{**common, **extra}])

    actions = []
    if sim_clock:
        import yaml
        with open(scenario, encoding="utf-8") as f:
            factor = float(yaml.safe_load(f).get("speed_factor", 1.0))
        actions.append(Node(package="fake_robots", executable="sim_clock_node",
                            name="sim_clock", output="screen",
                            parameters=[{"speed_factor": factor,
                                         "start_delay_s": CLOCK_START_DELAY_S,
                                         "rate_hz": CLOCK_RATE_HZ}]))
    for robot in fakes:
        actions.append(node("fake_pode_node", f"fake_{robot}", robot_id=robot,
                            params_file=arg["params_file"]))
    workcell, _, swap_arms = split_fake_arms(
        arg["fake_arms"], load_site(arg["map_file"] or None))
    if workcell:
        actions.append(node("fake_workcell_node", "fake_workcell"))
    for arm in swap_arms:
        actions.append(node("fake_arm_node", f"fake_{arm}", arm_id=arm))
    if _truthy(arg["fake_central"]):
        actions.append(node("fake_central_node", "fake_central", params_file=arg["params_file"]))
    if dock_robots:
        actions.append(node("real_dock_wait_node", "real_dock_wait", robots=" ".join(dock_robots)))
    if _truthy(arg["monitor"]):
        monitor = Node(package="fake_robots", executable="scenario_monitor_node",
                       name="scenario_monitor", output="screen",
                       parameters=[{**common, "result_dir": arg["result_dir"],
                                    "params_file": arg["params_file"]}],
                       on_exit=Shutdown())
        actions.append(monitor)
    return actions


def generate_launch_description():
    def declare(name, default, doc):
        return DeclareLaunchArgument(name, default_value=default, description=doc)

    return LaunchDescription([
        declare("scenario", "basic_orders", "scenario name or path to a scenario YAML"),
        declare("fake_robots", ALL_ROBOTS, "space separated robots to fake (others are real)"),
        declare("fake_arms", ALL_ARMS, "arm_left and arm_right together (workcell), arm_ship"),
        declare("fake_central", "true", "run the central stand-in"),
        declare("use_sim_clock", "true", "run the sim clock and use sim time everywhere"),
        declare("result_dir", "sim_result", "where the monitor writes summary.json and CSVs"),
        declare("monitor", "true", "run the scenario monitor, which shuts the launch down"),
        declare("map_file", "", "fleet map YAML (default: the one in fleet_core)"),
        declare("contract_file", "", "contract YAML (default: the installed one)"),
        declare("params_file", "", "fleet parameters YAML, for basket capacity and release limits"),
        declare("dock_wait_robots", "",
                "real robots whose dock the PC answers (they run DOCK_STUB=1)"),
        OpaqueFunction(function=_setup),
    ])
