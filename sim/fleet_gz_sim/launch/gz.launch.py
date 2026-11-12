"""Gazebo 핑키 3대 + 진짜 관제(관제·pinky_fsm·Nav2는 진짜, central·팔은 가짜).

    ros2 launch fleet_gz_sim gz.launch.py                          # 배치 D2, 시나리오 gz_d2
    ros2 launch fleet_gz_sim gz.launch.py layout:=c gui:=false rviz:=none

- 배치(layout: d2 기본, d, c)가 월드, Nav2 지도, keepout 마스크, 관제 구역 지도를 고른다
  (fleet_setup.arena_for). 시나리오는 비면 gz_<배치>이고, 시나리오의 map이 배치와 같아야 한다.
- 핑키는 시나리오의 start 지점(없으면 주차 지점 차례)에 그 지점의 yaw로 놓는다(fleet_setup).
- 로봇마다 Nav2(pinky_nav_config)에 배치의 지도와 keepout_mask를 준다. sim_nav_params가
  controller 넷(FollowPath, FollowPathThrough, FollowPathReverse, FollowPathGoal)의 충돌 검사를
  모두 켜고 속도를 nav_max_speed(0.12~0.20 m/s)로 묶는다.
- 모든 로봇이 IDLE이 되면(wait_ready) 관제(map_file = 배치 이름, traffic_mode zones), 가짜
  작업셀·비전·출고 팔·central, 시나리오 모니터를 띄운다. 모니터가 결과를 쓰고 launch를 끝낸다.
- 시계는 Gazebo /clock 하나를 모두 쓴다. 주문 시각(at_s)은 Gazebo 시계 0부터 센다.
- 도크: 기본은 진짜 pinky_fsm(DOCK_UNAVAILABLE). sim_dock_stub:=true면 시뮬 전용 대역
  (sim_stand_in)이 /{pinky}/dock을 맡고 pinky_fsm의 dock 서버는 /{pinky}/dock_fsm으로 비켜 둔다.
- fleet_check(기본 켜짐)가 로봇 사이 최소 거리, 정지 횟수, 쥔 구역 밖 위치 수를 쓴다.
- ROS 프로세스 수: LOCALHOST 탐색에서는 DDS 참여자 32개 정도만 서로 보인다. 그래서 브리지는
  하나로 모으고, 로봇마다 Nav2를 프로세스 하나로 묶는다(nav_composition).
"""
import math
import os
from pathlib import Path

from fleet_gz_sim import launch_parts
from fleet_gz_sim.isolation import require_isolated
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy import get_rmw_implementation_identifier

ARGS = {
    "layout": ("d2", "배치: d2(기본), d, c, 시험용 d2_shelf"),
    "scenario": ("", "시나리오 이름(fleet_gz_sim, fake_robots 순서로 찾음)이나 파일."
                     " 비면 gz_<배치>(주문 8건, 3대)"),
    "gui": ("true", "Gazebo 화면"),
    "rviz": ("fleet", "fleet이면 3대를 한 RViz에(fleet_view), 로봇 이름이면 그 로봇의 Nav2 화면,"
                      " none이면 띄우지 않는다"),
    "heartbeat_timeout_s": ("1.5", "pinky_fsm heartbeat 끊김 판정"),
    "nav_log_level": ("warn", "Nav2 노드 로그 수준"),
    "nav_composition": ("true", "로봇마다 Nav2 노드를 프로세스 하나로. false면 노드마다"
                                "(3대면 DDS 참여자가 LOCALHOST 탐색 한도 32개를 넘는다)"),
    "result_dir": ("gz_result", "시나리오 모니터 결과 폴더"),
    "fleet_params_file": ("", "관제 파라미터 파일. 비우면 설치된 fleet_params.yaml, 경로 없는"
                              " 이름이면 fleet_core/data/<이름>.yaml"),
    "monitor": ("true", "시나리오 판정. 끝나면 launch를 끝낸다"),
    "obstacles": ("", "상자를 놓았다 치우는 일정(fleet_gz_sim/obstacles의 이름이나 파일)."
                      " 비면 장애물 없음. d2_yard_express는 마당 하나, 급행 하나"),
    "sim_dock_stub": ("false", "true면 시뮬 전용 도킹 대역: 도크 지점 근처면 몇 초 뒤 성공."
                               " false면 진짜 pinky_fsm(DOCK_UNAVAILABLE)"),
    "fleet_check": ("true", "로봇 사이 최소 거리, 정지 횟수, 쥔 구역 밖 위치 수를"
                            " result_dir/fleet_check.json에"),
    "nav_max_speed": ("0.12", "controller들의 최고 속도(m/s), 0.12~0.20"),
    "keepout": ("true", "false면 Nav2 global costmap의 keepout 필터를 띄우지 않는다(비교용)"),
    "nav_pass_through": ("true", "잠금 경계에서 끝나는 이동(final_goal 거짓)을 접근 감속 없는 둘째"
                                " RPP(FollowPathThrough)로. false면 모두 FollowPath(둘 다 뜬다)"),
    "scenario_seed": ("", "시나리오 random 블록을 풀 seed(정수). 비면 파일의 seed"),
    "battery_low_pct": ("", "관제 battery_low_pct(0~100). 주면 그 값을 넣은 파라미터 사본"
                            "(result_dir/fleet_params.yaml)으로 돈다. 비우면 파일 그대로"),
    "fixed_line_collision_check": ("false", "시뮬 전용. true면 고정 경로 controller 셋(FollowPath,"
                                           " Through, Reverse)도 충돌 검사를 켠다. 마당은 늘 켬"),
    "scan_filter": ("true", "좁은 띠 안에서 벽·진열대에 붙은 라이다 점을 costmap에서 지우는 거르개"
                            "(pinky_scan_filter, 추가지시서3 7-2). false면 띄우지 않는다(비교용)"),
    "wall_band_m": ("0.05", "스캔 거르개: 벽·진열대에서 이만큼 안의 점을 지운다(m)."
                            " 0.05는 D2 측정으로 고른 값(fixed_line_results.md)"),
    "fleet_record": ("false", "true면 관제 기록(fleet_manager --record)을"
                              " result_dir/fleet_record.jsonl에 남긴다. tools/triage,"
                              " tools/avoid_regression.py, tools/replay.py가 읽는다"),
    "use_order_model": ("","true/false: 관제의 use_order_model 파라미터(order 모델이 있는"
                            " 관제만 읽는다). 비우면 params 파일의 값"),
}
ARMS = "arm1 arm2 arm3"


def flag(text: str, name: str) -> bool:
    """A true/false launch argument. A typo must not quietly mean false."""
    value = text.strip().lower()
    if value not in ("true", "false", "1", "0"):
        raise ValueError(f"{name} must be true or false, got {text!r}")
    return value in ("true", "1")


def params_file_of(value: str) -> str:
    """"" stays "", a path stays as given, a bare name is fleet_core/data/<name>.yaml."""
    from fleet_core.common.params import DEFAULT_PARAMS
    if not value or "/" in value or value.endswith(".yaml"):
        return value
    path = DEFAULT_PARAMS.parent / f"{value}.yaml"
    if not path.is_file():
        raise ValueError(f"fleet_params_file {value!r}: no {path}")
    return str(path)


def seed_of(text: str) -> int | None:
    """scenario_seed: "" means the file's seed. A typo must not quietly mean the file's seed."""
    if not text.strip():
        return None
    try:
        seed = int(text)
    except ValueError:
        raise ValueError(f"scenario_seed must be an integer, got {text!r}") from None
    if seed < 0:  # 노드에서는 음수가 '파일의 seed'라는 뜻이다
        raise ValueError(f"scenario_seed must be 0 or more, got {text!r}")
    return seed


def battery_low_of(text: str) -> float | None:
    """battery_low_pct: "" leaves the params file alone."""
    if not text.strip():
        return None
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"battery_low_pct must be a number from 0 to 100, got {text!r}") from None
    if not (math.isfinite(value) and 0.0 <= value <= 100.0):
        raise ValueError(f"battery_low_pct must be a number from 0 to 100, got {text!r}")
    return value


def params_for(params_file: str, result_dir: str, battery_low_pct: str = "") -> str:
    """The params file every fleet node reads. The fleet runs traffic_mode zones: a file with
    another mode stops the launch. With battery_low_pct, a copy with that value."""
    import yaml
    from fleet_core.common.params import DEFAULT_PARAMS, load_params
    from fleet_core.traffic.modes import ZONES
    source = Path(params_file) if params_file else DEFAULT_PARAMS
    mode = load_params(source).traffic_mode
    if mode != ZONES:
        raise ValueError(f"{source} has traffic_mode {mode!r}; the Gazebo fleet runs {ZONES!r}")
    battery = battery_low_of(battery_low_pct)
    if battery is None:
        return params_file
    values = yaml.safe_load(source.read_text(encoding="utf-8"))
    copy = Path(result_dir) / "fleet_params.yaml"
    if copy.resolve() == source.resolve() or (copy.exists() and copy.samefile(source)):
        raise ValueError(f"fleet_params_file {source} is where the params copy goes;"
                         " give another result_dir")
    values["battery_low_pct"] = battery  # 키 검사는 fleet_core가 한다
    copy.parent.mkdir(parents=True, exist_ok=True)
    copy.write_text(yaml.safe_dump(values, sort_keys=False, allow_unicode=True),
                    encoding="utf-8")
    return str(copy)


def fleet(scenario_file: str, robots: list[str], result_dir: str, monitor: bool,
          layout: str, params_file: str = "", obstacles_file: str = "", check: bool = True,
          scenario_seed: int | None = None, use_order_model: bool | None = None,
          record: str = "") -> list:
    """진짜 관제와 가짜 작업셀·비전·출고 팔·central·모니터, 장애물(있으면). 모두 Gazebo 시계.
    record: 관제 기록 파일(fleet_manager --record). 비면 남기지 않는다.
    layout: 관제와 가짜들의 map_file(배치 이름).
    scenario_seed: 시나리오를 읽는 노드에 넘길 seed. None이면 넘기지 않는다(파일의 seed).
    use_order_model: 관제 노드 파라미터 use_order_model. None이면 넘기지 않는다(파일의 값)."""
    from fake_robots.common import split_fake_arms
    from fleet_core.fleet_map import load_fleet_map
    map_file = {"map_file": layout}
    seed = {} if scenario_seed is None else {"scenario_seed": scenario_seed}
    common = {"use_sim_time": True, "scenario_file": scenario_file, **map_file, **seed}

    def fake(executable, name, **extra):
        return Node(package="fake_robots", executable=executable, name=name, output="screen",
                    parameters=[{**common, **extra}])

    actions = [Node(package="fleet_manager", executable="fleet_manager_node", name="fleet_manager",
                    output="screen", arguments=["--record", record] if record else [],
                    parameters=[{"use_sim_time": True, "robots": robots,
                                 "params_file": params_file, **map_file,
                                 **({} if use_order_model is None
                                    else {"use_order_model": use_order_model})}])]
    workcell, vision_arms, swap_arms = split_fake_arms(ARMS, load_fleet_map(layout))
    if workcell:
        actions.append(fake("fake_workcell_node", "fake_workcell"))
    actions += [fake("fake_vision_node", f"fake_vision_{arm}", arm_id=arm) for arm in vision_arms]
    actions += [fake("fake_arm_node", f"fake_{arm}", arm_id=arm) for arm in swap_arms]
    actions.append(fake("fake_central_node", "fake_central", params_file=params_file))
    if obstacles_file:
        # 로봇 실제 위치(sim/truth)를 보고 비어 있을 때만 놓는다. 일정 시각은 Gazebo 시계
        actions.append(Node(package="fleet_gz_sim", executable="sim_obstacles",
                            name="sim_obstacles", output="screen",
                            parameters=[{"use_sim_time": True, "robots": robots,
                                         "obstacles_file": obstacles_file, **map_file}]))
    probe = {"use_sim_time": True, "robots": robots, "result_dir": result_dir, "settle_s": 2.0}
    if check:
        # recovery_probe는 같은 프로세스에서 돈다(DDS 참여자를 아낀다)
        actions.append(Node(package="fleet_gz_sim", executable="fleet_check", name="fleet_check",
                            output="screen",
                            parameters=[{**probe, "recovery_probe": True,
                                         "output": str(Path(result_dir) / "fleet_check.json"),
                                         **map_file}]))
    else:
        actions.append(Node(package="fleet_gz_sim", executable="recovery_probe",
                            name="recovery_probe", output="screen", parameters=[probe]))
    if monitor:
        node = Node(package="fake_robots", executable="scenario_monitor_node",
                    name="scenario_monitor", output="screen",
                    parameters=[{**common, "result_dir": result_dir, "params_file": params_file}],
                    on_exit=Shutdown())
        actions.append(node)
    return actions


def fleet_rviz(robots: list[str], layout: str) -> list:
    """3대를 한 RViz에. 로봇 TF는 namespace로 나뉘므로 위치는 fleet_view가 마커로 그린다."""
    config = launch_parts.share("fleet_gz_sim") / "rviz" / "fleet_view.rviz"
    return [Node(package="fleet_gz_sim", executable="fleet_view", name="fleet_view",
                 output="screen",
                 parameters=[{"use_sim_time": True, "robots": robots, "map_file": layout}]),
            Node(package="rviz2", executable="rviz2", name="fleet_rviz", output="log",
                 arguments=["-d", str(config)], parameters=[{"use_sim_time": True}])]


def _start(context):
    # 노드는 launch의 작업 폴더를 물려받는다. 노드에 cwd를 따로 주면 그 폴더도 봐야 한다.
    require_isolated(os.environ, rmw=get_rmw_implementation_identifier(), workdir=Path.cwd())
    from fleet_core.fleet_map import load_fleet_map
    from fleet_gz_sim.fleet_setup import (
        arena_for,
        check_fits,
        has_low_battery,
        layout_of,
        narrow_strips,
        obstacles_path,
        read_scenario,
        scenario_path,
        start_nodes,
        start_poses,
    )
    from fleet_gz_sim.obstacles import load_schedule

    def value(name):
        return LaunchConfiguration(name).perform(context)

    layout = layout_of(value("layout"))
    gz_share, fake_share = launch_parts.share("fleet_gz_sim"), launch_parts.share("fake_robots")
    scenario_file = str(scenario_path(value("scenario") or f"gz_{layout}", gz_share, fake_share))
    arena = arena_for(layout, gz_share)
    site = load_fleet_map(layout)
    scenario_seed = seed_of(value("scenario_seed"))
    data = read_scenario(scenario_file)
    check_fits(data, Path(scenario_file).name, layout, scenario_seed)
    obstacles_file = obstacles_path(value("obstacles"), gz_share)
    if obstacles_file:
        load_schedule(obstacles_file, site)  # 틀린 일정이면 Gazebo를 띄우기 전에 멈춘다
    poses = start_poses(site, start_nodes(site, data))
    robots = list(poses)
    composed = flag(value("nav_composition"), "nav_composition")
    stub = flag(value("sim_dock_stub"), "sim_dock_stub")
    check = flag(value("fleet_check"), "fleet_check")
    through = flag(value("nav_pass_through"), "nav_pass_through")
    keepout = flag(value("keepout"), "keepout")
    fixed_check = flag(value("fixed_line_collision_check"), "fixed_line_collision_check")
    strips = (narrow_strips(site, float(value("wall_band_m")))
              if flag(value("scan_filter"), "scan_filter") else "")
    params_file = params_for(params_file_of(value("fleet_params_file")), value("result_dir"),
                             value("battery_low_pct"))
    max_speed = launch_parts.speed_of(value("nav_max_speed"))
    # low_battery가 있을 때만 pinky_fsm이 sim_stand_in의 배터리를 본다
    battery = has_low_battery(data, scenario_seed)
    actions = [*launch_parts.world(flag(value("gui"), "gui"), arena.world),
               launch_parts.bridge(robots)]
    for robot, (x, y, yaw) in poses.items():
        actions += launch_parts.pinky(robot, x, y, yaw,
                                      nav_map=arena.nav_map, nav_map_fine=arena.nav_map_fine,
                                      keepout_mask=arena.keepout_mask if keepout else "",
                                      heartbeat_timeout_s=value("heartbeat_timeout_s"),
                                      nav_log_level=value("nav_log_level"),
                                      rviz=robot == value("rviz"), dock_aside=stub,
                                      nav_composition=composed,
                                      through_controller_id=(launch_parts.THROUGH_CONTROLLER
                                                             if through else ""),
                                      nav_max_speed=max_speed, fixed_line_check=fixed_check,
                                      narrow_strips=strips,
                                      wall_band_m=float(value("wall_band_m")),
                                      battery_topic=(launch_parts.BATTERY_TOPIC.format(robot=robot)
                                                     if battery else ""))
    if value("rviz") == "fleet":
        actions += fleet_rviz(robots, layout)
    actions.append(Node(package="fleet_gz_sim", executable="sim_stand_in", name="sim_stand_in",
                        output="screen",
                        parameters=[{"use_sim_time": True, "robots": robots,
                                     "scenario_file": scenario_file, "dock_stub": stub,
                                     "map_file": layout,
                                     **({} if scenario_seed is None
                                        else {"scenario_seed": scenario_seed})}]))
    wait = Node(package="fleet_gz_sim", executable="wait_ready", name="wait_ready",
                output="screen", parameters=[{"robots": robots}])
    monitor = flag(value("monitor"), "monitor")
    record = (str(Path(value("result_dir")) / "fleet_record.jsonl")
              if flag(value("fleet_record"), "fleet_record") else "")
    order = (None if not value("use_order_model").strip()
             else flag(value("use_order_model"), "use_order_model"))

    def after_wait(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg="fleet_gz_sim: 로봇이 IDLE이 되지 않아 끝낸다"), Shutdown()]
        return fleet(scenario_file, robots, value("result_dir"), monitor, layout,
                     params_file, obstacles_file, check, scenario_seed, order, record)

    actions += [wait, RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=after_wait))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGS.items()),
        OpaqueFunction(function=_start),
    ])
