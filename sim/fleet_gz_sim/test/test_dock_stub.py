"""sim_dock_stub: 기본 꺼짐, 격리된 시뮬의 Gazebo 시각에서만 켜진다. 실물 bringup에서는 안 닿는다.
"""
from pathlib import Path

import pytest
from fleet_gz_sim.dock_stand_in import dock_stub_problems, require_dock_stub_allowed

REPO = Path(__file__).resolve().parents[3]
ISOLATED = {"ROS_DOMAIN_ID": "97", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"}
ROBOT_NETWORK = {"ROS_DOMAIN_ID": "6", "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET"}


def test_allowed_only_on_sim_time_in_an_isolated_sim(tmp_path):
    assert dock_stub_problems(True, ISOLATED, workdir=tmp_path) == []
    assert any("use_sim_time" in p for p in dock_stub_problems(False, ISOLATED))


def test_the_robot_network_blocks_the_stub_even_on_sim_time():
    found = dock_stub_problems(True, ROBOT_NETWORK)
    assert any("ROS_DOMAIN_ID" in p for p in found)
    assert any("LOCALHOST" in p for p in found)
    with pytest.raises(RuntimeError, match="sim_dock_stub"):
        require_dock_stub_allowed(True, ROBOT_NETWORK)


def test_a_wall_clock_run_blocks_the_stub_even_when_isolated():
    with pytest.raises(RuntimeError, match="use_sim_time"):
        require_dock_stub_allowed(False, ISOLATED)


def test_the_loaded_rmw_and_fast_dds_files_count(tmp_path):
    with pytest.raises(RuntimeError, match="rmw_cyclonedds_cpp"):
        require_dock_stub_allowed(True, ISOLATED, rmw="rmw_cyclonedds_cpp")
    (tmp_path / "DEFAULT_FASTRTPS_PROFILES.xml").write_text("<dds/>", encoding="utf-8")
    with pytest.raises(RuntimeError, match="DEFAULT_FASTRTPS_PROFILES.xml"):
        require_dock_stub_allowed(True, ISOLATED, workdir=tmp_path)


# ---- 실물 bringup은 대역에 닿지 않는다 ----

# 따옴표 친 패키지 이름은 패키지 사용이고, 설명 글은 언급해도 된다
STUB_WORDS = ("sim_dock_stub", "dock_stub", "sim_stand_in", "\"fleet_gz_sim\"", "'fleet_gz_sim'")


def real_launch_files() -> list[Path]:
    """실물 실행의 시작점인 launch 파일: PC bringup, 포함하는 가짜들, pode."""
    found = sorted((REPO / "bringup").rglob("*.launch.py"))
    found += sorted((REPO / "sim" / "fake_robots" / "launch").glob("*.launch.py"))
    found += sorted((REPO / "pode").glob("*/launch/*.launch.py"))
    return found


def test_no_real_launch_file_names_the_stub():
    files = real_launch_files()
    assert any(f.name == "pc.launch.py" for f in files)
    assert any(f.name == "pode_fsm.launch.py" for f in files)
    for path in files:
        text = path.read_text(encoding="utf-8")
        for word in STUB_WORDS:
            assert word not in text, f"{path.relative_to(REPO)} mentions {word}"


def test_the_bringup_package_does_not_depend_on_the_sim():
    text = (REPO / "bringup" / "iponic_bringup" / "package.xml").read_text(encoding="utf-8")
    assert "fleet_gz_sim" not in text


def test_passing_sim_dock_stub_to_the_pc_bringup_starts_nothing_from_the_sim():
    """ros2 launch는 선언 안 한 인자를 조용히 받는다. 이 인자가 아무것도 바꾸지 않아야 한다."""
    launch = pytest.importorskip("launch")
    pytest.importorskip("ament_index_python")  # ROS가 없으면 launch 폴더가 빈 패키지로 잡힌다
    import importlib.util

    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        get_package_share_directory("fake_robots")
    except PackageNotFoundError:
        pytest.skip("fake_robots is not installed")
    path = REPO / "bringup" / "iponic_bringup" / "launch" / "pc.launch.py"
    spec = importlib.util.spec_from_file_location("pc_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def made(**overrides):
        context = launch.LaunchContext()
        *args, setup = module.generate_launch_description().entities
        for action in args:
            action.visit(context)
        context.launch_configurations.update(overrides)
        return setup.execute(context)

    def describe(actions) -> str:
        return repr([(type(a).__name__, vars(a)) for a in actions])

    assert "sim_dock_stub" not in {a.name for a in module.generate_launch_description().entities
                                   if type(a).__name__ == "DeclareLaunchArgument"}
    plain = made()
    stubbed = made(sim_dock_stub="true")
    assert len(stubbed) == len(plain)
    for word in STUB_WORDS:
        assert word not in describe(stubbed)


# ---- 노드는 무엇이 띄웠든 스스로 거부한다 ----

@pytest.fixture
def ros(monkeypatch, tmp_path):
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("fake_robots")
    for name, value in ISOLATED.items():
        monkeypatch.setenv(name, value)
    for name in ("ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER", "FASTRTPS_DEFAULT_PROFILES_FILE",
                 "FASTDDS_DEFAULT_PROFILES_FILE", "FASTDDS_ENVIRONMENT_FILE",
                 "FASTDDS_BUILTIN_TRANSPORTS"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.chdir(tmp_path)
    if rclpy.get_rmw_implementation_identifier() != "rmw_fastrtps_cpp":
        pytest.skip("needs rmw_fastrtps_cpp")
    rclpy.init()
    yield rclpy
    rclpy.try_shutdown()


def stand_in(rclpy, **params):
    from fleet_gz_sim.stand_in_node import SimStandIn
    from rclpy.parameter import Parameter
    return SimStandIn(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])


def test_the_stand_in_serves_no_dock_unless_asked(ros):
    node = stand_in(ros, use_sim_time=True)
    try:
        assert node.dock_servers == []
    finally:
        node.destroy_node()


def test_the_stand_in_refuses_the_stub_on_the_wall_clock(ros):
    with pytest.raises(RuntimeError, match="use_sim_time"):
        stand_in(ros, dock_stub=True)


def test_the_stand_in_refuses_the_stub_outside_the_sim_domains(ros, monkeypatch):
    # 컨텍스트는 격리된 도메인에서 열렸고 검사만 이 값을 본다
    monkeypatch.setenv("ROS_DOMAIN_ID", "42")
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID"):
        stand_in(ros, dock_stub=True, use_sim_time=True)


def test_the_stub_answers_only_while_gazebo_time_runs():
    """/clock이 없으면(Gazebo 꺼짐) 도크 목표는 성공하지 않고 실패한다."""
    from fleet_gz_sim.dock_stand_in import stub_may_answer
    assert stub_may_answer(ros_time_active=True, now_s=12.0)
    assert not stub_may_answer(ros_time_active=False, now_s=12.0)
    assert not stub_may_answer(ros_time_active=True, now_s=0.0)


def test_a_stalled_clock_counts_as_stopped():
    """한 번 돌다 멈춘 /clock은 마지막 값을 유지하므로 대역이 알아채야 한다."""
    from fleet_gz_sim.dock_stand_in import ClockWatch
    watch = ClockWatch(limit_s=5.0)
    assert not watch.stalled(sim_now=10.0, wall_now=100.0)
    assert not watch.stalled(sim_now=10.5, wall_now=104.0)
    assert not watch.stalled(sim_now=10.5, wall_now=108.9)
    assert watch.stalled(sim_now=10.5, wall_now=109.1)
    assert not watch.stalled(sim_now=10.6, wall_now=109.2)  # runs again


def test_a_dock_ends_only_after_gazebo_time_moved():
    """duration_s 0이어도 목표 전에 멈춘 시계에서 성공하면 안 된다.
    시계가 움직이거나 ClockWatch가 멈춤으로 볼 때까지 기다린다.
    """
    from fleet_gz_sim.dock_stand_in import dock_done
    assert not dock_done(elapsed=0.0, duration=0.0)
    assert dock_done(elapsed=0.004, duration=0.0)
    assert not dock_done(elapsed=1.9, duration=2.0)
    assert dock_done(elapsed=2.0, duration=2.0)


def test_the_stub_keeps_gazebo_time_once_started(ros):
    """use_sim_time을 나중에 끄면 대역이 벽시계에 놓인다."""
    from ament_index_python.packages import get_package_share_directory
    from rclpy.parameter import Parameter
    scenario = f"{get_package_share_directory('fake_robots')}/scenarios/basic_orders.yaml"
    node = stand_in(ros, dock_stub=True, use_sim_time=True, scenario_file=scenario)
    try:
        [result] = node.set_parameters([Parameter("use_sim_time", value=False)])
        assert not result.successful
        assert node.get_parameter("use_sim_time").value is True
    finally:
        node.destroy_node()


def test_the_stand_in_serves_every_dock_when_allowed(ros):
    from ament_index_python.packages import get_package_share_directory
    scenario = f"{get_package_share_directory('fake_robots')}/scenarios/basic_orders.yaml"
    node = stand_in(ros, dock_stub=True, use_sim_time=True, scenario_file=scenario)
    try:
        assert len(node.dock_servers) == 3
    finally:
        node.destroy_node()


class _Handle:
    """SimStandIn._dock이 쓰는 도크 목표 핸들의 최소 대역."""

    def __init__(self, dock_id):
        from types import SimpleNamespace
        self.request = SimpleNamespace(dock_id=dock_id)
        self.is_cancel_requested = False
        self.ended = ""

    def abort(self):
        self.ended = "aborted"

    def succeed(self):
        self.ended = "succeeded"

    def canceled(self):
        self.ended = "canceled"

    def publish_feedback(self, feedback):
        pass


class _WallClock:
    """루프가 잘 때만 움직이는 벽시계. 루프가 고장나면 걸리지 않고 실패하도록 상한을 둔다."""

    def __init__(self, cap: int = 2000):
        self.t = 0.0
        self.sleeps = 0
        self.cap = cap

    def monotonic(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.sleeps += 1
        assert self.sleeps <= self.cap, "the dock loop did not end"
        self.t += seconds


def _docking_stub(ros, monkeypatch, sim_of_wall, duration=0.0):
    """로봇이 WORK 도크에 있는 대역. Gazebo 시각은 루프의 sleep으로 움직이는 _WallClock이고 실제 시간은 안 흐른다."""
    from types import SimpleNamespace

    from ament_index_python.packages import get_package_share_directory
    from fleet_gz_sim import stand_in_node
    scenario = f"{get_package_share_directory('fake_robots')}/scenarios/basic_orders.yaml"
    node = stand_in(ros, dock_stub=True, use_sim_time=True, scenario_file=scenario)
    node._duration = duration
    wall = _WallClock()
    monkeypatch.setattr(stand_in_node, "time",
                        SimpleNamespace(monotonic=wall.monotonic, sleep=wall.sleep))
    monkeypatch.setattr(stand_in_node, "stub_may_answer", lambda active, now: True)
    monkeypatch.setattr(node, "_now", lambda: sim_of_wall(wall.t))
    x, y = node._site.node_pose(node._site.stations["WORK"].dock)[:2]
    node._pose["pode1"] = (x, y, 0.0)
    return node, wall


def _dock(node):
    handle = _Handle("WORK")
    result = node._dock("pode1", handle)
    return result.success, handle.ended


def test_the_dock_loop_fails_a_goal_on_a_clock_that_stopped_before_it(ros, monkeypatch):
    """dock_done만이 아니라 _dock 루프 자체."""
    from fleet_gz_sim.stand_in_node import CLOCK_STALL_S
    node, wall = _docking_stub(ros, monkeypatch, lambda wall_s: 12.0)
    try:
        assert _dock(node) == (False, "aborted")
        assert wall.t > CLOCK_STALL_S, "failed by ClockWatch, not before"
    finally:
        node.destroy_node()


def test_the_dock_loop_succeeds_once_a_stopped_clock_moves_again(ros, monkeypatch):
    """목표 뒤 벽 2초 동안 Gazebo 시각이 서 있다가 흐른다: 루프가 기다리다 duration_s가 지나면 끝난다."""
    node, wall = _docking_stub(ros, monkeypatch, lambda wall_s: 12.0 + max(0.0, wall_s - 2.0),
                               duration=1.0)
    try:
        assert _dock(node) == (True, "succeeded")
        assert 3.0 <= wall.t < 3.1
    finally:
        node.destroy_node()


def test_the_dock_loop_fails_a_goal_when_the_clock_stops_partway(ros, monkeypatch):
    from fleet_gz_sim.stand_in_node import CLOCK_STALL_S
    node, wall = _docking_stub(ros, monkeypatch, lambda wall_s: 12.0 + min(wall_s, 0.5),
                               duration=2.0)
    try:
        assert _dock(node) == (False, "aborted")
        assert wall.t > 0.5 + CLOCK_STALL_S
    finally:
        node.destroy_node()


def test_a_dock_goal_fails_at_once_without_gazebo_time(ros):
    """/clock이 없으면 루프 전에 stub_may_answer 검사가 거부한다."""
    from ament_index_python.packages import get_package_share_directory
    scenario = f"{get_package_share_directory('fake_robots')}/scenarios/basic_orders.yaml"
    node = stand_in(ros, dock_stub=True, use_sim_time=True, scenario_file=scenario)
    try:
        assert node._now() == 0.0
        assert _dock(node) == (False, "aborted")
    finally:
        node.destroy_node()


def test_the_stand_in_hosts_no_peer_nodes_unless_asked(ros):
    from fleet_gz_sim.stand_in_node import peer_nodes
    node = stand_in(ros, use_sim_time=True)
    try:
        assert peer_nodes(node) == []
    finally:
        node.destroy_node()


def test_the_stand_in_hosts_a_peer_node_for_every_robot_when_asked(ros):
    """다른 로봇의 몸을 costmap에 넣는 노드(임시, 주행 시험용)를 로봇마다 이 프로세스에 둔다."""
    pytest.importorskip("pode_peer_obstacles")
    from fleet_gz_sim.stand_in_node import peer_nodes
    node = stand_in(ros, use_sim_time=True, peer_obstacles=True)
    peers = peer_nodes(node)
    try:
        assert [peer.get_namespace() for peer in peers] == ["/pode1", "/pode2", "/pode3"]
        assert [peer.robot_id for peer in peers] == ["pode1", "pode2", "pode3"]
        assert all(peer.get_parameter("use_sim_time").value for peer in peers)
    finally:
        for each in (node, *peers):
            each.destroy_node()
