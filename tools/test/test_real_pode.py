"""실물 Pode 한 대 실행 묶음(tools/real_pode): 속도 파라미터, launch 인자 이름, 셸 스크립트."""
import ast
import copy
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
KIT = ROOT / "tools" / "real_pode"
NAV2_SRC = ROOT / "pode" / "pode_nav_config" / "params" / "nav2_params.yaml"
AVOID_SRC = ROOT / "pode" / "pode_avoidance" / "params" / "avoidance.yaml"
NAV2_LAUNCH = ROOT / "pode" / "pode_nav_config" / "launch" / "nav2.launch.py"
FSM_LAUNCH = ROOT / "pode" / "pode_fsm" / "launch" / "pode_fsm.launch.py"
FORWARD = ("FollowPath", "FollowPathThrough")
# 전역 costmap 지도는 2.5 cm(arena_v7_a, 사용자 결정 2026-11-17), 위치 추정 1 cm, keepout
MAPS = ("arena_v7_a", "arena_v7_fine", "arena_v7_keepout")
# 위치 추정은 실물 SLAM 지도(2026-11-17 확정), costmap·keepout 은 v7 도면 지도
REAL_MAP = "arena_real_slam"
LAUNCH_MAPS = ("arena_v7_a", REAL_MAP, "arena_v7_keepout")

sys.path.insert(0, str(KIT))


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def _make_params():
    import make_params
    return make_params


# ---- make_params.py ----
@pytest.mark.parametrize("speed", [0.0, -0.1, 0.26, 0.25001, 0.00001, 1.0, float("nan"),
                                   float("inf")])
def test_make_params_refuses_bad_speed(tmp_path, speed):
    mp = _make_params()
    with pytest.raises(ValueError):
        mp.write_params(speed, tmp_path)
    assert not any(tmp_path.iterdir())


def test_make_params_cli_refuses_bad_speed(tmp_path):
    run = subprocess.run([sys.executable, str(KIT / "make_params.py"), "--speed", "0.3",
                          "--out", str(tmp_path / "p")], capture_output=True, text=True)
    assert run.returncode != 0
    assert not (tmp_path / "p").exists()


@pytest.mark.parametrize("speed", [0.12, 0.05, 0.25])
def test_make_params_sets_forward_speeds_only(tmp_path, speed):
    mp = _make_params()
    mp.write_params(speed, tmp_path)
    nav, nav_src = _load(tmp_path / "nav2_params.yaml"), _load(NAV2_SRC)
    avoid, avoid_src = _load(tmp_path / "avoidance.yaml"), _load(AVOID_SRC)
    ctl = nav["controller_server"]["ros__parameters"]
    for name in FORWARD:
        assert ctl[name]["desired_linear_vel"] == pytest.approx(speed)
    assert ctl["FollowPathReverse"]["desired_linear_vel"] == pytest.approx(0.05)
    assert avoid["follow_path_goal"]["desired_linear_vel"] == pytest.approx(speed)
    # 그 밖의 값은 저장소와 같다
    want = copy.deepcopy(nav_src)
    for name in FORWARD:
        want["controller_server"]["ros__parameters"][name]["desired_linear_vel"] = speed
    assert nav == want
    want = copy.deepcopy(avoid_src)
    want["follow_path_goal"]["desired_linear_vel"] = speed
    assert avoid == want


def test_make_params_keeps_comments(tmp_path):
    mp = _make_params()
    mp.write_params(0.12, tmp_path)
    for out, src in (("nav2_params.yaml", NAV2_SRC), ("avoidance.yaml", AVOID_SRC)):
        got = (tmp_path / out).read_text(encoding="utf-8").splitlines()
        orig = src.read_text(encoding="utf-8").splitlines()
        assert len(got) == len(orig)
        changed = [i for i, (a, b) in enumerate(zip(got, orig, strict=True)) if a != b]
        assert all("desired_linear_vel" in got[i] for i in changed)
        assert len(changed) == (2 if out == "nav2_params.yaml" else 1)


def test_make_params_cli_writes_and_speed_of_reads(tmp_path):
    run = subprocess.run([sys.executable, str(KIT / "make_params.py"), "--speed", "0.12",
                          "--out", str(tmp_path / "p")], capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    mp = _make_params()
    assert mp.speed_of(tmp_path / "p" / "nav2_params.yaml") == pytest.approx(0.12)


def test_horizon_cap_follows_params(tmp_path):
    mp = _make_params()
    mp.write_params(0.12, tmp_path)
    params = tmp_path / "nav2_params.yaml"
    assert float(mp.horizon_cap("", params)) == pytest.approx(0.12)
    assert float(mp.horizon_cap("0.10", params)) == pytest.approx(0.10)
    with pytest.raises(ValueError):
        mp.horizon_cap("0.20", params)  # 컨트롤러 속도보다 크면 안 된다
    with pytest.raises(ValueError):
        mp.horizon_cap("abc", params)


def test_speed_of_without_params_says_what_to_run(tmp_path):
    mp = _make_params()
    with pytest.raises(FileNotFoundError, match="make_params"):
        mp.speed_of(tmp_path / "nav2_params.yaml")


# ---- real_pode.launch.py: 지금 패키지의 인자 이름 ----
def _declared(path):
    return set(re.findall(r'DeclareLaunchArgument\(\s*"([a-z_]+)"', path.read_text("utf-8")))


def _constant(name):
    tree = ast.parse((KIT / "real_pode.launch.py").read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} not in real_pode.launch.py")


def test_nav2_arguments_exist_in_current_launch():
    passed = set(_constant("NAV2_ARGS"))
    assert passed <= _declared(NAV2_LAUNCH)
    assert {"namespace", "use_sim_time", "map", "map_fine", "keepout_mask", "params_file",
            "avoidance_params_file", "initial_x", "initial_y", "initial_yaw",
            "peer_obstacles", "peer_robots", "log_level", "use_composition"} <= passed


def test_fsm_arguments_exist_in_current_launch():
    passed = set(_constant("FSM_ARGS"))
    assert passed <= _declared(FSM_LAUNCH)
    assert {"namespace", "use_sim_time", "heartbeat_timeout_s", "through_controller_id",
            "horizon_cap_mps"} <= passed


def _need_launch():
    # launch 계열이 없으면 이 시험들은 건너뛴다
    for name in ("launch", "launch_ros", "ament_index_python"):
        pytest.importorskip(name)


def _fsm_actions(dock_aside):
    # _fsm()이 돌려주는 액션 목록(패키지 설치 없이: 경로 조회와 속도 읽기는 대역)
    _need_launch()
    import importlib.util
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location("real_pode_launch", KIT / "real_pode.launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.get_package_share_directory = lambda name: "/nonexistent/" + name
    mod.horizon_cap = lambda cap, params: "0.1"
    ctx = LaunchContext()
    ctx.launch_configurations.update(
        namespace="pode1", use_sim_time="false", heartbeat_timeout_s="1.5",
        horizon_cap_mps="", battery_topic="", params_file="x", dock_aside=dock_aside)
    return mod._fsm(ctx), ctx


def test_launch_declares_dock_aside_default_false():
    import importlib.util
    _need_launch()
    from launch.actions import DeclareLaunchArgument
    spec = importlib.util.spec_from_file_location("real_pode_launch2", KIT / "real_pode.launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.get_package_share_directory = lambda name: "/nonexistent/" + name
    decl = [a for a in mod.generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument) and a.name == "dock_aside"]
    assert len(decl) == 1
    assert "".join(t.text for t in decl[0].default_value) == "false"


def test_launch_default_does_not_wrap_fsm():
    _need_launch()
    from launch.actions import GroupAction, IncludeLaunchDescription
    acts, _ = _fsm_actions("false")
    assert any(isinstance(a, IncludeLaunchDescription) for a in acts)
    assert not any(isinstance(a, GroupAction) for a in acts)


def test_launch_dock_aside_wraps_fsm_with_remap():
    _need_launch()
    from launch.actions import GroupAction, IncludeLaunchDescription
    from launch.utilities import perform_substitutions
    from launch_ros.actions import SetRemap
    acts, ctx = _fsm_actions("true")
    group = next(a for a in acts if isinstance(a, GroupAction))
    inner = group.get_sub_entities()
    remap = next(a for a in inner if isinstance(a, SetRemap))
    assert perform_substitutions(ctx, remap.src) == "/pode1/dock"
    assert perform_substitutions(ctx, remap.dst) == "/pode1/dock_fsm"
    assert any(isinstance(a, IncludeLaunchDescription) for a in inner)


# ---- 실제 pode_fsm으로 확인: remap이 Dock 서버 이름에 닿는가 ----
PROBE_LAUNCH = """import importlib.util
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
spec = importlib.util.spec_from_file_location("rp", {path!r})
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def generate_launch_description():
    d = {{"namespace": "pode1", "use_sim_time": "false", "heartbeat_timeout_s": "0",
         "horizon_cap_mps": "0.1", "battery_topic": "", "params_file": {params!r},
         "dock_aside": "false", "through_controller_id": "FollowPathThrough"}}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in d.items()]
                             + [OpaqueFunction(function=mod._fsm)])
"""


def _action_servers(env, name):
    run = subprocess.run(["ros2", "action", "info", name], capture_output=True, text=True,
                         env=env, timeout=30)
    assert run.returncode == 0, f"ros2 action info {name} failed: {run.stderr}"
    assert "Action servers:" in run.stdout, f"unexpected output: {run.stdout!r}"
    text = run.stdout.split("Action servers:")[-1]
    return [t.strip() for t in text.splitlines()[1:] if t.strip()]


def _group_alive(pgid):
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _stop_group(proc):
    # 그룹 전체가 없어질 때까지 INT -> TERM -> KILL로 올린다. 정리 오류가 시험 결과를 가리지 않게 한다
    import signal
    pgid = proc.pid
    for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 10), (signal.SIGKILL, 10)):
        try:
            os.killpg(pgid, sig)
        except (ProcessLookupError, PermissionError):
            pass
        end = time.time() + wait
        while time.time() < end:
            if proc.poll() is not None:
                proc.wait()  # 리더 좀비를 거둔다
            if not _group_alive(pgid):
                return
            time.sleep(0.1)


@pytest.mark.parametrize("aside,present,absent", [("true", "dock_fsm", "dock"),
                                                  ("false", "dock", "dock_fsm")])
def test_launch_dock_aside_moves_real_fsm_server(tmp_path, aside, present, absent):
    if shutil.which("ros2") is None:
        pytest.skip("ros2 not available")
    if subprocess.run(["ros2", "pkg", "prefix", "pode_fsm"], capture_output=True).returncode:
        pytest.skip("pode_fsm not installed")
    # 호출한 쪽이 도메인을 정한다(12, 50~59는 쓰지 않는다)
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if (not domain.isdigit() or int(domain) == 12 or 50 <= int(domain) <= 59
            or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"):
        pytest.skip("set ROS_DOMAIN_ID (not 12, 50-59) and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
    env = dict(os.environ)
    # 같은 도메인에 이미 서버가 있으면 결과를 믿을 수 없다
    for name in ("/pode1/dock", "/pode1/dock_fsm"):
        if _action_servers(env, name):
            pytest.skip(f"a server already exists on {name} in this domain")
    import make_params
    make_params.write_params(0.12, tmp_path / "params")
    (tmp_path / "probe.launch.py").write_text(PROBE_LAUNCH.format(
        path=str(KIT / "real_pode.launch.py"),
        params=str(tmp_path / "params" / "nav2_params.yaml")))
    launch = subprocess.Popen(["ros2", "launch", str(tmp_path / "probe.launch.py"),
                               f"dock_aside:={aside}"], env=env, start_new_session=True,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        want, other = f"/pode1/{present}", f"/pode1/{absent}"
        servers = []
        for _ in range(20):
            servers = _action_servers(env, want)
            if servers:
                break
            time.sleep(0.5)
        assert servers == ["/pode1/pode_fsm"]
        assert _action_servers(env, other) == []
    finally:
        _stop_group(launch)


def test_launch_keeps_shutdown():
    text = (KIT / "real_pode.launch.py").read_text(encoding="utf-8")
    assert "OnProcessExit" in text and "Shutdown(" in text
    assert "cmd_watchdog.py" in text
    assert "bringup_robot.launch.xml" in text
    for name in LAUNCH_MAPS:
        assert f"{name}.yaml" in text
    assert 'MAPS / "arena_v7.yaml"' not in text  # 5 cm 지도는 기본이 아니다


def test_amcl_reads_the_real_slam_map():
    text = (KIT / "real_pode.launch.py").read_text(encoding="utf-8")
    line = next(t for t in text.splitlines() if '"map_fine"' in t and "DeclareLaunchArgument" in t)
    assert f'MAPS / "{REAL_MAP}.yaml"' in line


def test_real_slam_map_file():
    from PIL import Image
    meta = _load(KIT / "maps" / f"{REAL_MAP}.yaml")
    assert meta["image"] == f"{REAL_MAP}.pgm" and meta["mode"] == "trinary"
    assert meta["resolution"] == 0.025 and meta["origin"] == [-0.1, -0.1, 0.0]
    # 205(미지)는 미지로 남아야 한다
    p_unknown = (255 - 205) / 255.0
    assert meta["free_thresh"] < p_unknown < meta["occupied_thresh"]
    img = Image.open(KIT / "maps" / meta["image"])
    assert img.size == (88, 48)
    px = img.load()

    def cell(x, y):  # 경기장 좌표 -> 픽셀 값
        return px[int((x + 0.1) / 0.025), 47 - int((y + 0.1) / 0.025)]
    # 네 벽은 직선: 안쪽 면 x 0 / 1.975, y 0 / 0.975
    for i in range(79):
        x = 0.0125 + i * 0.025
        assert cell(x, -0.0125) == 0 and cell(x, 0.9875) == 0
    for i in range(39):
        y = 0.0125 + i * 0.025
        assert cell(-0.0125, y) == 0 and cell(1.9875, y) == 0
    # 급행 통로 가운데와 남쪽 통로는 빈칸, 진열대 B 는 점유
    assert cell(1.0, 0.89) == 254 and cell(1.0, 0.2) == 254
    assert cell(0.83, 0.37) == 0  # 진열대 B 남쪽 면 (속은 미지)


def test_kit_maps_equal_sim_maps():
    for name in MAPS:
        for ext in (".yaml", ".pgm"):
            kit = KIT / "maps" / (name + ext)
            assert kit.read_bytes() == (ROOT / "sim" / "fleet_gz_sim" / "maps" /
                                        (name + ext)).read_bytes()


# ---- cmd_watchdog.py ----
def test_watchdog_rule_without_ros():
    import cmd_watchdog
    assert cmd_watchdog.WATCHDOG_S == pytest.approx(0.4)
    assert not cmd_watchdog.overdue(None, 100.0)
    assert not cmd_watchdog.overdue(100.0, 100.3)
    assert cmd_watchdog.overdue(100.0, 100.5)


# ---- 셸 스크립트 ----
SCRIPTS = ("start.sh", "stop.sh", "env.sh", "deploy.sh")


@pytest.mark.parametrize("name", SCRIPTS)
def test_scripts_parse(name):
    assert subprocess.run(["bash", "-n", str(KIT / name)]).returncode == 0


def test_env_values():
    text = (KIT / "env.sh").read_text(encoding="utf-8")
    assert "export ROS_DOMAIN_ID=12" in text
    assert "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" in text
    assert re.search(r"unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS", text)
    for path in ("/opt/ros/jazzy/setup.bash", "pinky_pro/install/local_setup.bash",
                 "nav2fleet_ws/install/local_setup.bash"):
        assert path in text


def test_start_usage_and_speed_refusal():
    run = subprocess.run(["bash", str(KIT / "start.sh"), "pode1", "0.1"],
                         capture_output=True, text=True)
    assert run.returncode == 2
    before = sorted(p.name for p in KIT.iterdir() if p.name != "logs")
    run = subprocess.run(["bash", str(KIT / "start.sh"), "pode1", "0.12", "0.10", "1.5708",
                          "0.3"], capture_output=True, text=True)
    assert run.returncode != 0
    # 잠금 파일(logs/) 말고는 아무것도 만들지 않았다
    assert sorted(p.name for p in KIT.iterdir() if p.name != "logs") == before
    assert not (KIT / "logs" / "launch.pid").exists()


def _stub_kit(tmp_path):
    # start.sh만 진짜, make_params와 env는 대역(env에서 멈춰 launch까지 가지 않는다)
    shutil.copy(KIT / "start.sh", tmp_path)
    (tmp_path / "make_params.py").write_text("print('params ok')\n")
    (tmp_path / "env.sh").write_text("echo ENV_REACHED\nreturn 1\n")
    return tmp_path / "start.sh"


def test_start_refuses_while_another_start_holds_the_lock(tmp_path):
    import fcntl
    start = _stub_kit(tmp_path)
    (tmp_path / "logs").mkdir()
    with open(tmp_path / "logs" / "start.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        run = subprocess.run(["bash", str(start), "pode1", "0.12", "0.10", "1.5708"],
                             capture_output=True, text=True, timeout=10)
    assert run.returncode == 1
    assert "another start.sh" in run.stdout and "ENV_REACHED" not in run.stdout
    assert "params ok" not in run.stdout  # 잠금보다 먼저 사본을 바꾸지 않는다
    # 잠금이 풀리면 env까지 간다(대역 env가 거기서 멈춘다)
    run = subprocess.run(["bash", str(start), "pode1", "0.12", "0.10", "1.5708"],
                         capture_output=True, text=True, timeout=10)
    assert run.returncode == 1 and "ENV_REACHED" in run.stdout


def _start_with_fake_ros2(tmp_path, peer_robots, dock_stub=None):
    # 대역 ros2가 받은 인자를 파일에 적고 끝난다. PEER_ROBOTS=None이면 변수를 빼고 돌린다
    start = _stub_kit(tmp_path)
    (tmp_path / "env.sh").write_text("return 0\n")
    fake = tmp_path / "bin"
    fake.mkdir()
    (fake / "ros2").write_text(f'#!/bin/bash\nprintf "%s\\n" "$@" > "{tmp_path}/ros2.args"\n')
    (fake / "ros2").chmod(0o755)
    env = dict(os.environ, PATH=f"{fake}:{os.environ['PATH']}")
    env.pop("PEER_ROBOTS", None)
    if peer_robots is not None:
        env["PEER_ROBOTS"] = peer_robots
    env.pop("DOCK_STUB", None)
    if dock_stub is not None:
        env["DOCK_STUB"] = dock_stub
    run = subprocess.run(["bash", str(start), "pode1", "0.12", "0.10", "1.5708"],
                         capture_output=True, text=True, env=env, timeout=30)
    args = tmp_path / "ros2.args"
    for _ in range(50):
        if args.exists() and args.read_text().strip():
            break
        time.sleep(0.1)
    return run, (args.read_text().splitlines() if args.exists() else [])


# 예전 start.sh가 ros2에 넘기던 인자 그대로
OLD_ARGV = ["launch", "./real_pode.launch.py", "namespace:=pode1", "initial_x:=0.12",
            "initial_y:=0.10", "initial_yaw:=1.5708"]


@pytest.mark.parametrize("value", ["pode1,pode2", "pode2,pode3", "pode_a"])
def test_start_peer_robots_adds_peer_arguments(tmp_path, value):
    run, argv = _start_with_fake_ros2(tmp_path, value)
    assert run.returncode == 0, run.stdout + run.stderr
    assert argv == OLD_ARGV + ["peer_obstacles:=true", f"peer_robots:={value}"]


@pytest.mark.parametrize("value", [None, ""])
def test_start_without_peer_robots_has_no_peer_arguments(tmp_path, value):
    run, argv = _start_with_fake_ros2(tmp_path, value)
    assert run.returncode == 0, run.stdout + run.stderr
    assert argv == OLD_ARGV


@pytest.mark.parametrize("value", ["pode1;rm", "pode1,,pode2", "Pode1", "pode1,", ",pode1", "a b",
                                   "2robot", "pode1,3x"])
def test_start_refuses_bad_peer_robots_before_make_params(tmp_path, value):
    run, argv = _start_with_fake_ros2(tmp_path, value)
    assert run.returncode == 2
    assert "PEER_ROBOTS" in run.stdout + run.stderr
    assert "params ok" not in run.stdout and argv == []


# ---- DOCK_STUB ----
def test_start_dock_stub_adds_dock_aside(tmp_path):
    run, argv = _start_with_fake_ros2(tmp_path, None, "1")
    assert run.returncode == 0, run.stdout + run.stderr
    assert argv == OLD_ARGV + ["dock_aside:=true"]


def test_start_dock_stub_with_peers_order(tmp_path):
    run, argv = _start_with_fake_ros2(tmp_path, "pode1,pode2", "1")
    assert run.returncode == 0, run.stdout + run.stderr
    assert argv == OLD_ARGV + ["peer_obstacles:=true", "peer_robots:=pode1,pode2",
                               "dock_aside:=true"]


@pytest.mark.parametrize("value", [None, "", "0"])
def test_start_dock_stub_off_keeps_old_argv(tmp_path, value):
    run, argv = _start_with_fake_ros2(tmp_path, None, value)
    assert run.returncode == 0, run.stdout + run.stderr
    assert argv == OLD_ARGV


@pytest.mark.parametrize("value", ["yes", "true", "2", "1;rm", " 1"])
def test_start_refuses_bad_dock_stub(tmp_path, value):
    run, argv = _start_with_fake_ros2(tmp_path, None, value)
    assert run.returncode == 2
    assert "DOCK_STUB" in run.stdout + run.stderr
    assert "params ok" not in run.stdout and argv == []


# 대역 setsid들: launch.pid를 쓰지 않고 잠만 잔다(ros2까지 가지 않는다)
SLEEPER = '#!/bin/bash\necho $$ > "{d}/fake.pid"\nexec sleep 30\n'
# 진짜 setsid로 새 그룹을 만들고 그룹에 프로세스 둘을 둔다
GROUP = ('#!/bin/bash\nexec /usr/bin/setsid bash -c \'echo $$ > "{d}/fake.pid"; '
         'sleep 30 & echo $! > "{d}/fake2.pid"; exec sleep 30\'\n')
# 새 그룹에서 TERM을 무시한다
DEAF = ('#!/bin/bash\nexec /usr/bin/setsid bash -c \'trap "" TERM; echo $$ > "{d}/fake.pid"; '
        'exec sleep 30\'\n')


def _start_with_fake_setsid(tmp_path, script):
    start = _stub_kit(tmp_path)
    (tmp_path / "env.sh").write_text("return 0\n")
    fake = tmp_path / "bin"
    fake.mkdir()
    (fake / "setsid").write_text(script.format(d=tmp_path))
    (fake / "setsid").chmod(0o755)
    env = dict(os.environ, PATH=f"{fake}:{os.environ['PATH']}")
    return subprocess.run(["bash", str(start), "pode1", "0.12", "0.10", "1.5708"],
                          capture_output=True, text=True, env=env, timeout=30)


def _fake_pids(tmp_path):
    return [int(f.read_text()) for f in (tmp_path / "fake.pid", tmp_path / "fake2.pid")
            if f.exists() and f.read_text().strip()]


def _gone(pids, wait_s=2.0):
    end = time.monotonic() + wait_s
    while time.monotonic() < end and any(_alive(p) for p in pids):
        time.sleep(0.1)
    return not any(_alive(p) for p in pids)


def _kill_all(pids):
    # 살아 있는 것만 죽이고, 그사이 사라지면 넘어간다(죽은 번호를 다른 프로세스가 받았을 수 있다)
    for pid in pids:
        if not _alive(pid):
            continue
        try:
            os.kill(pid, 9)
        except ProcessLookupError:
            pass


@pytest.mark.parametrize("script", [SLEEPER, GROUP], ids=["child_only", "own_group"])
def test_start_stops_a_launch_whose_pid_file_never_appears(tmp_path, script):
    try:
        run = _start_with_fake_setsid(tmp_path, script)
        pids = _fake_pids(tmp_path)
        assert pids
        if script is GROUP:
            assert len(pids) == 2
        assert run.returncode == 1
        assert "stopped it" in run.stdout, run.stdout
        assert _gone(pids, 0.5)  # start.sh가 끝날 때 이미 없다
    finally:
        _kill_all(_fake_pids(tmp_path))


def test_start_says_so_when_the_launch_ignores_term(tmp_path):
    try:
        run = _start_with_fake_setsid(tmp_path, DEAF)
        assert run.returncode == 1
        assert "still running" in run.stdout and "stopped it" not in run.stdout, run.stdout
        pids = _fake_pids(tmp_path)
        assert pids
        assert all(_alive(p) for p in pids)  # KILL로 올리지 않는다
    finally:
        _kill_all(_fake_pids(tmp_path))


def _own_group(tmp_path, *argv):
    proc = subprocess.Popen(list(argv), start_new_session=True, cwd=tmp_path)
    (tmp_path / "logs").mkdir(exist_ok=True)
    (tmp_path / "logs" / "launch.pid").write_text(str(proc.pid))
    return proc


def test_stop_not_running(tmp_path):
    shutil.copy(KIT / "stop.sh", tmp_path)
    run = subprocess.run(["bash", str(tmp_path / "stop.sh")], capture_output=True, text=True)
    assert run.returncode == 0 and "not running" in run.stdout


def test_stop_refuses_a_group_that_is_not_ours(tmp_path):
    shutil.copy(KIT / "stop.sh", tmp_path)
    proc = _own_group(tmp_path, "sleep", "30")
    try:
        run = subprocess.run(["bash", str(tmp_path / "stop.sh")], capture_output=True, text=True)
        assert run.returncode == 1
        assert proc.poll() is None
    finally:
        proc.kill()
        proc.wait()


def _alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def test_stop_stops_our_group(tmp_path):
    # start.sh처럼 부모가 먼저 끝나는 새 세션으로 띄운다(좀비가 남지 않게)
    shutil.copy(KIT / "stop.sh", tmp_path)
    (tmp_path / "logs").mkdir()
    pid = int(subprocess.run(
        ["bash", "-c", f"setsid {sys.executable} -c 'import time; time.sleep(30)' "
                       "real_pode.launch.py > /dev/null 2>&1 & echo $!"],
        capture_output=True, text=True, check=True).stdout)
    (tmp_path / "logs" / "launch.pid").write_text(str(pid))
    try:
        time.sleep(0.3)
        run = subprocess.run(["bash", str(tmp_path / "stop.sh")], capture_output=True, text=True,
                             timeout=30)
        assert run.returncode == 0, run.stdout + run.stderr
        assert not _alive(pid)
    finally:
        if _alive(pid):
            os.kill(pid, 9)


PACKAGES = ("interfaces/fleet_interfaces", "pode/pode_fsm", "pode/pode_nav_config",
            "pode/pode_avoidance", "pode/pode_peer_obstacles", "pode/pode_scan_filter",
            "pode/pode_dock")


def _fake_repo(tmp_path):
    repo = tmp_path / "repo"
    for pkg in PACKAGES:
        (repo / pkg).mkdir(parents=True)
        (repo / pkg / "package.xml").write_text(pkg)
    (repo / "pode" / "other_pkg").mkdir()
    (repo / "pode" / "other_pkg" / "package.xml").write_text("x")
    shutil.copytree(KIT, repo / "tools" / "real_pode",
                    ignore=shutil.ignore_patterns("logs", "params", "__pycache__"))
    git = ["git", "-C", str(repo), "-c", "user.name=t", "-c", "user.email=t@t"]
    subprocess.run(git[:3] + ["init", "-q"], check=True)
    subprocess.run(git + ["add", "-A"], check=True)
    subprocess.run(git + ["commit", "-qm", "t"], check=True)
    return repo


def test_deploy_copies_only_the_kit_and_packages(tmp_path):
    repo = _fake_repo(tmp_path)
    fake = tmp_path / "bin"
    fake.mkdir()
    log = tmp_path / "calls.log"
    for tool in ("ssh", "rsync", "scp", "colcon", "ros2"):
        (fake / tool).write_text(f'#!/bin/bash\necho "{tool} $*" >> "{log}"\n')
        (fake / tool).chmod(0o755)
    env = dict(os.environ, PATH=f"{fake}:{os.environ['PATH']}")
    run = subprocess.run(["bash", str(repo / "tools" / "real_pode" / "deploy.sh"),
                          "pinky@robot"], capture_output=True, text=True, env=env, timeout=60)
    assert run.returncode == 0, run.stdout + run.stderr
    calls = log.read_text().splitlines()
    assert not any(c.startswith(("colcon", "ros2")) for c in calls)
    assert "cd ~/nav2fleet_ws && colcon build" in run.stdout
    rsyncs = [c for c in calls if c.startswith("rsync")]
    dests = [c.split()[-1] for c in rsyncs]
    for pkg in PACKAGES:
        assert f"pinky@robot:~/nav2fleet_ws/src/{pkg}/" in dests
    assert "pinky@robot:~/nav2fleet_real/" in dests
    assert not any("other_pkg" in c for c in calls)
    for c, d in zip(rsyncs, dests, strict=True):
        path = d.split(":", 1)[1]
        assert path.startswith(("~/nav2fleet_ws/src/", "~/nav2fleet_real/"))
        if "--delete" in c:
            # 지우기는 패키지 폴더 안에서만(실행 묶음의 logs, params는 지우지 않는다)
            assert path.startswith("~/nav2fleet_ws/src/") and path != "~/nav2fleet_ws/src/"
    for c in calls:
        if c.startswith("ssh"):
            assert "rm " not in c and "colcon" not in c and "launch" not in c


def test_deploy_needs_a_host():
    run = subprocess.run(["bash", str(KIT / "deploy.sh")], capture_output=True, text=True)
    assert run.returncode == 2


# ---- 주석은 한국어로 ----
HANGUL = re.compile("[가-힣]")


@pytest.mark.parametrize("name", ["real_pode.launch.py", "make_params.py", "cmd_watchdog.py",
                                  *SCRIPTS])
def test_comments_are_korean(name):
    for number, line in enumerate((KIT / name).read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.strip()
        if number == 1 and stripped.startswith("#!"):
            continue
        if "#" not in stripped:
            continue
        comment = stripped[stripped.index("#"):] if stripped.startswith("#") else None
        if comment is None:
            match = re.search(r"\s#\s(.*)$", line)
            if not match or match.group(0).count('"') or match.group(0).count("'"):
                continue
            comment = match.group(0).strip()
        body = comment.lstrip("#").strip()
        if not body or re.fullmatch(r"-+ .* -+", body) or "noqa" in body:
            continue
        assert HANGUL.search(body), f"{name}:{number}: {line}"

