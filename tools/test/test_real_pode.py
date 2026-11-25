"""실물 Pode 한 대 실행 묶음(tools/real_pode): 속도 파라미터, launch 인자 이름, 셸 스크립트."""
import ast
import copy
import math
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
FIXED_LINE = ("FollowPath", "FollowPathThrough", "FollowPathReverse")
# 전역 costmap 지도 2.5 cm(arena_v7_a), 위치 추정 1 cm, keepout
MAPS = ("arena_v7_a", "arena_v7_fine", "arena_v7_keepout")
# 위치 추정도 v7 도면 1 cm 지도(시뮬과 같다). 옛 SLAM 지도는 map_fine 인자로 고른다
AMCL_MAP = "arena_v7_fine"
OLD_SLAM_MAP = "arena_real_slam"
LAUNCH_MAPS = ("arena_v7_a", AMCL_MAP, "arena_v7_keepout")
# 실물 키트 지도 그림의 SHA-256. 지도를 일부러 바꿀 때만 같이 고친다
KIT_MAP_SHA256 = {
    "arena_v7.pgm": "6435e48dd37ec706cbb932955d23e069d9c3bdaa09e22dbe14ebc65aaf7545c3",
    "arena_v7_a.pgm": "c81b10325779080bfcc38970dbc7d84e63eeff39e1e1fdc8a3380e09b2ecc8df",
    "arena_v7_a1.pgm": "feac937f80c535fdd37d1397181b57a69a721f662285fca957c7e8d7aceb4e51",
    "arena_v7_fine.pgm": "feac937f80c535fdd37d1397181b57a69a721f662285fca957c7e8d7aceb4e51",
    "arena_v7_keepout.pgm": "6783b8bc1de5c1cb4e4efec931234910119381d30e6b3cc9af92566a9b83246c",
    "arena_real_slam.pgm": "553e4be9dee15d25f480b5b5dfbb29c94b8f9d981cf7859d4f6a213ec78643d1",
}

sys.path.insert(0, str(KIT))


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def _make_params():
    import make_params
    return make_params


# ---- make_params.py ----
SPEED_SRC = ROOT / "pode" / "pode_nav_config" / "params" / "nav_speed.yaml"
TOP = _load(SPEED_SRC)["nav_speed_mps"]


@pytest.mark.parametrize("speed", [0.0, -0.1, TOP + 0.01, TOP + 0.00001, 0.00001, 1.0,
                                   float("nan"), float("inf")])
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


@pytest.mark.parametrize("speed", [TOP, 0.05, 0.10])
def test_make_params_sets_forward_speeds_only(tmp_path, speed):
    mp = _make_params()
    mp.write_params(speed, tmp_path)
    nav, nav_src = _load(tmp_path / "nav2_params.yaml"), _load(NAV2_SRC)
    avoid, avoid_src = _load(tmp_path / "avoidance.yaml"), _load(AVOID_SRC)
    ctl = nav["controller_server"]["ros__parameters"]
    for name in FORWARD:
        assert ctl[name]["desired_linear_vel"] == pytest.approx(speed)
    assert "FollowPathMPPI" not in ctl["controller_plugins"]
    assert ctl["FollowPathReverse"]["desired_linear_vel"] == pytest.approx(0.05)
    assert avoid["follow_path_goal"]["desired_linear_vel"] == pytest.approx(speed)
    # 그 밖의 값은 저장소와 같다
    want = copy.deepcopy(nav_src)
    for name in FORWARD:
        want["controller_server"]["ros__parameters"][name]["desired_linear_vel"] = speed
    for name in FIXED_LINE:
        want["controller_server"]["ros__parameters"][name]["use_collision_detection"] = False
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
        assert all("desired_linear_vel" in got[i] or "use_collision_detection" in got[i]
                   for i in changed)
        assert len(changed) == (5 if out == "nav2_params.yaml" else 1)


# ---- 고정 경로 충돌 검사(FIXED_LINE_CHECK) ----
def test_make_params_fixed_line_check_off_by_default(tmp_path):
    mp = _make_params()
    mp.write_params(0.12, tmp_path)
    ctl = _load(tmp_path / "nav2_params.yaml")["controller_server"]["ros__parameters"]
    for name in FIXED_LINE:
        assert ctl[name]["use_collision_detection"] is False
    # 마당 goal(FollowPathGoal)은 그대로 켜 둔다
    assert _load(tmp_path / "avoidance.yaml")["follow_path_goal"]["use_collision_detection"] is True
    assert _load(AVOID_SRC)["follow_path_goal"]["use_collision_detection"] is True


def test_make_params_fixed_line_check_flag_keeps_true(tmp_path):
    mp = _make_params()
    mp.write_params(0.12, tmp_path, fixed_line_check=True)
    ctl = _load(tmp_path / "nav2_params.yaml")["controller_server"]["ros__parameters"]
    for name in FIXED_LINE:
        assert ctl[name]["use_collision_detection"] is True
    assert _load(tmp_path / "avoidance.yaml")["follow_path_goal"]["use_collision_detection"] is True


@pytest.mark.parametrize("flag,word", [([], "off"), (["--fixed-line-check"], "on")])
def test_make_params_cli_fixed_line_check(tmp_path, flag, word):
    run = subprocess.run([sys.executable, str(KIT / "make_params.py"), "--speed", "0.12",
                          "--out", str(tmp_path / "p"), *flag], capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    lines = [ln for ln in run.stdout.splitlines() if "fixed-line check" in ln]
    assert len(lines) == 1 and lines[0].endswith(word)
    ctl = _load(tmp_path / "p" / "nav2_params.yaml")["controller_server"]["ros__parameters"]
    assert all(ctl[n]["use_collision_detection"] is (word == "on") for n in FIXED_LINE)


@pytest.mark.parametrize("flag", [True, False])
def test_make_params_fixed_line_check_writes_explicit_value(tmp_path, flag):
    # 원본이 반대 값이어도 플래그대로 세 키를 명시해서 쓴다
    src = tmp_path / "src.yaml"
    text = NAV2_SRC.read_text(encoding="utf-8")
    src.write_text(text.replace("use_collision_detection: true",
                                f"use_collision_detection: {str(not flag).lower()}"),
                   encoding="utf-8")
    mp = _make_params()
    mp.write_params(0.12, tmp_path / "o", nav2_src=src, fixed_line_check=flag)
    ctl = _load(tmp_path / "o" / "nav2_params.yaml")["controller_server"]["ros__parameters"]
    for name in FIXED_LINE:
        assert ctl[name]["use_collision_detection"] is flag


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
            "peer_obstacles", "peer_robots", "log_level", "use_composition", "plan_keep"} <= passed


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


def test_launch_declares_plan_keep_default_false():
    # 길 유지 노드(해결안 A)는 실물에서 기본 꺼짐
    import importlib.util
    _need_launch()
    from launch.actions import DeclareLaunchArgument
    spec = importlib.util.spec_from_file_location("real_pode_launch3", KIT / "real_pode.launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.get_package_share_directory = lambda name: "/nonexistent/" + name
    decl = [a for a in mod.generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument) and a.name == "plan_keep"]
    assert len(decl) == 1
    assert "".join(t.text for t in decl[0].default_value) == "false"


# ---- deploy.sh: 복사하는 패키지가 기대는 저장소 패키지도 함께 복사한다 ----
def _deploy_packages():
    text = (KIT / "deploy.sh").read_text(encoding="utf-8")
    m = re.search(r"^packages=\(([^)]*)\)", text, re.M)
    assert m, "packages=(...) not in deploy.sh"
    return m.group(1).split()


def _repo_packages():
    out = subprocess.run(["git", "-C", str(ROOT), "ls-files", "*package.xml"],
                         capture_output=True, text=True, check=True).stdout.split()
    found = {}
    for rel in out:
        name = re.search(r"<name>([^<]+)</name>", (ROOT / rel).read_text("utf-8")).group(1)
        found[name] = str(Path(rel).parent)
    return found


def test_deploy_copies_every_repo_dependency():
    repo = _repo_packages()
    deployed = _deploy_packages()
    assert "pode/pode_plan_keep" in deployed
    for rel in deployed:
        xml = (ROOT / rel / "package.xml").read_text("utf-8")
        for dep in re.findall(r"<(?:exec_depend|depend|build_depend)>([^<]+)<", xml):
            if dep in repo:
                assert repo[dep] in deployed, f"{rel} needs {dep} ({repo[dep]})"


# ---- start.sh: PLAN_KEEP=1이면 launch에 plan_keep:=true ----
def _start_args(tmp_path, extra):
    """start.sh는 진짜, 대역 ros2가 받은 인자를 한 줄에 하나씩 적는다."""
    kit = tmp_path / "kit"
    kit.mkdir()
    for name in ("start.sh", "env.sh", "net.sh"):
        shutil.copy(KIT / name, kit)
    (kit / "make_params.py").write_text("print('params ok')\n")
    fake = tmp_path / "bin"
    fake.mkdir()
    seen = tmp_path / "ros2.args"
    (fake / "ros2").write_text(f'#!/bin/bash\nprintf "%s\\n" "$@" > "{seen}"\n')
    (fake / "ros2").chmod(0o755)
    env = {"PATH": f"{fake}:/usr/bin:/bin", **_robot_home(tmp_path), **extra}
    run = subprocess.run(["bash", str(kit / "start.sh"), "pode1", "0.12", "0.10", "1.5708"],
                         env=env, capture_output=True, text=True, timeout=30)
    for _ in range(50):
        if seen.exists() and seen.read_text().strip():
            break
        time.sleep(0.1)
    return run, seen.read_text().split("\n") if seen.exists() else None


@pytest.mark.parametrize("extra, want", [({}, False), ({"PLAN_KEEP": "0"}, False),
                                         ({"PLAN_KEEP": "1"}, True)])
def test_start_plan_keep(tmp_path, extra, want):
    run, args = _start_args(tmp_path, extra)
    assert run.returncode == 0, run.stdout + run.stderr
    assert args is not None and "namespace:=pode1" in args
    assert ("plan_keep:=true" in args) == want
    assert not any(a.startswith("plan_keep:=") and a != "plan_keep:=true" for a in args)


def test_start_refuses_bad_plan_keep(tmp_path):
    run, args = _start_args(tmp_path, {"PLAN_KEEP": "yes"})
    assert run.returncode == 2
    assert "bad PLAN_KEEP" in run.stdout
    assert args is None


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
    except OSError:
        return True
    return True


def _stop_group(proc):
    # 그룹 전체가 없어질 때까지 INT -> TERM -> KILL로 올린다.
    # 정리 오류가 시험 결과를 가리지 않게 한다
    import signal
    pgid = proc.pid
    for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 10), (signal.SIGKILL, 10)):
        try:
            os.killpg(pgid, sig)
        except OSError:
            pass
        end = time.time() + wait
        while time.time() < end:
            if proc.poll() is not None:
                proc.wait()  # 리더 좀비를 거둔다
            if not _group_alive(pgid):
                return
            time.sleep(0.1)
    import warnings
    warnings.warn(f"process group {pgid} still alive after SIGKILL")  # noqa: B028


@pytest.mark.parametrize("aside,present,absent", [("true", "dock_fsm", "dock"),
                                                  ("false", "dock", "dock_fsm")])
def test_launch_dock_aside_moves_real_fsm_server(tmp_path, aside, present, absent):
    _need_launch()
    if shutil.which("ros2") is None:
        pytest.skip("ros2 not available")
    if subprocess.run(["ros2", "pkg", "prefix", "pode_fsm"], capture_output=True).returncode:
        pytest.skip("pode_fsm not installed")
    # 호출한 쪽이 도메인을 정한다(6, 12, 50~59는 쓰지 않는다)
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if (not domain.isdigit() or int(domain) in (6, 12) or 50 <= int(domain) <= 59
            or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"):
        pytest.skip("set ROS_DOMAIN_ID (not 6, 12, 50-59) and "
                    "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
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


def test_amcl_reads_the_drawing_fine_map():
    text = (KIT / "real_pode.launch.py").read_text(encoding="utf-8")
    line = next(t for t in text.splitlines() if '"map_fine"' in t and "DeclareLaunchArgument" in t)
    assert f'MAPS / "{AMCL_MAP}.yaml"' in line


def test_old_slam_map_file_is_kept():
    # 되돌릴 때 쓰도록 남긴다(map_fine:=.../arena_real_slam.yaml)
    from PIL import Image
    meta = _load(KIT / "maps" / f"{OLD_SLAM_MAP}.yaml")
    assert meta["image"] == f"{OLD_SLAM_MAP}.pgm" and meta["mode"] == "trinary"
    assert meta["resolution"] == 0.005 and meta["origin"] == [-0.1, -0.1, 0.0]
    # 205(미지)는 미지로 남아야 한다
    p_unknown = (255 - 205) / 255.0
    assert meta["free_thresh"] < p_unknown < meta["occupied_thresh"]
    img = Image.open(KIT / "maps" / meta["image"])
    assert img.size == (440, 240)
    px = img.load()

    def cell(x, y):  # 경기장 좌표 -> 픽셀 값
        return px[int((x + 0.1) / 0.005), 239 - int((y + 0.1) / 0.005)]
    # 네 벽은 직선: 안쪽 면 x 0.01 / 1.965, y 0.01 / 0.965
    for i in range(79):
        x = 0.0125 + i * 0.025
        assert cell(x, 0.0075) == 0 and cell(x, 0.9675) == 0
        assert cell(x, -0.0125) == 0 and cell(x, 0.9875) == 0
    for i in range(39):
        y = 0.0125 + i * 0.025
        assert cell(0.0075, y) == 0 and cell(1.9675, y) == 0
        assert cell(-0.0125, y) == 0 and cell(1.9875, y) == 0
    # 급행 통로 가운데와 남쪽 통로는 빈칸, 진열대 B 는 점유
    assert cell(1.0, 0.89) == 254 and cell(1.0, 0.2) == 254
    assert cell(0.83, 0.37) == 0  # 진열대 B 남쪽 면 (속은 미지)


# 옛 진열대 끝으로 그린 채 둘 키트 그림. 키트 지도를 다시 만들면 비운다
KIT_PENDING_TAPE_2026_11_23 = set()
OLD_SHELF_ENDS = (("  - [0.654, 0.754, 1.574, 0.794]", "  - [0.634, 0.754, 1.574, 0.794]"),
                  ("  - [0.704, 0.364, 1.014, 0.404]", "  - [0.684, 0.364, 1.014, 0.404]"))


def test_kit_maps_are_drawn_from_the_real_shelves(tmp_path):
    # 키트 지도 = python3 -m fleet_gz_sim.arena --layout v7 --real 의 결과(실측 진열대)
    pytest.importorskip("fleet_core")
    arena = pytest.importorskip("fleet_gz_sim.arena")
    from fleet_core.fleet_map import load_fleet_map
    from fleet_core.fleet_map.fleet_map import fleet_map_from_text
    arena.main(["--layout", "v7", "--real", "--out", str(tmp_path / "now")])
    old = load_fleet_map("v7").source_text
    for new, before in OLD_SHELF_ENDS:
        assert new in old, new  # --real은 shelves만 본다(sim_shelves 같은 줄도 함께 바뀐다)
        old = old.replace(new, before)
    before = fleet_map_from_text(old)
    arena.generate(tmp_path / "old", before, real=True)
    arena.generate_nav_variants(tmp_path / "old", before, real=True)
    for name in ("arena_v7", *MAPS, "arena_v7_a1"):
        for ext in (".yaml", ".pgm"):
            made = "old" if name + ext in KIT_PENDING_TAPE_2026_11_23 else "now"
            kit = KIT / "maps" / (name + ext)
            assert kit.read_bytes() == (tmp_path / made / "maps" / (name + ext)).read_bytes(), name
    # 대기 그림은 지금 진열대로 그린 것과 정말 다르다(같아지면 목록을 비운다)
    for name in KIT_PENDING_TAPE_2026_11_23:
        assert (KIT / "maps" / name).read_bytes() != (tmp_path / "now" / "maps" / name).read_bytes(), name


def test_kit_map_images_are_the_pinned_measured_maps():
    # 위 생성 비교는 fleet_gz_sim이 없으면 건너뛴다. 이 시험은 늘 돈다
    import hashlib
    names = sorted(path.name for path in (KIT / "maps").glob("*.pgm"))
    assert names == sorted(KIT_MAP_SHA256), "새 키트 지도는 KIT_MAP_SHA256에도 적는다"
    for name, digest in KIT_MAP_SHA256.items():
        assert hashlib.sha256((KIT / "maps" / name).read_bytes()).hexdigest() == digest, name


def test_kit_and_sim_maps_draw_shelf_a_north_face_at_the_measured_height():
    # 키트와 시뮬 1 cm 지도 모두 진열대 A 북쪽 면이 0.794다(x 1.0)
    from PIL import Image

    def cell(path, x, y):
        meta = _load(path)
        img = Image.open(path.parent / meta["image"])
        ox, oy = meta["origin"][:2]
        res = meta["resolution"]
        return img.getpixel((int((x - ox) / res), img.height - 1 - int((y - oy) / res)))
    for path in (KIT / "maps" / "arena_v7_fine.yaml",
                 ROOT / "sim" / "fleet_gz_sim" / "maps" / "arena_v7_fine.yaml"):
        assert cell(path, 1.0, 0.792) == 0 and cell(path, 1.0, 0.805) == 254, path


# ---- cmd_watchdog.py ----
def test_watchdog_rule_without_ros():
    import cmd_watchdog
    assert cmd_watchdog.WATCHDOG_S == pytest.approx(0.4)
    assert not cmd_watchdog.overdue(None, 100.0)
    assert not cmd_watchdog.overdue(100.0, 100.3)
    assert cmd_watchdog.overdue(100.0, 100.5)


# ---- 셸 스크립트 ----
SCRIPTS = ("start.sh", "stop.sh", "env.sh", "deploy.sh", "net.sh", "pc_env.sh",
           "discovery_server.sh")


@pytest.mark.parametrize("name", SCRIPTS)
def test_scripts_parse(name):
    assert subprocess.run(["bash", "-n", str(KIT / name)]).returncode == 0


def test_env_values():
    text = (KIT / "env.sh").read_text(encoding="utf-8")
    assert "net.sh" in text
    for path in ("/opt/ros/jazzy/setup.bash", "pinky_pro/install/local_setup.bash",
                 "nav2fleet_ws/install/local_setup.bash"):
        assert path in text
    # 도메인은 net.sh 한곳에만 둔다
    assert "export ROS_DOMAIN_ID=6" in (KIT / "net.sh").read_text(encoding="utf-8")
    assert "ROS_DOMAIN_ID" not in text


# ---- 통신 설정: net.sh(로봇과 PC 공통), env.sh, pc_env.sh, discovery_server.sh ----
NET_VARS = ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_AUTOMATIC_DISCOVERY_RANGE",
            "ROS_DISCOVERY_SERVER", "ROS_SUPER_CLIENT", "ROS_LOCALHOST_ONLY", "ROS_STATIC_PEERS")
DEFAULT_SERVER = "192.168.0.252:11811"
# 다른 설정이 남아 있는 셸에서 시작한다
POLLUTED = {"ROS_DOMAIN_ID": "12", "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST", "ROS_DISCOVERY_SERVER": "10.9.9.9:1",
            "ROS_SUPER_CLIENT": "TRUE", "ROS_LOCALHOST_ONLY": "1", "ROS_STATIC_PEERS": "10.9.9.8"}
SERVER_MODE = {"ROS_DOMAIN_ID": "6", "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
               "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET", "ROS_DISCOVERY_SERVER": DEFAULT_SERVER}
MULTICAST_MODE = {k: v for k, v in SERVER_MODE.items() if k != "ROS_DISCOVERY_SERVER"}
PRINT_NET = ('for v in ' + " ".join(NET_VARS)
             + '; do [ -z "${!v+x}" ] || echo "$v=${!v}"; done')


def _net_values(text):
    out = {}
    for line in text.splitlines():
        key, sep, value = line.partition("=")
        if sep and key in NET_VARS:
            out[key] = value
    return out


def _sourced(lines, extra=None, cwd=None):
    """다른 값이 남은 bash에서 줄들을 돌리고, 끝난 뒤의 통신 변수를 읽는다."""
    env = {"PATH": "/usr/bin:/bin", "HOME": "/nonexistent", **POLLUTED, **(extra or {})}
    run = subprocess.run(["bash", "-c", "\n".join(lines + [PRINT_NET])], env=env, cwd=cwd,
                         capture_output=True, text=True, timeout=30)
    return run, _net_values(run.stdout)


def test_net_default_is_server_mode_and_clears_old_values():
    run, got = _sourced([f'source "{KIT}/net.sh"'])
    assert run.returncode == 0, run.stderr
    assert got == SERVER_MODE


def test_net_other_server_address():
    run, got = _sourced([f'DISCOVERY_SERVER=10.0.0.5:12000 source "{KIT}/net.sh"'])
    assert got == {**SERVER_MODE, "ROS_DISCOVERY_SERVER": "10.0.0.5:12000"}


@pytest.mark.parametrize("extra", [{}, {"SUPER": "1"}])
def test_net_off_is_multicast_and_ignores_super(extra):
    run, got = _sourced([f'source "{KIT}/net.sh"'], {"DISCOVERY_SERVER": "off", **extra})
    assert run.returncode == 0, run.stderr
    assert got == MULTICAST_MODE


def test_net_super_client():
    run, got = _sourced([f'SUPER=1 source "{KIT}/net.sh"'])
    assert got == {**SERVER_MODE, "ROS_SUPER_CLIENT": "TRUE"}


def test_net_resourcing_super_normal_off_in_one_shell():
    run, got = _sourced([
        f'SUPER=1 source "{KIT}/net.sh"', 'echo "A $ROS_SUPER_CLIENT"',
        f'source "{KIT}/net.sh"', 'echo "B ${ROS_SUPER_CLIENT:-none} $ROS_DISCOVERY_SERVER"',
        f'DISCOVERY_SERVER=off source "{KIT}/net.sh"',
        'echo "C ${ROS_DISCOVERY_SERVER:-none} ${ROS_SUPER_CLIENT:-none}"',
        # 앞에 붙인 값은 셸에 남지 않는다
        'echo "D ${SUPER:-unset} ${DISCOVERY_SERVER:-unset}"'])
    out = run.stdout.splitlines()
    assert "A TRUE" in out
    assert f"B none {DEFAULT_SERVER}" in out
    assert "C none none" in out
    assert "D unset unset" in out
    assert got == MULTICAST_MODE


@pytest.mark.parametrize("nounset", [True, False])
def test_net_works_under_caller_nounset(nounset):
    run, got = _sourced((["set -u"] if nounset else []) + [f'source "{KIT}/net.sh"'])
    assert run.returncode == 0, run.stderr
    assert got == SERVER_MODE


def _robot_home(tmp_path):
    """로봇의 ~/pinky_pro, ~/nav2fleet_ws setup과 ROS setup 대역."""
    home = tmp_path / "home"
    for rel in ("pinky_pro/install/local_setup.bash", "nav2fleet_ws/install/local_setup.bash"):
        path = home / rel
        path.parent.mkdir(parents=True)
        path.write_text("")
    ros = tmp_path / "ros_setup.bash"
    ros.write_text("export ROS_STUB=1\n")
    return {"HOME": str(home), "ROS_SETUP": str(ros)}


def test_env_reads_net_next_to_itself(tmp_path):
    # 다른 폴더에서 source해도 옆의 net.sh를 읽는다(launch 설명의 사용법)
    run, got = _sourced([f'source "{KIT}/env.sh"'], _robot_home(tmp_path), cwd=tmp_path)
    assert run.returncode == 0, run.stdout + run.stderr
    assert got == SERVER_MODE


def _start_with_real_env(tmp_path, extra):
    """start.sh, env.sh, net.sh는 진짜. 대역 ros2가 받은 통신 변수를 적는다."""
    kit = tmp_path / "kit"
    kit.mkdir()
    for name in ("start.sh", "env.sh", "net.sh"):
        shutil.copy(KIT / name, kit)
    (kit / "make_params.py").write_text("print('params ok')\n")
    fake = tmp_path / "bin"
    fake.mkdir()
    seen = tmp_path / "ros2.env"
    (fake / "ros2").write_text(f'#!/bin/bash\n{PRINT_NET} > "{seen}"\n')
    (fake / "ros2").chmod(0o755)
    env = {"PATH": f"{fake}:/usr/bin:/bin", **POLLUTED, **_robot_home(tmp_path), **extra}
    run = subprocess.run(["bash", str(kit / "start.sh"), "pode1", "0.12", "0.10", "1.5708"],
                         env=env, capture_output=True, text=True, timeout=30)
    for _ in range(50):
        if seen.exists() and seen.read_text().strip():
            break
        time.sleep(0.1)
    return run, _net_values(seen.read_text() if seen.exists() else "")


def test_start_launch_gets_server_mode(tmp_path):
    run, got = _start_with_real_env(tmp_path, {})
    assert run.returncode == 0, run.stdout + run.stderr
    assert got == SERVER_MODE


def test_start_launch_gets_multicast_when_off(tmp_path):
    run, got = _start_with_real_env(tmp_path, {"DISCOVERY_SERVER": "off"})
    assert run.returncode == 0, run.stdout + run.stderr
    assert got == MULTICAST_MODE


def _pc_setups(tmp_path, workspace=True):
    """PC의 ROS setup과 작업공간 setup 대역. 진짜 setup처럼 정의되지 않은 변수를 읽는다."""
    ros = tmp_path / "ros_setup.bash"
    ros.write_text(': "${UNDEFINED_IN_TEST}"\nexport ROS_STUB=1\n')
    ws = tmp_path / "ws_setup.bash"
    if workspace:
        # setup이 셸 옵션을 바꿔도 pc_env.sh가 원래대로 돌린다
        ws.write_text('export STUB_MARK=overlay\nset -u\n')
    return {"ROS_SETUP": str(ros), "NAV2FLEET_WS_SETUP": str(ws)}


@pytest.mark.parametrize("nounset", [True, False])
def test_pc_env_reads_setups_and_keeps_caller_nounset(tmp_path, nounset):
    run, got = _sourced((["set -u"] if nounset else []) + [
        f'source "{KIT}/pc_env.sh"', 'echo "MARK=${STUB_MARK:-none}"',
        'case $- in *u*) echo NOUNSET_ON ;; *) echo NOUNSET_OFF ;; esac'], _pc_setups(tmp_path))
    assert run.returncode == 0, run.stderr
    assert "MARK=overlay" in run.stdout
    assert ("NOUNSET_ON" if nounset else "NOUNSET_OFF") in run.stdout
    assert got == SERVER_MODE


def test_pc_env_super_and_off(tmp_path):
    setups = _pc_setups(tmp_path)
    run, got = _sourced([f'SUPER=1 source "{KIT}/pc_env.sh"'], setups)
    assert got == {**SERVER_MODE, "ROS_SUPER_CLIENT": "TRUE"}
    run, got = _sourced([f'SUPER=1 DISCOVERY_SERVER=off source "{KIT}/pc_env.sh"'], setups)
    assert got == MULTICAST_MODE


def test_pc_env_missing_workspace_warns_and_still_sets(tmp_path):
    setups = _pc_setups(tmp_path, workspace=False)
    run, got = _sourced([f'source "{KIT}/pc_env.sh"', 'echo "RC=$?"'], setups)
    assert "RC=0" in run.stdout
    assert setups["NAV2FLEET_WS_SETUP"] in run.stderr
    assert got == SERVER_MODE


NOUNSET_STATE = 'case $- in *u*) echo NOUNSET_ON ;; *) echo NOUNSET_OFF ;; esac'


@pytest.mark.parametrize("nounset", [True, False])
def test_pc_env_missing_ros_returns_without_closing_the_shell(tmp_path, nounset):
    run, got = _sourced((["set -u"] if nounset else []) + [
        f'source "{KIT}/pc_env.sh"', 'echo "RC=$?"', NOUNSET_STATE,
        'compgen -v _pc_ || echo NO_HELPERS'], {"ROS_SETUP": str(tmp_path / "nope.bash")})
    assert "RC=1" in run.stdout
    assert ("NOUNSET_ON" if nounset else "NOUNSET_OFF") in run.stdout
    assert "NO_HELPERS" in run.stdout
    assert str(tmp_path / "nope.bash") in run.stderr


@pytest.mark.parametrize("broken", ["ros", "workspace"])
@pytest.mark.parametrize("nounset", [True, False])
def test_pc_env_failing_setup_returns_1(tmp_path, broken, nounset):
    # setup 파일은 있는데 실패하면 성공처럼 넘어가지 않는다
    setups = _pc_setups(tmp_path)
    # 실패하면서 nounset까지 켜는 setup
    Path(setups["ROS_SETUP" if broken == "ros" else "NAV2FLEET_WS_SETUP"]).write_text(
        "set -u\nreturn 3\n")
    run, got = _sourced((["set -u"] if nounset else []) + [
        f'source "{KIT}/pc_env.sh"', 'echo "RC=$?"', NOUNSET_STATE,
        'compgen -v _pc_ || echo NO_HELPERS'], setups)
    assert "RC=1" in run.stdout
    assert ("NOUNSET_ON" if nounset else "NOUNSET_OFF") in run.stdout
    assert "NO_HELPERS" in run.stdout
    assert "pc_env.sh" in run.stderr


@pytest.mark.parametrize("net", [None, "set -u\nreturn 2\n"])
@pytest.mark.parametrize("nounset", [True, False])
def test_pc_env_net_failure_returns_1(tmp_path, net, nounset):
    # net.sh가 없거나, 있는데 실패하는 경우
    kit = tmp_path / "kit"
    kit.mkdir()
    shutil.copy(KIT / "pc_env.sh", kit)
    if net is not None:
        (kit / "net.sh").write_text(net)
    setups = _pc_setups(tmp_path)
    # 작업공간 대역은 nounset을 건드리지 않게 해, net.sh가 켠 것만 남는지 본다
    Path(setups["NAV2FLEET_WS_SETUP"]).write_text("export STUB_MARK=overlay\n")
    run, got = _sourced((["set -u"] if nounset else []) + [
        f'source "{kit}/pc_env.sh"', 'echo "RC=$?"', NOUNSET_STATE,
        'compgen -v _pc_ || echo NO_HELPERS'], setups)
    assert "RC=1" in run.stdout
    assert ("NOUNSET_ON" if nounset else "NOUNSET_OFF") in run.stdout
    assert "NO_HELPERS" in run.stdout
    assert "net.sh" in run.stderr


def test_pc_env_leaves_no_helper_variables(tmp_path):
    run, got = _sourced([f'source "{KIT}/pc_env.sh"', 'compgen -v _pc_ || echo NONE'],
                        _pc_setups(tmp_path))
    assert run.stdout.splitlines()[0] == "NONE"


def test_discovery_server_runs_fastdds_in_front(tmp_path):
    fake = tmp_path / "bin"
    fake.mkdir()
    seen = tmp_path / "fastdds.args"
    # 대역 fastdds: 자기 PID, 서버 변수, 인자를 적는다
    (fake / "fastdds").write_text(
        '#!/bin/bash\n{ echo "PID=$$"; echo "DS=${ROS_DISCOVERY_SERVER-unset}"; '
        'echo "SUPER=${ROS_SUPER_CLIENT-unset}"; printf "%s\\n" "$@"; } > "' + str(seen) + '"\n')
    (fake / "fastdds").chmod(0o755)
    ros = tmp_path / "ros_setup.bash"
    # 진짜 setup처럼 정의되지 않은 변수를 읽고, 서버 변수를 export해도 서버에는 가지 않는다
    ros.write_text(': "${UNDEFINED_IN_TEST}"\nexport ROS_DISCOVERY_SERVER=10.9.9.7:1\n')
    env = {"PATH": f"{fake}:/usr/bin:/bin", "HOME": str(tmp_path), "ROS_SETUP": str(ros),
           "ROS_DISCOVERY_SERVER": "10.9.9.9:1", "ROS_SUPER_CLIENT": "TRUE"}
    proc = subprocess.Popen(["bash", str(KIT / "discovery_server.sh")], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out, err = proc.communicate(timeout=10)
    assert proc.returncode == 0, err
    lines = seen.read_text().splitlines()
    # exec: 스크립트 프로세스가 그대로 fastdds가 된다(터미널의 Ctrl+C가 바로 간다)
    assert lines[0] == f"PID={proc.pid}"
    assert lines[1:3] == ["DS=unset", "SUPER=unset"]
    # 포트는 net.sh 기본 주소의 포트와 같다
    port = DEFAULT_SERVER.split(":")[1]
    assert lines[3:] == ["discovery", "-i", "0", "-p", port]


def test_discovery_server_without_ros(tmp_path):
    env = {"PATH": "/usr/bin:/bin", "HOME": str(tmp_path), "ROS_SETUP": str(tmp_path / "nope")}
    run = subprocess.run(["bash", str(KIT / "discovery_server.sh")], env=env,
                         capture_output=True, text=True, timeout=10)
    assert run.returncode == 1


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
    (tmp_path / "make_params.py").write_text(
        "import sys\nprint('params ok', *sys.argv[1:])\n")
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


def _start_with_fake_ros2(tmp_path, peer_robots, dock_stub=None, fixed_line=None):
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
    env.pop("FIXED_LINE_CHECK", None)
    if fixed_line is not None:
        env["FIXED_LINE_CHECK"] = fixed_line
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


# ---- FIXED_LINE_CHECK ----
@pytest.mark.parametrize("value,flag", [(None, False), ("", False), ("0", False), ("1", True)])
def test_start_fixed_line_check_passes_flag(tmp_path, value, flag):
    run, argv = _start_with_fake_ros2(tmp_path, None, None, value)
    assert run.returncode == 0, run.stdout + run.stderr
    assert ("--fixed-line-check" in run.stdout) is flag
    assert argv == OLD_ARGV  # launch 인자는 그대로


@pytest.mark.parametrize("value", ["yes", "true", "2", "1;rm", " 1"])
def test_start_refuses_bad_fixed_line_check(tmp_path, value):
    run, argv = _start_with_fake_ros2(tmp_path, None, None, value)
    assert run.returncode == 2
    assert "FIXED_LINE_CHECK" in run.stdout + run.stderr
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
            "pode/pode_dock", "pode/pode_plan_keep")


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



# ---- NO_MOTION: 동작 없음 pode_fsm만 띄운다(설계 4.5, 2026-11-24 결정) ----
def _no_motion_start(tmp_path, args, **env_vars):
    start = _stub_kit(tmp_path)
    (tmp_path / "env.sh").write_text("return 0\n")
    fake = tmp_path / "bin"
    fake.mkdir()
    (fake / "ros2").write_text(f'#!/bin/bash\nprintf "%s\\n" "$@" > "{tmp_path}/ros2.args"\n')
    (fake / "ros2").chmod(0o755)
    env = dict(os.environ, PATH=f"{fake}:{os.environ['PATH']}")
    for key in ("PEER_ROBOTS", "DOCK_STUB", "FIXED_LINE_CHECK", "NO_MOTION", "PLAN_KEEP"):
        env.pop(key, None)
    env.update(env_vars)
    run = subprocess.run(["bash", str(start), *args], capture_output=True, text=True, env=env,
                         timeout=30)
    seen = tmp_path / "ros2.args"
    for _ in range(50):
        if seen.exists() and seen.read_text().strip():
            break
        time.sleep(0.1)
    return run, (seen.read_text().splitlines() if seen.exists() else [])


def _no_motion_argv(ns, x, y, yaw):
    return ["launch", "pode_fsm", "pode_fsm.launch.py", f"namespace:={ns}", "no_motion:=true",
            f"initial_x:={x}", f"initial_y:={y}", f"initial_yaw:={yaw}"]


def _parking(name):
    site = yaml.safe_load((ROOT / "pc" / "fleet_core" / "fleet_core" / "data"
                           / "fleet_map_v7.yaml").read_text())
    node = site["nodes"][name]
    return node["x"], node["y"], math.radians(node["yaw_deg"])


@pytest.mark.parametrize("ns,parking", [("pode1", "P-1"), ("pode2", "P-2"), ("pode3", "P-3")])
def test_no_motion_start_uses_the_robots_parking_spot(tmp_path, ns, parking):
    run, argv = _no_motion_start(tmp_path, [ns], NO_MOTION="1")
    assert run.returncode == 0, run.stdout + run.stderr
    assert argv[:5] == _no_motion_argv(ns, 0, 0, 0)[:5]
    assert [float(a.split(":=")[1]) for a in argv[5:]] == pytest.approx(_parking(parking),
                                                                        abs=1e-3)
    assert "params ok" not in run.stdout  # 속도 사본은 만들지 않는다


def test_no_motion_start_takes_an_explicit_pose(tmp_path):
    run, argv = _no_motion_start(tmp_path, ["pode2", "1.0", "0.5", "0"], NO_MOTION="1")
    assert run.returncode == 0, run.stdout + run.stderr
    assert argv == _no_motion_argv("pode2", "1.0", "0.5", "0")


@pytest.mark.parametrize("args", [[], ["pode9"], ["pode1", "0.1"],
                                  ["pode1", "0.12", "0.10", "1.5708", "0.12"],
                                  ["pode1", "bad", "0", "0"], ["pode1", "0.1", "1e", "0"],
                                  ["pode1", "0.1", "0.2", ""], ["pode1", "0.1", "0.2", "--"]])
def test_no_motion_start_refuses_bad_arguments(tmp_path, args):
    run, argv = _no_motion_start(tmp_path, args, NO_MOTION="1")
    assert run.returncode == 2 and argv == []


@pytest.mark.parametrize("extra", [{"DOCK_STUB": "1"}, {"PEER_ROBOTS": "pode1,pode2"},
                                   {"PEER_ROBOTS": "0"}, {"FIXED_LINE_CHECK": "1"},
                                   {"PLAN_KEEP": "1"}])
def test_no_motion_start_refuses_real_drive_options(tmp_path, extra):
    run, argv = _no_motion_start(tmp_path, ["pode1"], NO_MOTION="1", **extra)
    assert run.returncode == 2 and argv == []
    assert "NO_MOTION" in run.stdout + run.stderr


@pytest.mark.parametrize("value", ["2", "yes", "true"])
def test_start_refuses_bad_no_motion(tmp_path, value):
    run, argv = _no_motion_start(tmp_path, ["pode1", "0.12", "0.10", "1.5708"], NO_MOTION=value)
    assert run.returncode == 2 and argv == []
    assert "NO_MOTION" in run.stdout + run.stderr


@pytest.mark.parametrize("value", ["", "0"])
def test_start_no_motion_off_keeps_old_argv(tmp_path, value):
    run, argv = _no_motion_start(tmp_path, ["pode1", "0.12", "0.10", "1.5708"], NO_MOTION=value)
    assert run.returncode == 0, run.stdout + run.stderr
    assert argv == OLD_ARGV


@pytest.mark.parametrize("value", [{"DOCK_STUB": "0"}, {"FIXED_LINE_CHECK": "0"},
                                   {"PEER_ROBOTS": ""}])
def test_no_motion_start_accepts_off_values(tmp_path, value):
    run, argv = _no_motion_start(tmp_path, ["pode1", "-0.5", "+1", ".25"], NO_MOTION="1", **value)
    assert run.returncode == 0, run.stdout + run.stderr
    assert argv == _no_motion_argv("pode1", "-0.5", "+1", ".25")


def test_stop_refuses_a_pode_fsm_launch_that_start_sh_did_not_start(tmp_path):
    # 오래된 pid 번호가 손으로 띄운 pode_fsm launch에 다시 쓰였을 때
    shutil.copy(KIT / "stop.sh", tmp_path)
    proc = _own_group(tmp_path, sys.executable, "-c", "import time; time.sleep(30)",
                      "pode_fsm.launch.py", "namespace:=pode1")
    try:
        run = subprocess.run(["bash", str(tmp_path / "stop.sh")], capture_output=True, text=True)
        assert run.returncode == 1, run.stdout + run.stderr
        assert proc.poll() is None
    finally:
        proc.kill()
        proc.wait()


def test_stop_stops_a_no_motion_group(tmp_path):
    shutil.copy(KIT / "stop.sh", tmp_path)
    (tmp_path / "logs").mkdir()
    pid = int(subprocess.run(
        ["bash", "-c", f"setsid {sys.executable} -c 'import time; time.sleep(30)' "
                       "pode_fsm pode_fsm.launch.py namespace:=pode1 no_motion:=true "
                       "initial_x:=0.12 > /dev/null 2>&1 & echo $!"],
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
