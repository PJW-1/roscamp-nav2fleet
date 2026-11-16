"""실물 핑키 한 대 실행 묶음(tools/real_pinky): 속도 파라미터, launch 인자 이름, 셸 스크립트."""
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
KIT = ROOT / "tools" / "real_pinky"
NAV2_SRC = ROOT / "pinky" / "pinky_nav_config" / "params" / "nav2_params.yaml"
AVOID_SRC = ROOT / "pinky" / "pinky_avoidance" / "params" / "avoidance.yaml"
NAV2_LAUNCH = ROOT / "pinky" / "pinky_nav_config" / "launch" / "nav2.launch.py"
FSM_LAUNCH = ROOT / "pinky" / "pinky_fsm" / "launch" / "pinky_fsm.launch.py"
FORWARD = ("FollowPath", "FollowPathThrough")
MAPS = ("arena_v7", "arena_v7_fine", "arena_v7_keepout")

sys.path.insert(0, str(KIT))


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def _make_params():
    import make_params
    return make_params


# ---- make_params.py ----
@pytest.mark.parametrize("speed", [0.0, -0.1, 0.26, 0.25001, 0.00001, 1.0, float("nan"), float("inf")])
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
        changed = [i for i, (a, b) in enumerate(zip(got, orig)) if a != b]
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


# ---- real_pinky.launch.py: 지금 패키지의 인자 이름 ----
def _declared(path):
    return set(re.findall(r'DeclareLaunchArgument\(\s*"([a-z_]+)"', path.read_text("utf-8")))


def _constant(name):
    tree = ast.parse((KIT / "real_pinky.launch.py").read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} not in real_pinky.launch.py")


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


def test_launch_drops_dock_stand_in_and_keeps_shutdown():
    text = (KIT / "real_pinky.launch.py").read_text(encoding="utf-8")
    assert "dock_aside" not in text and "dock_fsm" not in text
    assert "OnProcessExit" in text and "Shutdown(" in text
    assert "cmd_watchdog.py" in text
    assert "bringup_robot.launch.xml" in text
    for name in MAPS:
        assert f"{name}.yaml" in text


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
    run = subprocess.run(["bash", str(KIT / "start.sh"), "pinky1", "0.1"],
                         capture_output=True, text=True)
    assert run.returncode == 2
    before = sorted(p.name for p in KIT.iterdir() if p.name != "logs")
    run = subprocess.run(["bash", str(KIT / "start.sh"), "pinky1", "0.12", "0.10", "1.5708",
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
        run = subprocess.run(["bash", str(start), "pinky1", "0.12", "0.10", "1.5708"],
                             capture_output=True, text=True, timeout=10)
    assert run.returncode == 1
    assert "another start.sh" in run.stdout and "ENV_REACHED" not in run.stdout
    assert "params ok" not in run.stdout  # 잠금보다 먼저 사본을 바꾸지 않는다
    # 잠금이 풀리면 env까지 간다(대역 env가 거기서 멈춘다)
    run = subprocess.run(["bash", str(start), "pinky1", "0.12", "0.10", "1.5708"],
                         capture_output=True, text=True, timeout=10)
    assert run.returncode == 1 and "ENV_REACHED" in run.stdout


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
                       "real_pinky.launch.py > /dev/null 2>&1 & echo $!"],
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


PACKAGES = ("interfaces/fleet_interfaces", "pinky/pinky_fsm", "pinky/pinky_nav_config",
            "pinky/pinky_avoidance", "pinky/pinky_peer_obstacles", "pinky/pinky_scan_filter",
            "pinky/pinky_dock")


def _fake_repo(tmp_path):
    repo = tmp_path / "repo"
    for pkg in PACKAGES:
        (repo / pkg).mkdir(parents=True)
        (repo / pkg / "package.xml").write_text(pkg)
    (repo / "pinky" / "other_pkg").mkdir()
    (repo / "pinky" / "other_pkg" / "package.xml").write_text("x")
    shutil.copytree(KIT, repo / "tools" / "real_pinky",
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
    run = subprocess.run(["bash", str(repo / "tools" / "real_pinky" / "deploy.sh"),
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
    for c, d in zip(rsyncs, dests):
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


@pytest.mark.parametrize("name", ["real_pinky.launch.py", "make_params.py", "cmd_watchdog.py",
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

