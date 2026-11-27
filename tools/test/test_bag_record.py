"""tools/real_pode/bag_record.sh 시험. 가짜 ros2로 녹화기 시작, 실패, 멈춤을 본다."""
import fcntl
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
KIT = ROOT / "tools" / "real_pode"

# 가짜 ros2: FAKE_MODE에 따라 bag을 열고 SIGINT에 마무리하거나, 바로 죽거나, bag을 안 연다
FAKE_ROS2 = r"""#!/bin/bash
printf "%s\n" "$@" > "$FAKE_DIR/ros2.args"
out=
while [ $# -gt 0 ]; do [ "$1" = -o ] && out=$2; shift; done
case $FAKE_MODE in
  fail) exit 1 ;;
  slow) while true; do sleep 0.1; done ;;
esac
mkdir -p "$out" && : > "$out/$(basename "$out")_0.mcap"
trap 'echo done > "$out/metadata.yaml"; exit 0' INT
while true; do sleep 0.1; done
"""


def _kit(tmp_path, mode="ok"):
    kit = tmp_path / "kit"
    kit.mkdir()
    shutil.copy(KIT / "bag_record.sh", kit)
    (kit / "env.sh").write_text('[ "$SUPER" = 1 ] || { echo "SUPER not set"; return 1; }\nreturn 0\n')
    fake = tmp_path / "bin"
    fake.mkdir()
    (fake / "ros2").write_text(FAKE_ROS2)
    (fake / "ros2").chmod(0o755)
    env = dict(os.environ, PATH=f"{fake}:{os.environ['PATH']}", FAKE_MODE=mode,
               FAKE_DIR=str(tmp_path), BAG_START_TIMEOUT_S="2")
    env.pop("SUPER", None)
    return kit, env


def _run(kit, env, *args):
    return subprocess.run(["bash", str(kit / "bag_record.sh"), *args], capture_output=True,
                          text=True, env=env, timeout=60)


def _alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    try:
        with open(f"/proc/{pid}/stat") as f:
            return f.read().rsplit(")", 1)[1].split()[0] != "Z"
    except FileNotFoundError:  # 확인하는 사이에 사라졌다
        return False


def _pid(kit):
    return int((kit / "logs" / "bag.pid").read_text().split()[0])


def test_start_records_namespace_topics_and_stop_finalizes(tmp_path):
    kit, env = _kit(tmp_path)
    run = _run(kit, env, "start", "pode1", "run1")
    assert run.returncode == 0, run.stdout + run.stderr
    assert run.stderr == ""  # pid 파일이 없어도 오류 문구를 내지 않는다
    pid = _pid(kit)
    try:
        # start가 끝나도 녹화기는 자기 프로세스 그룹에서 계속 돈다
        assert _alive(pid) and os.getpgid(pid) == pid
        args = (tmp_path / "ros2.args").read_text().splitlines()
        assert args[:4] == ["bag", "record", "-o", "bags/run1"]
        assert args[4:] == ["-s", "mcap", "-e", "^/pode1/(scan|odom|cmd_vel|amcl_pose|particle_cloud|plan|plan_kept|"
                                  "tf|tf_static|state)$"]
        run = _run(kit, env, "stop")
        assert run.returncode == 0, run.stdout + run.stderr
        assert (kit / "bags" / "run1" / "metadata.yaml").read_text().strip() == "done"
        assert not _alive(pid)
        assert not (kit / "logs" / "bag.pid").exists()
        run = _run(kit, env, "stop")
        assert run.returncode == 0 and "not recording" in run.stdout and run.stderr == ""
    finally:
        if _alive(pid):
            os.kill(pid, 9)


def test_start_stops_the_recorder_when_the_pid_file_cannot_be_written(tmp_path):
    kit, env = _kit(tmp_path)
    (kit / "logs" / "bag.pid").mkdir(parents=True)
    run = _run(kit, env, "start", "pode1", "run1")
    assert run.returncode == 1, run.stdout + run.stderr
    found = re.search(r"cannot write logs/bag.pid; stopping recorder (\d+)", run.stdout)
    assert found, run.stdout + run.stderr
    assert not _alive(int(found.group(1)))


def test_start_fails_and_cleans_up_when_recorder_dies(tmp_path):
    kit, env = _kit(tmp_path, "fail")
    run = _run(kit, env, "start", "pode1", "run1")
    assert run.returncode == 1 and "did not start" in run.stdout
    assert not (kit / "logs" / "bag.pid").exists()
    assert not (kit / "bags" / "run1").exists()  # 같은 이름으로 다시 시작할 수 있다


def test_start_stops_a_recorder_that_never_opens_the_bag(tmp_path):
    kit, env = _kit(tmp_path, "slow")
    t0 = time.monotonic()
    run = _run(kit, env, "start", "pode1", "run1")
    assert run.returncode == 1 and "did not start" in run.stdout
    assert time.monotonic() - t0 < 10
    out = subprocess.run(["pgrep", "-f", f"{tmp_path}/bin/ros2"], capture_output=True, text=True)
    assert out.stdout.strip() == ""  # 남은 녹화기가 없다
    assert not (kit / "logs" / "bag.pid").exists()


def test_start_refuses_while_another_call_holds_the_lock(tmp_path):
    kit, env = _kit(tmp_path)
    (kit / "logs").mkdir()
    with open(kit / "logs" / "bag.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        run = _run(kit, env, "start", "pode1", "run1")
    assert run.returncode == 1 and "another bag_record.sh" in run.stdout
    assert not (tmp_path / "ros2.args").exists()


def test_start_refuses_while_recording_and_existing_name(tmp_path):
    kit, env = _kit(tmp_path)
    assert _run(kit, env, "start", "pode1", "run1").returncode == 0
    pid = _pid(kit)
    try:
        run = _run(kit, env, "start", "pode1", "run2")
        assert run.returncode == 1 and "already recording" in run.stdout
        assert _run(kit, env, "stop").returncode == 0
        run = _run(kit, env, "start", "pode1", "run1")
        assert run.returncode == 1 and "exists" in run.stdout
    finally:
        if _alive(pid):
            os.kill(pid, 9)


def test_stop_leaves_a_reused_pid_alone(tmp_path):
    # pid 파일의 번호가 다른 프로세스에 다시 쓰였다: 명령줄에 bag record가 있어도 우리 bag이 아니면 건드리지 않는다
    kit, env = _kit(tmp_path)
    (kit / "logs").mkdir()
    other = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)",
                              "bag", "record", "-o", "bags/other"], start_new_session=True)
    try:
        (kit / "logs" / "bag.pid").write_text(f"{other.pid} run1\n")
        run = _run(kit, env, "stop")
        assert run.returncode == 0 and "not recording" in run.stdout
        assert other.poll() is None
        run = _run(kit, env, "start", "pode1", "run1")
        assert run.returncode == 0, run.stdout + run.stderr
        assert _run(kit, env, "stop").returncode == 0
        assert other.poll() is None
    finally:
        other.kill()
        other.wait()


def test_usage_and_bad_names(tmp_path):
    kit, env = _kit(tmp_path)
    for args in ([], ["start", "pode1"], ["start", "Pode-1", "run1"], ["start", "pode1", "a/b"],
                 ["go"]):
        run = _run(kit, env, *args)
        assert run.returncode == 2, args
    assert not (tmp_path / "ros2.args").exists()


# 0.5 s 동안 명령 인자에 bags/NAME이 따로 보이지 않다가 진짜 setsid가 된다(exec 전 상태 흉내)
FAKE_SETSID = r'''#!/usr/bin/env python3
import os, sys
os.execv(sys.executable, [sys.executable, "-c",
    "import os, sys, time; time.sleep(0.5); os.execv(sys.argv[1], ['setsid'] + sys.argv[2].split('\x01'))",
    REAL, "\x01".join(sys.argv[1:])])
'''


def test_start_keeps_a_recorder_it_cannot_recognise_yet(tmp_path):
    real = shutil.which("setsid")
    kit, env = _kit(tmp_path)
    (tmp_path / "bin" / "setsid").write_text(FAKE_SETSID.replace("REAL", repr(real)))
    (tmp_path / "bin" / "setsid").chmod(0o755)
    run = _run(kit, env, "start", "pode1", "run1")
    pid = None
    try:
        assert run.returncode == 0, run.stdout + run.stderr
        pid = _pid(kit)
        assert _run(kit, env, "stop").returncode == 0
        assert (kit / "bags" / "run1" / "metadata.yaml").exists()
    finally:
        time.sleep(1)  # 실패했을 때 늦게 뜬 녹화기까지 정리한다
        subprocess.run(["pkill", "-f", f"{tmp_path}/bin/ros2"])
    out = subprocess.run(["pgrep", "-f", f"{tmp_path}/bin/ros2"], capture_output=True, text=True)
    assert out.stdout.strip() == ""


def test_stop_reports_a_recorder_that_died_while_recording(tmp_path):
    kit, env = _kit(tmp_path)
    assert _run(kit, env, "start", "pode1", "run1").returncode == 0
    pid = _pid(kit)
    os.kill(pid, 9)  # SIGKILL: bag을 마무리하지 못한다
    for _ in range(50):
        if not _alive(pid):
            break
        time.sleep(0.1)
    run = _run(kit, env, "stop")
    assert run.returncode == 1 and "reindex" in run.stdout, run.stdout
    assert not (kit / "logs" / "bag.pid").exists()
