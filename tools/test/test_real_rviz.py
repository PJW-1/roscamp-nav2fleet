"""실물 시험 RViz 설정: 로봇마다 토픽이 있고, 스크립트 문법이 맞다."""
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

KIT = Path(__file__).resolve().parents[2] / "tools" / "real_pode"
sys.path.insert(0, str(KIT))
import rviz_config  # noqa: E402

ROBOTS = ["pode1", "pode2", "pode3"]


def _topics(cfg):
    return {d["Topic"]["Value"]: d["Topic"] for d in cfg["Visualization Manager"]["Displays"]
            if "Topic" in d}


def test_overview_names_every_robot():
    cfg = yaml.safe_load(rviz_config.overview(ROBOTS))
    topics = _topics(cfg)
    for ns in ROBOTS:
        for t in ("amcl_pose", "received_global_plan", "particle_cloud"):
            assert f"/{ns}/{t}" in topics
    assert cfg["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"


def test_overview_map_is_transient_local():
    cfg = yaml.safe_load(rviz_config.overview(ROBOTS[:2]))
    assert _topics(cfg)["/pode1/map"]["Durability Policy"] == "Transient Local"
    assert "/pode3/amcl_pose" not in _topics(cfg)


@pytest.mark.parametrize("ns", ROBOTS)
def test_per_robot_config(ns):
    text = rviz_config.per_robot(ns)
    topics = _topics(yaml.safe_load(text))
    assert f"/{ns}/map" in topics and f"/{ns}/scan" in topics
    assert f"/{ns}/local_costmap/published_footprint" in topics
    assert "<robot_namespace>" not in text
    assert topics[f"/{ns}/map"]["Durability Policy"] == "Transient Local"


def test_script_syntax_and_args():
    sh = KIT / "rviz_pc.sh"
    assert subprocess.run(["bash", "-n", str(sh)]).returncode == 0
    assert subprocess.run(["bash", str(sh)], capture_output=True, text=True).returncode == 2
    text = sh.read_text()
    assert "ROS_DOMAIN_ID=12" in text and "SUBNET" in text


def _run_stubbed(tmp_path, ws_setup, ros_body="", home=None, unset_ws=False):
    """가짜 rviz2를 PATH 맨 앞에 두고 실제 스크립트를 돌려 호출 인자를 읽는다."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    log = tmp_path / "calls.log"
    fake = bindir / "rviz2"
    fake.write_text('#!/usr/bin/env bash\n{ printf \'%s\\n\' "---" "MARK=${STUB_MARK:-none}" "$@"; } '
                    f'>> {log}\nexit 0\n')
    fake.chmod(0o755)
    ros = tmp_path / "ros_setup.bash"
    ros.write_text(ros_body)
    env = {"PATH": f"{bindir}:/usr/bin:/bin", "HOME": str(home or tmp_path), "ROS_SETUP": str(ros)}
    if not unset_ws:
        env["NAV2FLEET_WS_SETUP"] = str(ws_setup)
    r = subprocess.run(["bash", str(KIT / "rviz_pc.sh"), "pode1", "pode2"], env=env,
                       capture_output=True, text=True, timeout=30)
    calls = []
    if log.exists():
        for blk in log.read_text().split("---\n")[1:]:
            calls.append(blk.splitlines())
    return r, calls


def _check_calls(calls, mark):
    assert len(calls) == 3
    assert all(c[0] == f"MARK={mark}" for c in calls)
    args = [c[1:] for c in calls]
    ov = [a for a in args if a[1].endswith("/overview.rviz")]
    assert len(ov) == 1
    assert ov[0][0] == "-d"
    assert ov[0][2:] == ["--ros-args", "-r", "/tf:=/pode1/tf", "-r", "/tf_static:=/pode1/tf_static"]
    for ns in ("pode1", "pode2"):
        per = [a for a in args if a[1].endswith(f"/{ns}.rviz")]
        assert len(per) == 1
        assert per[0][0] == "-d"
        assert per[0][2:] == ["--ros-args", "-r", f"/tf:=/{ns}/tf", "-r", f"/tf_static:=/{ns}/tf_static"]


def test_launcher_with_overlay(tmp_path):
    ws = tmp_path / "ws_setup.bash"
    ws.write_text("export STUB_MARK=overlay\n")
    r, calls = _run_stubbed(tmp_path, ws)
    assert r.returncode == 0, r.stderr
    _check_calls(calls, "overlay")


def test_launcher_overlay_missing_is_skipped(tmp_path):
    r, calls = _run_stubbed(tmp_path, tmp_path / "nope.bash")
    assert r.returncode == 0, r.stderr
    _check_calls(calls, "none")


def test_launcher_setup_files_may_read_unset_vars(tmp_path):
    """실제 ROS setup은 미설정 변수를 읽는다. set -u가 새면 실패해야 한다."""
    ws = tmp_path / "ws_setup.bash"
    ws.write_text('echo "${WS_UNSET_VAR}" >/dev/null\nexport STUB_MARK=overlay\n')
    r, calls = _run_stubbed(tmp_path, ws, ros_body='echo "${ROS_UNSET_VAR}" >/dev/null\n')
    assert r.returncode == 0, r.stderr
    _check_calls(calls, "overlay")


def test_launcher_default_overlay_path(tmp_path):
    home = tmp_path / "home"
    d = home / "ros2camp" / "nav2fleet_ws" / "install"
    d.mkdir(parents=True)
    (d / "setup.bash").write_text("export STUB_MARK=default_overlay\n")
    r, calls = _run_stubbed(tmp_path, None, home=home, unset_ws=True)
    assert r.returncode == 0, r.stderr
    _check_calls(calls, "default_overlay")
