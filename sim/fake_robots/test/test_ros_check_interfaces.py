"""떠 있는 가짜 노드에 scripts/check_interfaces.py를 돌린다."""
import subprocess
import sys

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import CONTRACT, REPO, write_scenario  # noqa: E402

SCRIPT = REPO / "scripts" / "check_interfaces.py"
# 가짜가 아닌 제공자: 플릿 매니저 몫이다.
SKIP = "assignments,zones,robots,heartbeat,confirm_clear,reinstate,force_ship"


def start_all(harness, tmp_path):
    from fake_robots.fake_arm_node import FakeArmNode
    from fake_robots.fake_central_node import FakeCentralNode
    from fake_robots.fake_pode_node import FakePodeNode
    from fake_robots.fake_workcell_node import FakeWorkcellNode
    scn = write_scenario(tmp_path)
    for r in ("pode1", "pode2", "pode3"):
        harness.add(FakePodeNode(parameter_overrides=harness.params(
            robot_id=r, scenario_file=scn)))
    harness.add(FakeWorkcellNode(parameter_overrides=harness.params(scenario_file=scn)))
    harness.add(FakeArmNode(parameter_overrides=harness.params(
        arm_id="arm_ship", scenario_file=scn)))
    harness.add(FakeCentralNode(parameter_overrides=harness.params(scenario_file=scn)))


def check(*extra):
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--contract", str(CONTRACT), "--mode", "fake",
         "--wait-s", "12", "--skip", SKIP, *extra],
        capture_output=True, text=True, timeout=60)


def test_fakes_satisfy_the_contract(harness, tmp_path):
    start_all(harness, tmp_path)
    out = check()
    assert out.returncode == 0, out.stdout + out.stderr


def test_wrong_qos_is_caught(harness, tmp_path):
    import rclpy.node
    from fleet_interfaces.msg import RobotState
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    start_all(harness, tmp_path)
    rogue = harness.add(rclpy.node.Node("rogue"))
    rogue.create_publisher(RobotState, "/pode1/state", QoSProfile(
        depth=5, reliability=ReliabilityPolicy.RELIABLE))
    out = check()
    assert out.returncode == 1, out.stdout + out.stderr
    assert "missing" not in out.stdout
    assert "/pode1/state" in out.stdout and "publisher reliability is reliable" in out.stdout


def test_missing_provider_is_reported(harness, tmp_path):
    out = check()
    assert out.returncode == 1 and "missing" in out.stdout
