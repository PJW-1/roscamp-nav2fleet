"""CI 5번 항목: 시나리오마다 끝까지 돌고, 실행 중인 그래프가 contract.yaml과 맞는다."""
import csv
import json
import os
import signal
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")

from ament_index_python.packages import (  # noqa: E402
    PackageNotFoundError,
    get_package_share_directory,
)

SCENARIOS = ["basic_orders", "order_burst", "shared_lane_failure", "stock_shortage",
             "stuck_robot", "operator_recovery", "blocked_lane", "auto_isolation", "reinstate"]
WALL_TIMEOUT_S = 420
FIRST_DOMAIN = 50  # 시나리오마다 도메인 하나, 로봇 네트워크와 겹치지 않는 범위
CHECKER = Path(__file__).parents[3] / "scripts" / "check_interfaces.py"
REPLAY = Path(__file__).parents[3] / "tools" / "replay.py"
# 구역을 몸이 아직 걸친 채 놓는 것(R1)은 다른 판정과 똑같이 실패다. 놓는 거리가 0.10 m이던 때에는
# 보고 잡음에 따라 가끔 나와서 그 판정만 틀린 실행을 건너뛰었다. 몸 전체가 나간 뒤에 놓게 바꾼
# 뒤로는(0.135 m, 추가 지시서 2의 5절) 건너뛰지 않는다


def unexplained_safety(summary: dict, safety: dict) -> tuple[int, int]:
    """safety.json의 (early, near) 가운데 심판의 R1 판정으로 설명되지 않는 것.

    심판과 SafetyMeter는 같은 놓기를 같은 실제 자세로 본다: 자세를 아는 로봇의 R1 한 건은 early 한
    건으로도 잡힌다. 심판은 자세를 모르는 로봇의 놓기와 거리가 반지름과 꼭 같은 놓기도 R1로 세므로,
    R1을 기대값으로 적은 시나리오에서는 이 차감이 early를 덜 셀 수 있다. 지금 그런 시나리오는 없다:
    R1이 0이어야 통과한다.
    """
    r1 = summary["checks"]["released_without_position_check"]["value"]
    return max(0, safety["early"] - r1), safety["near"]


def test_an_early_release_the_judge_called_r1_is_not_counted_again():
    summary = {"checks": {"released_without_position_check": {"value": 1}}}
    assert unexplained_safety(summary, {"early": 1, "near": 0}) == (0, 0)


def test_an_early_release_the_judge_did_not_see_and_any_near_miss_still_count():
    summary = {"checks": {"released_without_position_check": {"value": 1}}}
    assert unexplained_safety(summary, {"early": 2, "near": 0}) == (1, 0)
    assert unexplained_safety(summary, {"early": 1, "near": 1}) == (0, 1)
    clean = {"checks": {"released_without_position_check": {"value": 0}}}
    assert unexplained_safety(clean, {"early": 1, "near": 0}) == (1, 0)


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_scenario_finishes_and_matches_the_contract(scenario, tmp_path):
    try:
        for package in ("iponic_bringup", "fake_robots"):
            get_package_share_directory(package)
    except PackageNotFoundError as exc:
        pytest.skip(f"not built in this workspace: {exc}")
    env = dict(os.environ, ROS_DOMAIN_ID=str(FIRST_DOMAIN + SCENARIOS.index(scenario)),
               ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    log_path = tmp_path / "launch.log"
    with log_path.open("w", encoding="utf-8") as log:
        launch = subprocess.Popen(
            ["ros2", "launch", "iponic_bringup", "pc.launch.py", "use_fake:=true",
             f"scenario:={scenario}", f"result_dir:={tmp_path}",
             f"record:={tmp_path / 'fms.jsonl'}"],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        try:
            check = subprocess.run(
                [sys.executable, str(CHECKER), "--mode", "fake", "--wait-s", "15"],
                env=env, capture_output=True, text=True, timeout=120,
            )
            launch.wait(timeout=WALL_TIMEOUT_S)
        finally:
            if launch.poll() is None:
                os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)

    tail = log_path.read_text(encoding="utf-8")[-3000:]
    assert check.returncode == 0, f"contract check failed:\n{check.stdout}\n{check.stderr}"
    summary_path = tmp_path / "summary.json"
    assert summary_path.exists(), f"no summary was written. launch log tail:\n{tail}"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["pass"], f"{summary}\nlaunch log tail:\n{tail}"
    assert summary["orders_shipped"] >= 1, "a scenario must not pass by shipping nothing"
    safety = json.loads((tmp_path / "safety.json").read_text(encoding="utf-8"))
    assert unexplained_safety(summary, safety) == (0, 0), safety
    if scenario == "shared_lane_failure":
        with (tmp_path / "fleet_robots.csv").open(encoding="utf-8") as f:
            seen = [row["fault"] for row in csv.DictReader(f) if row["robot"] == "pinky2"]
        assert seen[0] == "NONE" and seen[-1] == "NONE", seen
        assert "COMM_LOST" in seen, f"the comm loss never showed on /fleet/robots: {seen}"
    played = subprocess.run([sys.executable, str(REPLAY), str(tmp_path / "fms.jsonl")],
                            capture_output=True, text=True, timeout=120)
    print(f"{scenario}: {played.stdout}")
    assert played.returncode == 0, played.stdout + played.stderr
    assert "same as recorded" in played.stdout
