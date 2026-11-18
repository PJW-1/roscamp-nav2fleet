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
