"""실물 시험용 RViz 설정 생성. 로봇마다 TF 가 /<ns>/tf 에 따로 있어서 RViz 도 로봇마다 한 개 띄운다.

- 로봇별 창: pode_nav.rviz 의 <robot_namespace> 를 /<ns> 로 바꾼다(지도, 스캔, 경로, 발자국).
- 전체 창: TF 없이 map 틀 토픽만(지도, 로봇별 amcl_pose, 입자, 경로).
  스캔과 발자국은 TF 가 필요해 뺀다.
"""
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
NAV_RVIZ = ROOT / "pode" / "pode_nav_config" / "rviz" / "pode_nav.rviz"
COLORS = ("255; 60; 60", "60; 200; 60", "60; 120; 255", "255; 200; 0")


def per_robot(ns):
    return NAV_RVIZ.read_text(encoding="utf-8").replace("<robot_namespace>", "/" + ns)


def _qos(durability, reliability):
    return {"Depth": 5, "Durability Policy": durability, "History Policy": "Keep Last",
            "Reliability Policy": reliability}


def overview(robots, map_robot=None):
    map_robot = map_robot or robots[0]
    displays = [
        {"Class": "rviz_default_plugins/Grid", "Name": "Grid", "Enabled": True,
         "Cell Size": 0.5, "Plane Cell Count": 12, "Offset": {"X": 1.85, "Y": 1.3, "Z": 0},
         "Color": "160; 160; 164", "Alpha": 0.3, "Reference Frame": "map"},
        {"Class": "rviz_default_plugins/Map", "Name": "Map", "Enabled": True, "Alpha": 0.8,
         "Color Scheme": "map", "Draw Behind": True,
         "Topic": {"Value": f"/{map_robot}/map", **_qos("Transient Local", "Reliable")}},
    ]
    for i, ns in enumerate(robots):
        color = COLORS[i % len(COLORS)]
        displays += [
            {"Class": "rviz_default_plugins/PoseWithCovariance", "Name": f"{ns} amcl_pose",
             "Enabled": True, "Color": color, "Shape": "Arrow",
             "Topic": {"Value": f"/{ns}/amcl_pose", **_qos("Transient Local", "Reliable")}},
            {"Class": "nav2_rviz_plugins/ParticleCloud", "Name": f"{ns} particles",
             "Enabled": False, "Color": color, "Alpha": 1, "Shape": "Arrow (Flat)",
             "Arrow Length": 0.03, "Min Arrow Length": 0.02, "Max Arrow Length": 0.05,
             "Topic": {"Value": f"/{ns}/particle_cloud", **_qos("Volatile", "Best Effort")}},
            {"Class": "rviz_default_plugins/Path", "Name": f"{ns} received_global_plan",
             "Enabled": True, "Color": color, "Line Style": "Billboards", "Line Width": 0.015,
             "Alpha": 1,
             "Topic": {"Value": f"/{ns}/received_global_plan", **_qos("Volatile", "Reliable")}},
            {"Class": "rviz_default_plugins/Path", "Name": f"{ns} plan", "Enabled": False,
             "Color": color, "Line Style": "Lines", "Alpha": 1,
             "Topic": {"Value": f"/{ns}/plan", **_qos("Volatile", "Reliable")}},
        ]
    displays.append({"Class": "rviz_default_plugins/MarkerArray", "Name": "fleet markers",
                     "Enabled": True, "Namespaces": {},
                     "Topic": {"Value": "/fleet_view/markers", **_qos("Volatile", "Reliable")}})
    cfg = {
        "Visualization Manager": {
            "Class": "", "Displays": displays, "Enabled": True,
            "Global Options": {"Background Color": "48; 48; 48", "Fixed Frame": "map",
                               "Frame Rate": 30},
            "Name": "root",
            "Tools": [{"Class": "rviz_default_plugins/MoveCamera"},
                      {"Class": "rviz_default_plugins/Measure"}],
            "Views": {"Current": {"Class": "rviz_default_plugins/TopDownOrtho",
                                  "Name": "Current View", "Angle": 0, "Scale": 220,
                                  "X": 1.85, "Y": 1.3, "Target Frame": "<Fixed Frame>"}},
        },
        "Window Geometry": {"Height": 900, "Width": 1400},
    }
    return yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True)


if __name__ == "__main__":
    # 사용: rviz_config.py per <ns> | overview <ns>...
    mode, rest = sys.argv[1], sys.argv[2:]
    sys.stdout.write(per_robot(rest[0]) if mode == "per" else overview(rest))
