"""경기장 Gazebo 월드(SDF). 벽과 진열대는 충돌이 있는 상자, 고정 경로는 바닥 그림이다."""
from __future__ import annotations

import math

from fleet_core.fleet_map import FOLLOW_PATH, RETREAT

from fleet_gz_sim.arena import WORLD, obstacles

COLOURS = {"wall": "0.55 0.45 0.35", "shelf": "0.27 0.42 0.62"}
PATH_COLOUR = "0.2 0.3 0.8"
# Gazebo는 물리 한 스텝마다 /clock을 내고, use_sim_time 노드는 그때마다 깬다. 1 ms(1 kHz)에서는
# 핑키 3대 실행의 Python 노드가 코어를 50~90 %씩 써서 실시간의 0.3~0.5로 돌았다(2026-11-08).
PHYSICS_STEP_S = 0.004


def _num(v: float) -> str:
    return f"{v:.4f}".rstrip("0").rstrip(".") if v else "0"


def _material(colour: str) -> str:
    return f"<material><ambient>{colour} 1</ambient><diffuse>{colour} 1</diffuse></material>"


def _box_xml(box) -> str:
    sx, sy = box.x1 - box.x0, box.y1 - box.y0
    cx, cy = (box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2
    pose = f"{_num(cx)} {_num(cy)} {_num(box.height_m / 2)} 0 0 0"
    size = f"{_num(sx)} {_num(sy)} {_num(box.height_m)}"
    geometry = f"<geometry><box><size>{size}</size></box></geometry>"
    return (f'      <collision name="{box.name}"><pose>{pose}</pose>{geometry}</collision>\n'
            f'      <visual name="{box.name}_visual"><pose>{pose}</pose>{geometry}'
            f"{_material(COLOURS[box.kind])}</visual>\n")


def path_segments(site) -> list[tuple[tuple[float, float], tuple[float, float], str]]:
    """고정 경로(FollowPath, Retreat) 연결의 선분. 방향을 빼고 한 번씩, (a, b, 이름)."""
    seen, out = set(), []
    for link in site.links():
        key = frozenset((link.a, link.b))
        if link.via not in (FOLLOW_PATH, RETREAT) or key in seen:
            continue
        seen.add(key)
        for p, q in zip(link.points, link.points[1:], strict=False):
            out.append((tuple(p), tuple(q), f"{link.a}-{link.b}"))
    return out


def _path_xml(a, b, name: str) -> str:
    """바닥에 그린 고정 경로. 충돌 없음, 높이 1 mm라 라이다(약 0.125 m)에 보이지 않는다."""
    (ax, ay), (bx, by) = a, b
    length, yaw = math.hypot(bx - ax, by - ay), math.atan2(by - ay, bx - ax)
    pose = f"{_num((ax + bx) / 2)} {_num((ay + by) / 2)} 0.0005 0 0 {_num(yaw)}"
    return (f'      <visual name="path_{name}"><pose>{pose}</pose>'
            f"<geometry><box><size>{_num(length)} 0.01 0.001</size></box></geometry>"
            f"{_material(PATH_COLOUR)}</visual>\n")


def world_sdf(site, render_engine: str = "ogre2") -> str:
    body = "".join(_box_xml(b) for b in obstacles(site))
    body += "".join(_path_xml(a, b, name) for a, b, name in path_segments(site))
    layout = site.name.removeprefix("arena_")
    return f"""<?xml version="1.0"?>
<!-- 생성 파일: python3 -m fleet_gz_sim.arena (layout {layout}) 로 다시 만든다.
     손으로 고치지 않는다.
     월드 좌표 = 지도 좌표 = 구역 지도(fleet_map_{layout}.yaml) 좌표. 고정 경로는 바닥 그림이다. -->
<sdf version="1.9">
  <world name="{WORLD}">
    <physics name="4ms" type="ignored">
      <max_step_size>{PHYSICS_STEP_S}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>{render_engine}</render_engine>
    </plugin>
    <scene>
      <ambient>0.7 0.7 0.7 1</ambient>
      <background>0.85 0.85 0.85 1</background>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material>
            <ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="arena">
      <static>true</static>
      <link name="link">
{body}      </link>
    </model>
  </world>
</sdf>
"""
