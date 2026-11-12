"""gz.launch.py가 배치와 시나리오에서 정하는 것. ROS 없이 돈다(fleet_core, fake_robots 순수 파이썬).

- 배치(layout): d2(기본), d, c, 시험용 d2_shelf. 구역 지도 fleet_core/data/fleet_map_<배치>.yaml과,
  arena.py가 그 지도로 만든 월드·지도·keepout 마스크(Arena)를 고른다.
- 시나리오의 `map`은 배치 이름이다(없으면 d2). launch의 layout과 다르면 띄우지 않는다.
- 핑키를 놓는 자리: 시나리오 robots의 start(지점 이름). start가 없으면 주차 지점을 차례로 준다.
  방향은 그 지점의 yaw(없으면 0).
- 돌릴 수 없는 시나리오: 로봇 고장이 있는 것(가짜 핑키가 일으키는 고장이라 진짜 pinky_fsm에는
  일으킬 곳이 없다). low_battery는 된다: sim_stand_in이 배터리 토픽으로 낸다.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml
from fleet_core.fleet_map import KNOWN_LAYOUTS, load_fleet_map
from fleet_core.fleet_map.model import SHARED

DEFAULT_LAYOUT = "d2"
ARENA_FAULTS = ("low_battery",)  # 경기장에서도 일으킬 수 있는 로봇 고장(sim_stand_in)


class ScenarioUnfit(RuntimeError):
    """Gazebo 경기장에서 돌릴 수 없는 시나리오."""


def layout_of(value: str) -> str:
    if value not in KNOWN_LAYOUTS:
        raise ScenarioUnfit(f"layout must be one of {', '.join(KNOWN_LAYOUTS)}, got {value!r}")
    return value


def scenario_path(value: str, *shares: str | Path) -> Path:
    """파일 경로이거나, 주어진 패키지 share의 scenarios 폴더에 있는 이름(앞 share가 먼저)."""
    path = Path(value)
    if path.is_file():
        return path
    name = value if value.endswith(".yaml") else value + ".yaml"
    folders = [Path(share) / "scenarios" for share in shares]
    for folder in folders:
        if (folder / name).is_file():
            return folder / name
    where = ", ".join(str(folder) for folder in folders)
    raise ScenarioUnfit(f"scenario {value!r} is nowhere: not a file and not in {where}")


def obstacles_path(value: str, share: str | Path) -> str:
    """gz.launch.py의 obstacles 인자. 비면 장애물 없음, 아니면 파일이나 share/obstacles의 이름."""
    from fleet_gz_sim.obstacles import ScheduleError
    if not value:
        return ""
    if Path(value).is_file():
        return value
    name = value if value.endswith(".yaml") else value + ".yaml"
    path = Path(share) / "obstacles" / name
    if not path.is_file():
        raise ScheduleError(f"obstacle schedule {value!r} is nowhere: not a file and not in "
                            f"{path.parent}")
    return str(path)


@dataclass(frozen=True)
class Arena:
    """한 배치의 Gazebo 월드, Nav2 지도 둘, keepout 마스크. layout은 관제 map_file 값이기도 하다."""
    layout: str
    world: str
    nav_map: str
    nav_map_fine: str
    keepout_mask: str


def arena_for(layout: str, gz_share: str | Path) -> Arena:
    name = load_fleet_map(layout_of(layout)).name
    gz = Path(gz_share)
    found = Arena(layout, str(gz / "worlds" / f"{name}.sdf"), str(gz / "maps" / f"{name}.yaml"),
                  str(gz / "maps" / f"{name}_fine.yaml"),
                  str(gz / "maps" / f"{name}_keepout.yaml"))
    missing = [p for p in (found.world, found.nav_map, found.nav_map_fine, found.keepout_mask)
               if not Path(p).is_file()]
    if missing:
        raise ScenarioUnfit(f"layout {layout!r} has no {', '.join(missing)} "
                            f"(python3 -m fleet_gz_sim.arena --layout {layout})")
    return found


def read_scenario(path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ScenarioUnfit(f"{Path(path).name} is not a mapping")
    return data


def scenario_layout(data: dict) -> str:
    """시나리오의 map(배치 이름). 없으면 d2."""
    return layout_of(str(data.get("map") or DEFAULT_LAYOUT))


def _faults(data: dict, seed: int | None) -> list[dict]:
    from fake_robots.scenario import expand_random
    return [f for f in expand_random(data, seed).get("faults") or [] if isinstance(f, dict)]


def check_fits(data: dict, name: str, layout: str, seed: int | None = None) -> None:
    """배치가 맞고 로봇 고장이 없어야 한다. random 블록은 seed로 풀어 본다(None이면 파일의 seed)."""
    from fake_robots.scenario import ROBOT_FAULTS
    wants = scenario_layout(data)
    if wants != layout:
        raise ScenarioUnfit(f"{name} is for layout {wants!r}, the launch runs {layout!r}")
    faults = sorted({str(f.get("type")) for f in _faults(data, seed)
                     if f.get("type") in ROBOT_FAULTS and f.get("type") not in ARENA_FAULTS})
    if faults:
        raise ScenarioUnfit(f"{name} has robot faults {faults}; only fake pinkys can make them")


@dataclass(frozen=True)
class StandInInputs:
    """sim_stand_in이 시나리오에서 쓰는 것: 도킹 시간과 고장(dict, fake_robots.battery가 읽는다)."""
    dock_s: float
    faults: list


def stand_in_inputs(path, seed: int | None = None) -> StandInInputs:
    data = read_scenario(path)
    dock = data.get("dock") or {}
    return StandInInputs(float(dock.get("duration_s", 3.0)), _faults(data, seed))


def has_low_battery(data: dict, seed: int | None = None) -> bool:
    return any(f.get("type") == "low_battery" for f in _faults(data, seed))


def start_nodes(site, data: dict) -> dict[str, str]:
    """로봇마다 시작 지점. start가 없으면 아직 안 쓴 주차 지점을 앞에서부터."""
    robots = data.get("robots") or {}
    if not isinstance(robots, dict) or not robots:
        raise ScenarioUnfit("the scenario names no robots")
    given = {robot: str((cfg or {}).get("start") or "") for robot, cfg in robots.items()}
    free = [p for p in site.parking if p not in given.values()]
    out = {}
    for robot, start in given.items():
        if not start:
            if not free:
                raise ScenarioUnfit(f"no parking node left for {robot}")
            start = free.pop(0)
        if start not in site.nodes:
            raise ScenarioUnfit(f"{robot} starts at {start!r}, which is not a node of {site.name}")
        out[robot] = start
    if len(set(out.values())) != len(out):
        raise ScenarioUnfit(f"two robots start on one node: {out}")
    return out


def narrow_strips(site) -> str:
    """스캔 거르개의 좁은 띠(넓은 구역이 아닌 구역의 사각형)를 "x0,y0,x1,y1; ..." 글로.

    pinky_nav_config nav2.launch.py의 narrow_strips 인자 꼴이다(추가지시서3 7-2). 차례는 x0, y0 순.
    """
    rects = sorted(rect for zone in site.zones.values() if zone.kind != SHARED
                   for rect in zone.rects)
    return "; ".join(",".join(f"{v:g}" for v in rect) for rect in rects)


def start_poses(site, starts: dict[str, str]) -> dict[str, tuple[float, float, float]]:
    """로봇마다 (x, y, yaw). yaw는 지점의 yaw, 없으면 0."""
    poses = {}
    for robot, node in starts.items():
        x, y = site.node_pose(node)
        yaw = site.nodes[node].yaw
        poses[robot] = (x, y, 0.0 if yaw is None else yaw)
    return poses
