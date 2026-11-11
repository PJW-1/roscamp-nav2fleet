"""Gazebo 경기장: 구역 지도(fleet_core.fleet_map)에서 월드(SDF), Nav2 지도, keepout 마스크를 만든다.

월드 좌표 = 지도(map) 좌표 = 구역 지도 좌표다(원점 [0, 0]). 그래서 Gazebo의 실제 위치를 AMCL
위치, 관제 지점 좌표와 바로 비교할 수 있다. 벽은 구역 지도의 bounds 둘레, 진열대는 shelves다.

    python3 -m fleet_gz_sim.arena                # 세 배치(d2, d, c)를 이 패키지의 worlds/, maps/에
    python3 -m fleet_gz_sim.arena --layout d2    # 한 배치만
    (소스 폴더 sim/fleet_gz_sim에서 PYTHONPATH=../../pc/fleet_core:. 로 부른다)

배치마다 만드는 것:
  worlds/arena_<배치>.sdf              벽, 진열대, 바닥에 그린 고정 경로
  maps/arena_<배치>.{yaml,pgm}         5 cm 지도(costmap)
  maps/arena_<배치>_fine.{yaml,pgm}    1 cm 지도(AMCL, R15)
  maps/arena_<배치>_keepout.{yaml,pgm} global costmap keepout 마스크(arena_maps.keepout_rects)
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

WORLD = "arena"  # 모든 배치의 월드 이름(launch와 장애물 노드가 이 이름으로 부른다)
LAYOUTS = ("d2", "d", "c")
WALL_M = 0.05
WALL_HEIGHT_M = 0.30  # 라이다 높이(약 0.125 m)보다 높게
LIDAR_Z_M = 0.125
MIN_SHELF_HEIGHT_M = 0.15  # 지시서 5절: 라이다가 바닥 위 0.125 m를 본다
PKG = Path(__file__).resolve().parents[1]


class ArenaError(ValueError):
    """구역 지도로 경기장을 만들 수 없다."""


@dataclass(frozen=True)
class Box:
    name: str
    x0: float
    y0: float
    x1: float
    y1: float
    kind: str  # wall, shelf
    height_m: float


def walls(site) -> list[Box]:
    x0, y0, x1, y1 = site.bounds()
    t = WALL_M
    return [Box("wall_south", x0 - t, y0 - t, x1 + t, y0, "wall", WALL_HEIGHT_M),
            Box("wall_north", x0 - t, y1, x1 + t, y1 + t, "wall", WALL_HEIGHT_M),
            Box("wall_west", x0 - t, y0, x0, y1, "wall", WALL_HEIGHT_M),
            Box("wall_east", x1, y0, x1 + t, y1, "wall", WALL_HEIGHT_M)]


def shelves(site) -> list[Box]:
    height = float(site.shelf_height_m)
    if height < MIN_SHELF_HEIGHT_M - 1e-9:
        raise ArenaError(f"{site.name}: shelf_height_m {height} is below {MIN_SHELF_HEIGHT_M} m; "
                         f"the lidar at {LIDAR_Z_M} m would not see the shelves")
    return [Box(f"shelf_{i}", *rect, "shelf", height)
            for i, rect in enumerate(site.shelves, start=1)]


def obstacles(site) -> list[Box]:
    """벽과 진열대. 지도에 막힌 칸으로 들어가는 것들이다."""
    return [*walls(site), *shelves(site)]


def outputs(name: str) -> tuple[str, ...]:
    return (f"worlds/{name}.sdf", f"maps/{name}.yaml", f"maps/{name}.pgm",
            f"maps/{name}_fine.yaml", f"maps/{name}_fine.pgm",
            f"maps/{name}_keepout.yaml", f"maps/{name}_keepout.pgm")


def generate(out: Path, site) -> tuple[str, ...]:
    """site의 월드와 지도를 out/worlds, out/maps에 쓰고 쓴 파일 목록을 돌려준다."""
    from fleet_gz_sim.arena_maps import COARSE_M, FINE_M, keepout_pgm, map_yaml, pgm
    from fleet_gz_sim.arena_world import world_sdf
    name = site.name
    (out / "worlds").mkdir(parents=True, exist_ok=True)
    (out / "maps").mkdir(parents=True, exist_ok=True)
    (out / "worlds" / f"{name}.sdf").write_text(world_sdf(site), encoding="utf-8")
    for suffix, resolution in (("", COARSE_M), ("_fine", FINE_M)):
        (out / "maps" / f"{name}{suffix}.yaml").write_text(
            map_yaml(f"{name}{suffix}", resolution, site), encoding="utf-8")
        (out / "maps" / f"{name}{suffix}.pgm").write_bytes(pgm(resolution, site))
    (out / "maps" / f"{name}_keepout.yaml").write_text(
        map_yaml(f"{name}_keepout", FINE_M, site), encoding="utf-8")
    (out / "maps" / f"{name}_keepout.pgm").write_bytes(keepout_pgm(FINE_M, site))
    return outputs(name)


def main(argv=None) -> None:
    from fleet_core.fleet_map import load_fleet_map
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=PKG,
                        help="worlds/, maps/를 쓸 폴더(기본: 이 패키지 소스)")
    parser.add_argument("--layout", choices=LAYOUTS, action="append",
                        help="배치(여러 번 줄 수 있다). 없으면 세 배치 모두")
    args = parser.parse_args(argv)
    for layout in args.layout or LAYOUTS:
        written = generate(args.out, load_fleet_map(layout))
        print(f"{layout}: wrote {', '.join(written)} under {args.out}")


if __name__ == "__main__":
    main()
