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
v7만 더(가운데 통로 갈래, gz.launch.py nav_map_variant로 고른다):
  maps/arena_v7_{a,b,ab}.{yaml,pgm}    costmap용 갈래 지도(arena_maps.NAV_VARIANTS)
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from fleet_core.fleet_map import KNOWN_LAYOUTS, LAYOUTS

WORLD = "arena"  # 모든 배치의 월드 이름(launch와 장애물 노드가 이 이름으로 부른다)
WALL_M = 0.05
WALL_HEIGHT_M = 0.30  # 라이다 높이(약 0.125 m)보다 높게
LIDAR_Z_M = 0.125
MIN_SHELF_HEIGHT_M = 0.15  # 라이다가 바닥 위 0.125 m를 본다
PKG = Path(__file__).resolve().parents[1]
NAV_VARIANT_LAYOUTS = ("v7",)  # 갈래 지도(arena_maps.NAV_VARIANTS)를 만드는 배치


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
    # 시뮬 전용 진열대(sim_shelves): 월드와 시뮬 지도가 같은 상자를 쓴다
    return [Box(f"shelf_{i}", *rect, "shelf", height)
            for i, rect in enumerate(site.sim_shelves, start=1)]


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
    (out / "worlds" / f"{name}.sdf").write_text(world_sdf(site), encoding="utf-8", newline="\n")
    for suffix, resolution in (("", COARSE_M), ("_fine", FINE_M)):
        (out / "maps" / f"{name}{suffix}.yaml").write_text(
            map_yaml(f"{name}{suffix}", resolution, site), encoding="utf-8", newline="\n")
        (out / "maps" / f"{name}{suffix}.pgm").write_bytes(pgm(resolution, site))
    (out / "maps" / f"{name}_keepout.yaml").write_text(
        map_yaml(f"{name}_keepout", FINE_M, site), encoding="utf-8", newline="\n")
    (out / "maps" / f"{name}_keepout.pgm").write_bytes(keepout_pgm(FINE_M, site))
    return outputs(name)


def nav_variant_outputs(name: str) -> tuple[str, ...]:
    from fleet_gz_sim.arena_maps import NAV_VARIANTS
    return tuple(f"maps/{name}_{variant}.{ext}" for variant in NAV_VARIANTS
                 for ext in ("yaml", "pgm"))


def generate_nav_variants(out: Path, site) -> tuple[str, ...]:
    """costmap용 갈래 지도를 out/maps에 쓰고 쓴 파일 목록을 돌려준다.
    기본 지도는 건드리지 않는다."""
    from fleet_gz_sim.arena_maps import NAV_VARIANTS, map_yaml, pgm
    name = site.name
    (out / "maps").mkdir(parents=True, exist_ok=True)
    for variant, (resolution, paint) in NAV_VARIANTS.items():
        stem = f"{name}_{variant}"
        note = f"갈래 {variant}: {resolution} m, 칠하기 {paint} (nav_map_variant:={variant})"
        (out / "maps" / f"{stem}.yaml").write_text(
            map_yaml(stem, resolution, site, note), encoding="utf-8", newline="\n")
        (out / "maps" / f"{stem}.pgm").write_bytes(pgm(resolution, site, paint))
    return nav_variant_outputs(name)


def main(argv=None) -> None:
    from fleet_core.fleet_map import load_fleet_map
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=PKG,
                        help="worlds/, maps/를 쓸 폴더(기본: 이 패키지 소스)")
    parser.add_argument("--layout", choices=KNOWN_LAYOUTS, action="append",
                        help="배치(여러 번). 없으면 후보 셋. 시험용 d2_shelf는 이름을 줘야 한다")
    args = parser.parse_args(argv)
    for layout in args.layout or LAYOUTS:
        site = load_fleet_map(layout)
        written = generate(args.out, site)
        if layout in NAV_VARIANT_LAYOUTS:
            written += generate_nav_variants(args.out, site)
        print(f"{layout}: wrote {', '.join(written)} under {args.out}")


if __name__ == "__main__":
    main()
