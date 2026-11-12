#!/usr/bin/env python3
"""구역 지도에서 Nav2 keepout 마스크를 만든다. ROS 없이 돈다.

    python3 tools/make_keepout.py --layout d2 --map maps/arena.yaml
        -> maps/arena_keepout.yaml, maps/arena_keepout.pgm

급행, 도크, 갈림목(넓은 구역이 아닌 구역)을 막은 그림을 `--map` 지도와 같은 크기, 원점, 칸
크기로 만든다. 그래서 SLAM으로 만든 실제 경기장 지도에도 그대로 쓸 수 있다. 지도 그림은 PGM
이어야 한다(크기만 읽는다). 만든 yaml을 `pinky_nav_config` `nav2.launch.py`의
`keepout_mask:=`에 넘긴다.

막는 넓이는 구역보다 몸 반폭(`--pad`, 기본 0.06 m)만큼 넓다: keepout 필터는 둘레를 부풀리지
않으므로 곧게 선 몸이 구역에 걸치지 않게 하려는 것이다. 45도로 선 몸의 모서리는 0.085 m까지
나가므로, 어느 방향에서도 걸치지 않게 하려면 `--pad 0.085`를 준다.

지도의 origin에 회전(yaw)이 있으면 만들지 않는다: 마스크는 축에 맞춘 격자로만 그린다.
"""
import argparse
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pc" / "fleet_core"))

from fleet_core.fleet_map import load_fleet_map  # noqa: E402
from fleet_core.fleet_map.keepout import (  # noqa: E402
    BODY_HALF_WIDTH_M,
    mask,
    mask_yaml,
    pgm,
    pgm_size,
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--layout", default=None,
                        help="구역 지도 이름(d2, d, c, 시험용 d2_shelf) 또는 yaml 경로")
    parser.add_argument("--map", required=True, type=Path, help="맞출 지도의 map_server yaml")
    parser.add_argument("--out", type=Path, help="마스크 yaml 경로(기본: <지도>_keepout.yaml)")
    parser.add_argument("--pad", type=float, default=BODY_HALF_WIDTH_M, help="넓히는 폭 m")
    args = parser.parse_args(argv)

    info = yaml.safe_load(args.map.read_text(encoding="utf-8"))
    picture = (args.map.parent / info["image"]).read_bytes()
    try:
        width, height = pgm_size(picture)
    except ValueError as problem:
        print(f"{args.map.parent / info['image']}: {problem}", file=sys.stderr)
        return 2
    if len(info["origin"]) > 2 and abs(info["origin"][2]) > 1e-9:
        print(f"{args.map}: the origin has a yaw of {info['origin'][2]}; this tool only "
              "draws masks for maps that are not turned", file=sys.stderr)
        return 2
    origin = tuple(info["origin"][:2])
    grid = mask(load_fleet_map(args.layout), width, height, info["resolution"], origin, args.pad)
    out = args.out or args.map.with_name(args.map.stem + "_keepout.yaml")
    image = out.with_suffix(".pgm")
    image.write_bytes(pgm(grid))
    out.write_text(mask_yaml(image.name, info["resolution"], origin), encoding="utf-8")
    closed = sum(row.count(True) for row in grid)
    print(f"{out}: {width} x {height} cells, {closed} closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
