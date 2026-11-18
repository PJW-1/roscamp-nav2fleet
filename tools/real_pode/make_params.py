#!/usr/bin/env python3
"""실물 Pode용 Nav2 설정 사본을 속도 S로 만든다. ROS 없이 돈다.

    python3 make_params.py --speed 0.12 --out params

저장소의 pode_nav_config/params/nav2_params.yaml과 pode_avoidance/params/avoidance.yaml을
그대로 베끼고 앞으로 가는 컨트롤러의 desired_linear_vel만 S로 바꾼다:
FollowPath, FollowPathThrough(nav2_params.yaml), follow_path_goal(avoidance.yaml, FollowPathGoal).
FollowPathReverse(0.05)는 그대로다.
고정 경로 컨트롤러(FollowPath, FollowPathThrough, FollowPathReverse)의 use_collision_detection은
기본으로 false로 쓴다(시뮬 기본과 같다). --fixed-line-check를 주면 원본 값과 상관없이 true로 쓴다.
FollowPathGoal(follow_path_goal)의 검사는 늘 그대로다.
바꾼 줄 말고는 주석까지 그대로 두고, 다시 읽어 값을 확인한다.
"""
import argparse
import copy
import math
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
MAX_SPEED = 0.25  # 저장소 값
NAV2_NAME = "nav2_params.yaml"
AVOID_NAME = "avoidance.yaml"
NAV2_REL = Path("pode") / "pode_nav_config" / "params" / NAV2_NAME
AVOID_REL = Path("pode") / "pode_avoidance" / "params" / AVOID_NAME
# 저장소 안(PC)과 로봇(~/nav2fleet_ws/src, deploy.sh가 놓는 곳) 순서로 찾는다
SOURCE_ROOTS = (HERE.parents[1], Path.home() / "nav2fleet_ws" / "src")
NAV2_TARGETS = (("controller_server", "ros__parameters", "FollowPath", "desired_linear_vel"),
                ("controller_server", "ros__parameters", "FollowPathThrough",
                 "desired_linear_vel"))
COLLISION_TARGETS = tuple(("controller_server", "ros__parameters", name, "use_collision_detection")
                          for name in ("FollowPath", "FollowPathThrough", "FollowPathReverse"))
AVOID_TARGETS = (("follow_path_goal", "desired_linear_vel"),)
FORWARD = ("FollowPath", "FollowPathThrough")


# ---- 속도 확인 ----
def checked_speed(speed) -> float:
    value = float(speed)
    if not math.isfinite(value):
        raise ValueError(f"speed must be a finite number, got {speed!r}")
    # 반올림 전에 범위를 본다(0.25001이 0.25로 빠져나가지 않게)
    if not 0.0 < value <= MAX_SPEED or round(value, 4) <= 0.0:
        raise ValueError(f"speed must be above 0 and at most {MAX_SPEED} m/s, got {speed!r}")
    value = round(value, 4)
    if not 0.0 < value <= MAX_SPEED:
        raise ValueError(f"speed must be above 0 and at most {MAX_SPEED} m/s, got {speed!r}")
    return value


def _text(value: float) -> str:
    out = f"{value:.4f}".rstrip("0")
    return out + "0" if out.endswith(".") else out


# ---- 줄 단위 바꾸기(주석 보존) ----
def replace_values(text: str, targets, value, note=None) -> str:
    """targets 경로의 스칼라 값 줄만 바꾼다. 경로마다 정확히 한 번 바뀌어야 한다."""
    if note is None:
        note = f"make_params.py --speed (저장소 {MAX_SPEED})"
    shown = _text(value) if isinstance(value, float) else str(value).lower()
    lines = text.splitlines(keepends=True)
    stack = []  # (들여쓰기, 키)
    hits = {t: 0 for t in targets}
    for i, line in enumerate(lines):
        body = line.split("#", 1)[0].rstrip()
        stripped = body.lstrip()
        if not stripped or stripped.startswith("-") or ":" not in stripped:
            continue
        indent = len(body) - len(stripped)
        key = stripped.split(":", 1)[0].strip().strip("\"'")
        while stack and stack[-1][0] >= indent:
            stack.pop()
        stack.append((indent, key))
        path = tuple(k for _, k in stack)
        if path in hits:
            end = "\n" if line.endswith("\n") else ""
            lines[i] = " " * indent + f"{key}: {shown}  # {note}{end}"
            hits[path] += 1
    bad = {".".join(t): n for t, n in hits.items() if n != 1}
    if bad:
        raise RuntimeError(f"expected each target once, found {bad}")
    return "".join(lines)


def _set(data: dict, path, value) -> None:
    for key in path[:-1]:
        data = data[key]
    data[path[-1]] = value


def _converted(src: Path, targets, value: float, collision=(), collision_on=False) -> str:
    text = src.read_text(encoding="utf-8")
    out = replace_values(text, targets, value)
    if collision:
        out = replace_values(out, collision, collision_on,
                             f"make_params.py 고정 경로 충돌 검사 {'켬' if collision_on else '끔'}")
    want = copy.deepcopy(yaml.safe_load(text))
    for path in targets:
        _set(want, path, value)
    for path in collision:
        _set(want, path, collision_on)
    if yaml.safe_load(out) != want:
        raise RuntimeError(f"{src}: the copy differs from the source beyond the changed values")
    return out


def find_source(rel: Path, given=None) -> Path:
    if given:
        return Path(given)
    for root in SOURCE_ROOTS:
        if (root / rel).is_file():
            return root / rel
    raise FileNotFoundError(f"{rel} not found under {', '.join(map(str, SOURCE_ROOTS))}")


def write_params(speed, out_dir, nav2_src=None, avoid_src=None, fixed_line_check=False) -> tuple:
    """두 사본을 out_dir에 쓴다. 확인을 모두 통과해야 쓴다."""
    value = checked_speed(speed)
    nav2 = _converted(find_source(NAV2_REL, nav2_src), NAV2_TARGETS, value, COLLISION_TARGETS,
                      bool(fixed_line_check))
    avoid = _converted(find_source(AVOID_REL, avoid_src), AVOID_TARGETS, value)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / NAV2_NAME).write_text(nav2, encoding="utf-8")
    (out / AVOID_NAME).write_text(avoid, encoding="utf-8")
    return out / NAV2_NAME, out / AVOID_NAME


# ---- launch가 쓰는 도우미 ----
def speed_of(params_file) -> float:
    """만든 nav2_params.yaml의 앞으로 가는 속도(FollowPath, FollowPathThrough는 같아야 한다)."""
    path = Path(params_file)
    if not path.is_file():
        raise FileNotFoundError(f"{path} does not exist: run make_params.py --speed S "
                                f"--out {path.parent} (start.sh does it)")
    ctl = yaml.safe_load(path.read_text(encoding="utf-8"))["controller_server"]["ros__parameters"]
    speeds = {float(ctl[name]["desired_linear_vel"]) for name in FORWARD}
    if len(speeds) != 1:
        raise ValueError(f"{path}: FollowPath and FollowPathThrough speeds differ: {speeds}")
    return speeds.pop()


def horizon_cap(given: str, params_file) -> str:
    """pode_fsm horizon_cap_mps. 비면 컨트롤러 속도, 주면 그 속도 이하여야 한다."""
    speed = speed_of(params_file)
    if not str(given).strip():
        return _text(speed)
    try:
        cap = float(given)
    except ValueError:
        raise ValueError(f"horizon_cap_mps must be a number, got {given!r}") from None
    if not math.isfinite(cap) or not 0.0 < cap <= speed + 1e-9:
        raise ValueError(f"horizon_cap_mps must be above 0 and at most the controller "
                         f"speed {speed} in {params_file}, got {given!r}")
    return _text(cap)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--speed", required=True, help=f"m/s, 0 초과 {MAX_SPEED} 이하")
    parser.add_argument("--out", required=True, type=Path, help="사본을 쓸 폴더")
    parser.add_argument("--nav2-src", help="원본 nav2_params.yaml(기본: 저장소 또는 로봇 src)")
    parser.add_argument("--avoidance-src", help="원본 avoidance.yaml(기본: 같은 곳)")
    parser.add_argument("--fixed-line-check", action="store_true",
                        help="고정 경로 충돌 검사를 켜 둔다(기본: 끔)")
    args = parser.parse_args(argv)
    try:
        nav2, avoid = write_params(args.speed, args.out, args.nav2_src, args.avoidance_src,
                                  args.fixed_line_check)
    except (ValueError, FileNotFoundError, RuntimeError) as problem:
        print(f"make_params: {problem}", file=sys.stderr)
        return 2
    print(f"make_params: speed {_text(checked_speed(args.speed))} m/s -> {nav2}, {avoid}")
    print(f"make_params: fixed-line check {'on' if args.fixed_line_check else 'off'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
