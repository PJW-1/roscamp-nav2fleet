from __future__ import annotations

import math
from numbers import Real

RUN_MODES = ("RUN", "PAUSE", "STOP")


def shown(value) -> str:
    try:
        return repr(value)
    except Exception:
        return f"<{type(value).__name__}>"


def text(value) -> str:
    if isinstance(value, str):
        return str.__str__(value)
    return shown(value)


def checked(check, *args) -> str:
    try:
        return check(*args)
    except Exception as error:
        return f"the input could not be checked ({type(error).__name__})"


def _number(value) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool)


def pose_problem(x, y, yaw, bounds, margin) -> str:
    for label, value in (("x", x), ("y", y), ("theta", yaw)):
        if not _number(value) or not math.isfinite(value):
            return f"pose {label}={shown(value)} is not a finite number"
    min_x, min_y, max_x, max_y = bounds
    if not (min_x - margin <= x <= max_x + margin and min_y - margin <= y <= max_y + margin):
        return (f"pose ({x:.2f}, {y:.2f}) is more than {margin} m outside the map "
                f"x {min_x}..{max_x}, y {min_y}..{max_y}")
    return ""


def obstacle_of(x, y, kind, bounds, margin, kinds) -> tuple | None:
    if not (_number(x) and _number(y) and math.isfinite(x) and math.isfinite(y)):
        return None
    min_x, min_y, max_x, max_y = bounds
    if not (min_x - margin <= x <= max_x + margin and min_y - margin <= y <= max_y + margin):
        return None
    name = text(kind) if isinstance(kind, str) else ""
    return (float(x), float(y), name if name in kinds else kinds[0])


def robot_problem(robot, robots) -> str:
    if not isinstance(robot, str) or robot not in robots:
        return f"no such robot {shown(robot)}"
    return ""


def reported_id_problem(robot, reported_id) -> str:
    if reported_id in (None, "") or reported_id == robot:
        return ""
    return f"robot_id {shown(reported_id)} on the topic of {robot}"


def run_mode_problem(run_mode) -> str:
    if not isinstance(run_mode, str) or run_mode not in RUN_MODES:
        return f"run_mode {shown(run_mode)} is not one of {list(RUN_MODES)}"
    return ""


def clear_problem(robot, zone_ids, robots, zone_exists) -> str:
    problem = robot_problem(robot, robots)
    if problem:
        return problem
    if not isinstance(zone_ids, (list, tuple)):
        return f"zone_ids is a {type(zone_ids).__name__}, not a list"
    unknown = [z for z in zone_ids if not isinstance(z, str) or not zone_exists(z)]
    if unknown:
        return "no such zone " + ", ".join(shown(z) for z in unknown)
    return ""
