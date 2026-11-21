import math
from dataclasses import dataclass
from pathlib import Path

import yaml


# ---- 설정 읽기 ----
@dataclass(frozen=True)
class Mover:
    name: str
    shape: str
    size: tuple
    path: tuple
    speed: float
    once: bool = False
    yield_m: float = 0.0


def load(path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    movers = {}
    for name, m in data["movers"].items():
        if m["shape"] not in ("box", "cylinder"):
            raise ValueError(f"mover {name}: shape must be box or cylinder")
        if len(m["path"]) > 1 and float(m["speed"]) <= 0.0:
            raise ValueError(f"mover {name}: a path of two or more points needs a speed above 0")
        movers[name] = Mover(name, m["shape"], tuple(m["size"]),
                             tuple(tuple(p) for p in m["path"]), float(m["speed"]),
                             bool(m.get("once", False)), float(m.get("yield_m", 0.0)))
    for scenario, names in data["scenarios"].items():
        unknown = set(names) - set(movers)
        if unknown:
            raise ValueError(f"scenario {scenario} names unknown movers {sorted(unknown)}")
    data["movers"] = movers
    return data


# ---- 길 위의 자리 ----
def path_length(path) -> float:
    return sum(math.dist(a, b) for a, b in zip(path, path[1:], strict=False))


def point_at(path, s: float) -> tuple:
    for a, b in zip(path, path[1:], strict=False):
        seg = math.dist(a, b)
        if s <= seg:
            f = s / seg if seg > 0 else 0.0
            return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]),
                    math.atan2(b[1] - a[1], b[0] - a[0]))
        s -= seg
    a, b = path[-2], path[-1]
    return (b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0]))


def pose_at(mover: Mover, travelled: float) -> tuple:
    if len(mover.path) == 1:
        return (mover.path[0][0], mover.path[0][1], 0.0)
    length = path_length(mover.path)
    if mover.once:
        return point_at(mover.path, min(travelled, length))
    s = travelled % (2 * length)
    if s <= length:
        return point_at(mover.path, s)
    x, y, yaw = point_at(mover.path, 2 * length - s)
    return (x, y, yaw + math.pi)


def step(mover: Mover, travelled: float, dt: float, robot_xy, pose) -> float:
    near = robot_xy is not None and math.dist(robot_xy, pose[:2]) < mover.yield_m
    if mover.yield_m > 0.0 and near:
        return travelled
    return travelled + mover.speed * dt


# ---- Gazebo 모델 ----
def sdf(mover: Mover) -> str:
    if mover.shape == "box":
        geom = f"<box><size>{mover.size[0]} {mover.size[1]} {mover.size[2]}</size></box>"
        height = mover.size[2]
    else:
        geom = (f"<cylinder><radius>{mover.size[0]}</radius>"
                f"<length>{mover.size[1]}</length></cylinder>")
        height = mover.size[1]
    return (f"<?xml version='1.0'?><sdf version='1.9'><model name='bench_{mover.name}'>"
            f"<static>true</static><link name='link'><pose>0 0 {height / 2} 0 0 0</pose>"
            f"<collision name='c'><geometry>{geom}</geometry></collision>"
            f"<visual name='v'><geometry>{geom}</geometry>"
            f"<material><diffuse>0.9 0.4 0.1 1</diffuse></material></visual>"
            f"</link></model></sdf>")


# ---- 몸과 장애물 사이 거리 ----
def to_world(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in points]


def _seg_dist(p, a, b) -> float:
    ax, ay = b[0] - a[0], b[1] - a[1]
    n = ax * ax + ay * ay
    t = 0.0 if n == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / n))
    return math.dist(p, (a[0] + t * ax, a[1] + t * ay))


def _inside(p, poly) -> bool:
    inside = False
    for a, b in zip(poly, poly[1:] + poly[:1], strict=False):
        if (a[1] > p[1]) != (b[1] > p[1]):
            if p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]):
                inside = not inside
    return inside


def _cross(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _segments_cross(a, b, c, d) -> bool:
    return (_cross(a, b, c) * _cross(a, b, d) < 0) and (_cross(c, d, a) * _cross(c, d, b) < 0)


def poly_dist(p1, p2) -> float:
    if any(_inside(p, p2) for p in p1) or any(_inside(p, p1) for p in p2):
        return 0.0
    edges2 = list(zip(p2, p2[1:] + p2[:1], strict=False))
    edges1 = list(zip(p1, p1[1:] + p1[:1], strict=False))
    if any(_segments_cross(a, b, c, d) for a, b in edges1 for c, d in edges2):
        return 0.0
    return min(min(_seg_dist(p, a, b) for a, b in edges2 for p in p1),
               min(_seg_dist(p, a, b) for a, b in edges1 for p in p2))


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def mover_shape(mover: Mover, pose):
    x, y, yaw = pose
    if mover.shape == "box":
        hx, hy = mover.size[0] / 2, mover.size[1] / 2
        return to_world(rect(-hx, -hy, hx, hy), x, y, yaw)
    r = mover.size[0]
    return [(x + r * math.cos(k * math.pi / 8), y + r * math.sin(k * math.pi / 8))
            for k in range(16)]
