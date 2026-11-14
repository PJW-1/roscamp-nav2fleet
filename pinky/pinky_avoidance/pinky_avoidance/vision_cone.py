# ---- 상자 면 위의 점 가운데 앞쪽 카메라 원뿔 안의 것을 고르는 계산(ROS 없음) ----
import math
from dataclasses import dataclass


# ---- 상자와 카메라 원뿔 ----
@dataclass(frozen=True)
class Box:
    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float
    height: float
    base_z: float = 0.0


@dataclass(frozen=True)
class Cone:
    fov_rad: float
    range_m: float


def make_cone(fov_deg: float, range_m: float) -> Cone:
    if not (math.isfinite(fov_deg) and 0.0 < fov_deg < 180.0):
        raise ValueError(f"fov_deg must be above 0 and below 180, got {fov_deg!r}")
    if not (math.isfinite(range_m) and range_m > 0.0):
        raise ValueError(f"range_m must be above 0, got {range_m!r}")
    return Cone(math.radians(fov_deg), range_m)


def yaw_of(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def box_from_marker(x, y, z, qx, qy, qz, qw, size_x, size_y, size_z) -> Box:
    return Box(x, y, yaw_of(qx, qy, qz, qw), size_x, size_y, size_z, z - size_z / 2.0)


# ---- 면 위의 점 ----
def _steps(length: float, step_m: float) -> list[float]:
    count = max(1, math.ceil(length / step_m - 1e-9))
    return [length * k / count for k in range(count + 1)]


def faces(box: Box) -> list[tuple[tuple[float, float], tuple[float, float], tuple[float, float]]]:
    c, s = math.cos(box.yaw), math.sin(box.yaw)
    hx, hy = box.size_x / 2.0, box.size_y / 2.0

    def world(lx, ly):
        return (box.x + c * lx - s * ly, box.y + s * lx + c * ly)

    corners = [world(hx, -hy), world(hx, hy), world(-hx, hy), world(-hx, -hy)]
    normals = [(c, s), (-s, c), (-c, -s), (s, -c)]
    return [(corners[i], corners[(i + 1) % 4], normals[i]) for i in range(4)]


def face_points(box: Box, step_m: float, toward=None) -> list[tuple[float, float, float]]:
    if not (math.isfinite(step_m) and step_m > 0.0):
        raise ValueError(f"step_m must be above 0, got {step_m!r}")
    levels = [box.base_z + dz for dz in _steps(box.height, step_m)]
    out = []
    for (ax, ay), (bx, by), (nx, ny) in faces(box):
        if toward is not None:
            mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
            if (toward[0] - mx) * nx + (toward[1] - my) * ny <= 0.0:
                continue
        length = math.hypot(bx - ax, by - ay)
        for d in _steps(length, step_m):
            t = d / length if length > 0.0 else 0.0
            px, py = ax + t * (bx - ax), ay + t * (by - ay)
            out += [(px, py, z) for z in levels]
    return out


# ---- 카메라 원뿔 안의 점 ----
def in_cone(point, pose, cone: Cone) -> bool:
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    distance = math.hypot(dx, dy)
    if distance <= 0.0 or distance > cone.range_m:
        return False
    bearing = math.atan2(dy, dx) - pose[2]
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))
    return abs(bearing) <= cone.fov_rad / 2.0


def seen_points(boxes, pose, cone: Cone, step_m: float) -> list[tuple[float, float, float]]:
    out = []
    for box in boxes:
        out += [p for p in face_points(box, step_m, toward=pose) if in_cone(p, pose, cone)]
    return out


def to_body(points, pose) -> list[tuple[float, float, float]]:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    out = []
    for x, y, z in points:
        dx, dy = x - pose[0], y - pose[1]
        out.append((c * dx + s * dy, -s * dx + c * dy, z))
    return out
