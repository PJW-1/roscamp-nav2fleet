import math

import numpy as np


def parse_strips(text: str) -> list[tuple[float, float, float, float]]:
    strips = []
    for part in text.split(";"):
        if not part.strip():
            continue
        values = [float(v) for v in part.split(",")]
        if len(values) != 4:
            raise ValueError(f"a strip needs x0, y0, x1, y1: {part!r}")
        x0, y0, x1, y1 = values
        strips.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)))
    return strips


def near_cells(occupied: np.ndarray, resolution: float, band_m: float) -> np.ndarray:
    reach = int(math.ceil(band_m / resolution + 0.5))
    rows, cols = occupied.shape
    near = occupied.copy()
    for di in range(-reach, reach + 1):
        for dj in range(-reach, reach + 1):
            gap = math.hypot(max(0.0, abs(di) - 0.5), max(0.0, abs(dj) - 0.5)) * resolution
            if gap > band_m + 1e-9 or (di == 0 and dj == 0):
                continue
            src = occupied[max(0, -di):rows - max(0, di), max(0, -dj):cols - max(0, dj)]
            near[max(0, di):rows - max(0, -di), max(0, dj):cols - max(0, -dj)] |= src
    return near


class WallBand:
    def __init__(self, occupied, origin, resolution, band_m, strips):
        self.origin = (float(origin[0]), float(origin[1]))
        self.resolution = float(resolution)
        self.strips = np.asarray(strips, dtype=float).reshape(-1, 4)
        self.near = near_cells(np.asarray(occupied, dtype=bool), self.resolution, band_m)

    def drops(self, xs, ys) -> np.ndarray:
        xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
        if not len(self.strips):
            return np.zeros(xs.shape, dtype=bool)
        s = self.strips
        in_strip = ((xs[:, None] >= s[:, 0]) & (xs[:, None] <= s[:, 2])
                    & (ys[:, None] >= s[:, 1]) & (ys[:, None] <= s[:, 3])).any(axis=1)
        cols = np.floor((xs - self.origin[0]) / self.resolution).astype(int)
        rows = np.floor((ys - self.origin[1]) / self.resolution).astype(int)
        height, width = self.near.shape
        inside = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        near = np.zeros(xs.shape, dtype=bool)
        near[inside] = self.near[rows[inside], cols[inside]]
        return in_strip & near


def beam_points(ranges, angle_min, angle_increment, pose):
    r = np.asarray(ranges, dtype=float)
    a = pose[2] + angle_min + angle_increment * np.arange(len(r))
    return pose[0] + r * np.cos(a), pose[1] + r * np.sin(a)


def filter_ranges(band: WallBand, ranges, angle_min, angle_increment, range_min, range_max,
                  pose) -> np.ndarray:
    r = np.asarray(ranges, dtype=float).copy()
    valid = np.zeros(r.shape, dtype=bool)
    finite = np.isfinite(r)
    valid[finite] = (r[finite] >= range_min) & (r[finite] <= range_max)
    index = np.nonzero(valid)[0]
    if len(index):
        a = pose[2] + angle_min + angle_increment * index
        xs = pose[0] + r[index] * np.cos(a)
        ys = pose[1] + r[index] * np.sin(a)
        r[index[band.drops(xs, ys)]] = math.inf
    return r


def origin_yaw(qz: float, qw: float) -> float:
    yaw = 2.0 * math.atan2(qz, qw)
    return math.atan2(math.sin(yaw), math.cos(yaw))
