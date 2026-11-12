"""좁은 띠 안에서 지도의 벽·진열대에 붙은 라이다 점 고르기(추가지시서3 7-2, 기준(사용자)).

ROS를 모른다. 점은 map 좌표의 numpy 배열이고, 벽과 진열대는 점유 격자(지도의 막힌 칸)다.
- 벽·진열대 칸에서 band_m 안에 있는 점만 지운다(라이다 잡음 크기).
- 좁은 띠(급행, 갈림목, 도크의 사각형) 안의 점에만 적용한다. 넓은 구역의 점은 그대로 둔다.
- 거리는 점이 놓인 칸의 가운데에서 막힌 칸의 가장 가까운 면까지 잰다. 그래서 칸 크기의
  반만큼(1 cm 지도면 0.5 cm, 대각선이면 0.7 cm) 어긋날 수 있다.
"""
import math

import numpy as np


def parse_strips(text: str) -> list[tuple[float, float, float, float]]:
    """"x0, y0, x1, y1; x0, y0, x1, y1" 꼴의 글을 사각형 목록으로. 비면 빈 목록."""
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
    """막힌 칸의 면에서 칸 가운데까지가 band_m 이하인 칸(막힌 칸 포함)의 bool 격자."""
    reach = int(math.ceil(band_m / resolution + 0.5))
    rows, cols = occupied.shape
    near = occupied.copy()
    for di in range(-reach, reach + 1):
        for dj in range(-reach, reach + 1):
            gap = math.hypot(max(0.0, abs(di) - 0.5), max(0.0, abs(dj) - 0.5)) * resolution
            if gap > band_m + 1e-9 or (di == 0 and dj == 0):
                continue
            # 막힌 칸 (i, j)가 칸 (i + di, j + dj)를 가깝게 만든다
            src = occupied[max(0, -di):rows - max(0, di), max(0, -dj):cols - max(0, dj)]
            near[max(0, di):rows - max(0, -di), max(0, dj):cols - max(0, -dj)] |= src
    return near


class WallBand:
    """지도 한 장에 대해 지울 점을 고른다. 지도가 바뀌면 새로 만든다."""

    def __init__(self, occupied, origin, resolution, band_m, strips):
        self.origin = (float(origin[0]), float(origin[1]))
        self.resolution = float(resolution)
        self.strips = np.asarray(strips, dtype=float).reshape(-1, 4)
        self.near = near_cells(np.asarray(occupied, dtype=bool), self.resolution, band_m)

    def drops(self, xs, ys) -> np.ndarray:
        """점마다 지울지(True). 지도 밖이나 띠 밖의 점은 지우지 않는다."""
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
    """라이다 빔 끝점을 map 좌표로. pose는 map에서 라이다의 (x, y, yaw)."""
    r = np.asarray(ranges, dtype=float)
    a = pose[2] + angle_min + angle_increment * np.arange(len(r))
    return pose[0] + r * np.cos(a), pose[1] + r * np.sin(a)
