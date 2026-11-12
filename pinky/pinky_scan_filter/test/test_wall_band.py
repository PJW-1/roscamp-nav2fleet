"""wall_band: 좁은 띠 안에서 벽·진열대에 붙은 라이다 점을 고르는 순수 함수(추가지시서3 7-2)."""
import math
import warnings

import numpy as np
import pytest
from pinky_scan_filter.wall_band import (
    WallBand,
    beam_points,
    filter_ranges,
    near_cells,
    origin_yaw,
    parse_strips,
)

RES = 0.01


def _grid():
    """2.00 x 1.00 m, 1 cm. 북쪽 벽은 y 0.99 위, 진열대는 x 0.50~1.48, y 0.775~0.79."""
    occ = np.zeros((100, 200), dtype=bool)
    occ[99:, :] = True                 # y 0.99~1.00
    occ[77:79, 50:148] = True          # 진열대: y 0.77~0.79 칸(0.775는 칸 경계로 내림)
    return occ


STRIPS = [(0.0, 0.79, 0.50, 0.99), (0.50, 0.79, 1.48, 0.99)]


def _band(band=0.04, strips=STRIPS):
    return WallBand(_grid(), origin=(0.0, 0.0), resolution=RES, band_m=band, strips=strips)


def test_a_point_in_a_strip_close_to_the_wall_is_dropped():
    assert _band().drops(np.array([0.30]), np.array([0.965])).tolist() == [True]


def test_a_point_in_a_strip_farther_than_the_band_is_kept():
    # 벽 면 0.99에서 0.05 m: 0.04 띠 밖, 0.06 띠 안
    assert _band(0.04).drops(np.array([0.30]), np.array([0.94])).tolist() == [False]
    assert _band(0.06).drops(np.array([0.30]), np.array([0.94])).tolist() == [True]


def test_an_obstacle_in_the_middle_of_the_strip_is_kept():
    assert _band(0.06).drops(np.array([0.90]), np.array([0.89])).tolist() == [False]


def test_a_point_near_the_shelf_inside_the_lane_is_dropped():
    # 진열대 윗면 0.79, 급행 안 0.82: 0.03 m
    assert _band().drops(np.array([1.00]), np.array([0.82])).tolist() == [True]


def test_points_in_the_wide_zone_stay_even_next_to_a_wall():
    # 마당(띠 밖)의 진열대 아래 0.76: 진열대에서 0.01 m지만 지우지 않는다
    assert _band().drops(np.array([1.00]), np.array([0.76])).tolist() == [False]


def test_points_outside_the_map_are_kept():
    assert _band().drops(np.array([-1.0, 5.0]), np.array([0.9, 0.9])).tolist() == [False, False]


def test_no_strips_drops_nothing():
    band = _band(strips=[])
    assert band.drops(np.array([0.30]), np.array([0.98])).tolist() == [False]


def test_near_cells_match_a_cell_by_cell_count():
    rng = np.random.default_rng(3)
    occ = rng.random((30, 40)) > 0.97
    near = near_cells(occ, RES, 0.04)
    for i in range(30):
        for j in range(40):
            want = any(math.hypot(max(0.0, abs(i - a) - 0.5), max(0.0, abs(j - b) - 0.5)) * RES
                       <= 0.04 + 1e-9 for a, b in zip(*np.nonzero(occ), strict=True))
            assert near[i, j] == want, (i, j)


def test_strips_are_read_from_a_text_parameter():
    assert parse_strips("0, 0.79, 0.5, 0.99; 0.5,0.79,1.48,0.99") == STRIPS
    assert parse_strips("") == []
    with pytest.raises(ValueError):
        parse_strips("0, 0.79, 0.5")


def test_beam_points_put_each_range_in_the_map_frame():
    # 라이다가 (1, 1)에서 북쪽(90°)을 본다: 0° 빔 0.5 m는 (1, 1.5)
    xs, ys = beam_points([0.5, 0.5], angle_min=0.0, angle_increment=math.pi / 2,
                         pose=(1.0, 1.0, math.pi / 2))
    assert xs == pytest.approx([1.0, 0.5])
    assert ys == pytest.approx([1.5, 1.0])


def _filter(ranges, band=0.04, range_min=0.05, range_max=3.0):
    # 라이다가 (0.30, 0.89)에서 북쪽을 본다. 빔 하나뿐: 0° 빔이 북쪽 벽(면 0.99)으로 간다
    return filter_ranges(_band(band), ranges, angle_min=0.0, angle_increment=0.1,
                         range_min=range_min, range_max=range_max, pose=(0.30, 0.89, math.pi / 2))


def test_filter_ranges_turns_a_beam_on_the_wall_into_inf_and_keeps_the_rest():
    # 0.075 m: (0.30, 0.965) 벽 곁이라 지운다. 0.02 m: (0.30, 0.91) 벽에서 0.08 m라 둔다
    out = _filter([0.075])
    assert out.tolist() == [math.inf]
    assert _filter([0.02], range_min=0.01).tolist() == [0.02]


def test_filter_ranges_band_edge():
    # 칸 (row 94) 가운데 0.945에서 벽 면 0.99까지 0.045: 0.04 띠 밖, 0.05 띠 안
    assert _filter([0.055], band=0.04).tolist() == [0.055]
    assert _filter([0.055], band=0.05).tolist() == [math.inf]


def test_filter_ranges_leaves_non_finite_beams_alone_without_numpy_warnings():
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        out = _filter([math.inf, math.nan, -math.inf, 0.075])
    assert out[0] == math.inf and math.isnan(out[1]) and out[2] == -math.inf
    assert out[3] == math.inf


def test_filter_ranges_leaves_finite_beams_outside_range_min_max_alone():
    # 0.075는 벽 곁이지만 range_min 0.08보다 짧고, range_max 0.07보다 길다: 그대로 둔다
    assert _filter([0.075], range_min=0.08).tolist() == [0.075]
    assert _filter([0.075], range_max=0.07).tolist() == [0.075]


def test_origin_yaw_reads_the_map_origin_quaternion():
    assert origin_yaw(0.0, 1.0) == 0.0
    assert origin_yaw(math.sin(0.25), math.cos(0.25)) == pytest.approx(0.5)


def test_origin_yaw_of_an_unrotated_map_with_negative_w_is_zero():
    # (z 0, w -1)도 돌지 않은 자세다: 2 pi가 아니라 0
    assert origin_yaw(0.0, -1.0) == pytest.approx(0.0)
    assert -math.pi < origin_yaw(math.sin(2.0), math.cos(2.0)) <= math.pi
