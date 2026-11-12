"""BLOCKED의 장애물 위치와 CheckClear의 답(계약 0.12-draft). ROS 없이 돈다."""
import math

from pinky_fsm.costmap import Grid
from pinky_fsm.obstacle_report import BLOCKED, CLEAR, UNKNOWN, clear_answer, nearest_ahead

RES = 0.05


def grid(cells=None, origin=(0.0, 0.0), yaw=0.0, fill=0):
    data = [fill] * (40 * 40)
    for (i, j), v in (cells or {}).items():
        data[j * 40 + i] = v
    return Grid(resolution=RES, origin_x=origin[0], origin_y=origin[1], origin_yaw=yaw,
                width=40, height=40, data=data)


def center(i, j):
    return ((i + 0.5) * RES, (j + 0.5) * RES)


def test_the_nearest_lethal_cell_ahead_is_reported():
    g = grid({(14, 10): 100, (16, 10): 100})
    assert nearest_ahead(g, 0.50, 0.525, 0.0, 0.30) == center(14, 10)


def test_cells_behind_beside_or_beyond_the_look_are_not_reported():
    g = grid({(6, 10): 100, (10, 16): 100, (18, 10): 100, (12, 10): 99})
    # 뒤(x 0.325), 옆(y 0.825), look 밖(x 0.925), 부풀린 둘레(99)
    assert nearest_ahead(g, 0.50, 0.525, 0.0, 0.30) is None


def test_the_heading_decides_what_is_ahead():
    g = grid({(10, 14): 100})
    assert nearest_ahead(g, 0.525, 0.50, math.pi / 2, 0.30) == center(10, 14)
    assert nearest_ahead(g, 0.525, 0.50, -math.pi / 2, 0.30) is None


def test_a_rotated_grid_reports_in_the_costmap_frame():
    g = grid({(4, 0): 100}, origin=(1.0, 1.0), yaw=math.pi / 2)
    x, y = nearest_ahead(g, 0.975, 1.0, math.pi / 2, 0.30)
    assert math.isclose(x, 0.975, abs_tol=1e-9) and math.isclose(y, 1.225, abs_tol=1e-9)


def test_clear_blocked_and_unknown():
    g = grid({(20, 20): 100})
    assert clear_answer(g, *center(21, 20), 0.10) == BLOCKED
    assert clear_answer(g, *center(25, 20), 0.10) == CLEAR
    assert clear_answer(g, 5.0, 5.0, 0.10) == UNKNOWN, "outside the costmap"
    assert clear_answer(grid(fill=-1), *center(20, 20), 0.10) == UNKNOWN, "nothing seen there"


def test_clear_only_when_the_whole_circle_was_seen():
    """원 안에 모르는 칸이 하나라도 있거나 원의 일부가 격자 밖이면 clear라고 하지 않는다."""
    assert clear_answer(grid({(22, 20): -1}), *center(20, 20), 0.10) == UNKNOWN, "an unseen cell"
    assert clear_answer(grid(), *center(0, 20), 0.10) == UNKNOWN, "partly off the costmap"
    assert clear_answer(grid({(22, 20): -1, (19, 20): 100}), *center(20, 20), 0.10) == BLOCKED


def test_a_stale_costmap_answers_unknown():
    g = grid({(20, 20): 100})
    assert clear_answer(g, *center(25, 20), 0.10, age_s=1.0) == CLEAR
    assert clear_answer(g, *center(25, 20), 0.10, age_s=2.5) == UNKNOWN
    assert clear_answer(g, *center(21, 20), 0.10, age_s=2.5) == UNKNOWN, "old news either way"
