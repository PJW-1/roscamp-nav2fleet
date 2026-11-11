"""order model helpers without ROS: the horizon pre-check and the marker lines."""
import pytest
from pinky_fsm.costmap import Grid
from pinky_fsm.order_check import segment_problem
from pinky_fsm.order_markers import dashes, order_lines
from pinky_fsm.order_queue import MODE_FOLLOW_PATH, MODE_NAV_GOAL, MODE_RETREAT, Seg

# 2.0 x 1.0 m map at 0.1 m; the keepout mask blocks the strip y >= 0.8 and the column x 1.0..1.1
W, H, RES = 20, 10, 0.1
MAP = Grid(RES, 0.0, 0.0, 0.0, W, H, [0] * (W * H))
KEEPOUT = Grid(RES, 0.0, 0.0, 0.0, W, H,
               [100 if j >= 8 or i == 10 else 0 for j in range(H) for i in range(W)])


def path_seg(seq, *xy, mode=MODE_FOLLOW_PATH):
    return Seg(seq, mode, "FollowPath", tuple((x, y, 0.0) for x, y in xy), None, f"N{seq}")


def goal_seg(seq, x, y):
    return Seg(seq, MODE_NAV_GOAL, "", (), (x, y, 0.0), f"N{seq}")


def test_a_segment_inside_the_map_and_outside_keepout_is_fine():
    assert segment_problem(path_seg(1, (0.2, 0.3), (0.9, 0.3)), MAP, KEEPOUT) == ""


def test_a_path_leaving_the_map_is_reported():
    assert "outside the map" in segment_problem(path_seg(1, (0.2, 0.3), (2.4, 0.3)), MAP, None)


def test_a_path_crossing_keepout_between_its_points_is_reported():
    why = segment_problem(path_seg(1, (0.2, 0.3), (1.5, 0.3)), MAP, KEEPOUT)
    assert "keepout" in why


def test_a_nav_goal_checks_only_its_target_pose():
    assert segment_problem(goal_seg(1, 0.5, 0.5), MAP, KEEPOUT) == ""
    assert "keepout" in segment_problem(goal_seg(1, 1.0, 0.9), MAP, KEEPOUT)
    assert "outside the map" in segment_problem(goal_seg(1, 3.0, 0.5), MAP, KEEPOUT)


def test_no_map_and_no_mask_means_no_check():
    assert segment_problem(goal_seg(1, 9.0, 9.0), None, None) == ""


def test_base_lines_are_solid_and_horizon_lines_dashed():
    base = [path_seg(1, (0.2, 0.3), (0.8, 0.3)), goal_seg(2, 1.2, 0.6)]
    horizon = [path_seg(3, (1.2, 0.6), (1.2, 0.2), mode=MODE_RETREAT)]
    lines = order_lines((0.1, 0.3), base, horizon)
    assert [(ln.seq_id, ln.horizon) for ln in lines] == [(1, False), (2, False), (3, True)]
    assert lines[1].points == ((0.8, 0.3), (1.2, 0.6))  # a nav_goal: from the previous end
    assert lines[2].points == ((1.2, 0.6), (1.2, 0.2))


def test_a_first_nav_goal_starts_at_the_robot():
    [ln] = order_lines((0.1, 0.3), [goal_seg(1, 1.2, 0.6)], [])
    assert ln.points == ((0.1, 0.3), (1.2, 0.6))


def test_dashes_cut_a_line_into_pieces_with_gaps():
    pieces = dashes(((0.0, 0.0), (1.0, 0.0)), dash_m=0.1, gap_m=0.1)
    assert len(pieces) == 5
    assert [*pieces[0][0], *pieces[0][1]] == pytest.approx([0.0, 0.0, 0.1, 0.0])
    assert pieces[1][0] == pytest.approx((0.2, 0.0))
