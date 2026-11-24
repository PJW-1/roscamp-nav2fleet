"""후진 기본 동작(backup.py): 얼마나 물러나도 되는지와 결과의 뜻. ROS는 마지막 시험에서만 필요하다."""
import pytest
from pode_fsm.backup import (
    CANCELED,
    FAILED,
    NAV2_COLLISION_AHEAD,
    NAV2_NO_VALID_CONTROL,
    NAV2_TIMEOUT,
    OBSTACLE_BEHIND,
    TIMEOUT,
    backup_distance,
    keep_lateral,
    reason_of,
)


def test_distance_is_capped_at_the_maximum():
    assert backup_distance(1.0, max_m=0.15, keep_m=0.23) == pytest.approx(0.15)


def test_distance_keeps_the_centre_clear_of_the_last_vertex():
    """후진 거리는 마지막으로 지난 꼭짓점(경로 시작 포함)부터 센다. 꼭짓점에서 release_distance_m(0.2) 넘게 가면 관제가 뒤 레인을 놓는다."""
    assert backup_distance(0.32, max_m=0.15, keep_m=0.23) == pytest.approx(0.09)


def test_no_distance_close_to_the_last_vertex():
    assert backup_distance(0.20, max_m=0.15, keep_m=0.23) == 0.0
    assert backup_distance(0.0, max_m=0.15, keep_m=0.23) == 0.0


def test_lateral_limit_leaves_a_back_up_toward_the_path_alone():
    # 경로 왼쪽 0.05 m, 경로의 왼쪽 30도로 향함: 후진하면 경로 쪽으로 간다
    assert keep_lateral(0.15, lateral_m=0.05, heading_err=0.5236, limit_m=0.10) == \
        pytest.approx(0.15)


def test_lateral_limit_shortens_a_back_up_away_from_the_path():
    # 왼쪽 0.05 m, 경로의 오른쪽 30도로 향함: 후진하면 거리의 절반만큼 왼쪽으로 밀린다
    assert keep_lateral(0.15, lateral_m=0.05, heading_err=-0.5236, limit_m=0.10) == \
        pytest.approx(0.10, abs=1e-3)


def test_lateral_limit_never_makes_an_offset_beyond_the_limit_worse():
    assert keep_lateral(0.15, lateral_m=0.12, heading_err=-0.5236, limit_m=0.10) == 0.0
    # 한계 밖이어도 경로 쪽으로 물러나면 된다
    assert keep_lateral(0.15, lateral_m=0.12, heading_err=0.5236, limit_m=0.10) == \
        pytest.approx(0.15)


def test_lateral_limit_with_a_straight_heading_changes_nothing():
    assert keep_lateral(0.15, lateral_m=0.09, heading_err=0.0, limit_m=0.10) == \
        pytest.approx(0.15)


def test_reasons_follow_the_version_2_retreat_result():
    assert reason_of("succeeded", 0) == ""
    assert reason_of("canceled", 0) == CANCELED
    assert reason_of("aborted", NAV2_COLLISION_AHEAD) == OBSTACLE_BEHIND
    assert reason_of("aborted", NAV2_TIMEOUT) == TIMEOUT
    # 버전 2 목록에 없음: 서버 없음, TF 오류, 잘못된 입력
    assert reason_of("rejected", 0) == FAILED
    assert reason_of("aborted", 712) == FAILED


def test_a_reverse_follow_path_stopped_by_its_collision_check_is_an_obstacle_behind():
    """Retreat.path: 후진 방향 RPP 충돌 검사는 NO_VALID_CONTROL로 끝나고, 다른 FollowPath 실패(진전 없음, TF)는 FAILED다."""
    assert reason_of("aborted", NAV2_NO_VALID_CONTROL) == OBSTACLE_BEHIND
    assert reason_of("aborted", 105) == FAILED


def test_error_codes_match_nav2():
    action = pytest.importorskip("nav2_msgs.action")
    assert NAV2_TIMEOUT == action.BackUp.Result.TIMEOUT
    assert NAV2_COLLISION_AHEAD == action.BackUp.Result.COLLISION_AHEAD
    assert NAV2_NO_VALID_CONTROL == action.FollowPath.Result.NO_VALID_CONTROL
