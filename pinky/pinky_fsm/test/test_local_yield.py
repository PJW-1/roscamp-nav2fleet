"""로봇단 교착 처리의 순수 판단(추가 지시서 3의 7-4, 기준(사용자)). ROS 없이 돈다."""
import math
from types import SimpleNamespace

import pytest
from pinky_fsm.local_yield import (
    NO_PRIORITY,
    Peer,
    blocking_peer,
    extend_trail,
    peers_from,
    straight_back_m,
    yields_to,
)

AHEAD = [(0.0, 0.0), (0.1, 0.0), (0.2, 0.0), (0.3, 0.0), (0.4, 0.0), (0.5, 0.0)]


def test_a_peer_on_the_way_ahead_is_the_blocker():
    peer = Peer("pinky2", 0.30, 0.05, 1)
    assert blocking_peer(AHEAD, [peer], "pinky1", 0.20) == peer


def test_a_peer_beside_the_way_or_the_robot_itself_is_not_the_blocker():
    assert blocking_peer(AHEAD, [Peer("pinky2", 0.30, 0.40)], "pinky1", 0.20) is None
    assert blocking_peer(AHEAD, [Peer("pinky1", 0.30, 0.0)], "pinky1", 0.20) is None
    assert blocking_peer([], [Peer("pinky2", 0.0, 0.0)], "pinky1", 0.20) is None


def test_the_nearest_of_two_peers_ahead_is_the_blocker():
    far, near = Peer("pinky3", 0.45, 0.0), Peer("pinky2", 0.25, 0.0)
    assert blocking_peer(AHEAD, [far, near], "pinky1", 0.20) == near


@pytest.mark.parametrize("mine, theirs, me, other, backs", [
    (0, 2, "pinky1", "pinky2", True),  # 낮은 쪽이 물러난다
    (2, 0, "pinky1", "pinky2", False),
    (2, 0, "pinky3", "pinky2", False),  # 우선순위가 이름보다 먼저다
    (1, 1, "pinky1", "pinky2", True),  # 같으면 이름 순: 앞 이름이 물러난다(관제 choose_retreat)
    (1, 1, "pinky2", "pinky1", False),
    (NO_PRIORITY, 0, "pinky1", "pinky2", True),  # 하나라도 비면 이름 순
    (0, NO_PRIORITY, "pinky2", "pinky1", False),
])
def test_only_the_lower_side_backs_up(mine, theirs, me, other, backs):
    assert yields_to(me, mine, Peer(other, 0.0, 0.0, theirs)) is backs


@pytest.mark.parametrize("a, b", [((0, 2), (2, 0)), ((1, 1), (1, 1)), ((-1, 3), (3, -1))])
def test_two_robots_facing_each_other_agree_on_who_backs_up(a, b):
    one = yields_to("pinky1", a[0], Peer("pinky2", 0.0, 0.0, a[1]))
    two = yields_to("pinky2", b[0], Peer("pinky1", 0.0, 0.0, b[1]))
    assert one != two


def test_backing_straight_on_a_straight_trail_goes_up_to_the_limit():
    trail = [(x / 100, 0.0) for x in range(0, 41, 2)]  # 0 → 0.40 m 동쪽
    assert straight_back_m(trail, (0.40, 0.0, 0.0), 0.10, 0.03) == pytest.approx(0.10)


def test_backing_straight_stops_where_the_trail_bends_away():
    # 북쪽으로 0.30 m 와서 동쪽으로 0.06 m 꺾였다. 동쪽을 보고 곧게 물러나면 꺾인 곳까지만
    trail = [(0.0, y / 100) for y in range(0, 31, 2)] + [(0.02, 0.30), (0.04, 0.30),
                                                          (0.06, 0.30)]
    d = straight_back_m(trail, (0.06, 0.30, 0.0), 0.10, 0.03)
    assert 0.05 <= d <= 0.09


def test_backing_with_no_trail_or_facing_across_the_trail_is_nothing():
    assert straight_back_m([], (0.0, 0.0, 0.0), 0.10, 0.03) == 0.0
    trail = [(x / 100, 0.0) for x in range(0, 41, 2)]
    assert straight_back_m(trail, (0.40, 0.0, math.pi / 2), 0.10, 0.03) == pytest.approx(0.0)


def test_fleet_rows_become_peers_and_rows_without_priority_have_none():
    rows = [SimpleNamespace(robot_id="pinky2", x=1.0, y=2.0, priority=3),
            SimpleNamespace(robot_id="pinky3", x=0.5, y=0.5)]  # 0.13 전 메시지
    assert peers_from(rows) == [Peer("pinky2", 1.0, 2.0, 3), Peer("pinky3", 0.5, 0.5, NO_PRIORITY)]


def test_the_trail_keeps_only_its_last_part_and_counts_all_of_it():
    trail = []
    driven = 0.0
    for i in range(51):
        driven += extend_trail(trail, (i / 50, 0.0), 0.015, 0.5)
    assert driven == pytest.approx(1.0)
    assert trail[-1] == (1.0, 0.0) and trail[0][0] >= 0.47
    assert extend_trail(trail, (1.005, 0.0), 0.02, 0.5) == 0.0  # 너무 가깝다
