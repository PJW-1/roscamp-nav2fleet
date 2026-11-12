"""구역 허락 기록: 정원, 지점, 한 줄 구역의 방향, 한꺼번에 잡기."""
import pytest
from fleet_core.fleet_map import load_fleet_map
from fleet_core.traffic.zone.grants import ZoneGrants

D2 = load_fleet_map("d2")
D = load_fleet_map("d")


def make(site=D2, gap=4.0):
    now = [0.0]
    return ZoneGrants(site, lambda: now[0], follow_gap_s=gap), now


def test_a_free_stretch_is_taken_whole():
    grants, _ = make()
    assert grants.take("pinky1", zones=["J_W", "WORK_DOCK"], nodes=["N_W", "X_W", "W"])
    assert grants.holders("J_W") == ("pinky1",) and grants.holders("WORK_DOCK") == ("pinky1",)
    assert grants.held_by("pinky1") == ("J_W", "WORK_DOCK")
    assert grants.nodes_of("pinky1") == ("N_W", "X_W", "W")
    assert grants.node_holder("X_W") == "pinky1" and grants.node_holder("O") == ""


def test_a_stretch_with_one_busy_zone_takes_nothing_and_says_who_is_in_the_way():
    grants, _ = make()
    grants.take("pinky1", zones=["WORK_DOCK"], nodes=["W"])
    assert not grants.take("pinky2", zones=["J_W", "WORK_DOCK"], nodes=["N_W", "X_W", "W"])
    assert grants.held_by("pinky2") == () and grants.nodes_of("pinky2") == ()
    assert grants.holders("J_W") == ()
    assert grants.blockers("pinky2", zones=["J_W", "WORK_DOCK"], nodes=["X_W", "W"]) == {
        "WORK_DOCK": ("pinky1",), "W": ("pinky1",)}
    assert grants.waiting() == {"pinky2": ("WORK_DOCK", "W")}


def test_waiting_ends_when_the_stretch_is_taken_or_given_up():
    grants, _ = make()
    grants.take("pinky1", zones=["WORK_DOCK"])
    grants.take("pinky2", zones=["WORK_DOCK"])
    grants.take("pinky3", zones=["WORK_DOCK"])
    grants.stop_waiting("pinky3")
    assert grants.waiting() == {"pinky2": ("WORK_DOCK",)}
    grants.release_zone("pinky1", "WORK_DOCK")
    assert grants.take("pinky2", zones=["WORK_DOCK"])
    assert grants.waiting() == {}


def test_a_shared_zone_takes_robots_up_to_its_capacity():
    grants, _ = make(D)
    assert grants.take("pinky1", zones=["HALL_W"]) and grants.take("pinky2", zones=["HALL_W"])
    assert grants.take("pinky3", zones=["HALL_W"])
    assert grants.holders("HALL_W") == ("pinky1", "pinky2", "pinky3")
    assert grants.take("pinky1", zones=["HALL_W", "CORR_M", "HALL_E"])
    assert not grants.take("pinky2", zones=["HALL_W", "CORR_M", "HALL_E"])
    assert grants.blockers("pinky2", zones=["CORR_M"]) == {"CORR_M": ("pinky1",)}


def test_zones_and_nodes_a_robot_already_holds_do_not_stand_in_its_own_way():
    grants, _ = make()
    grants.take("pinky1", zones=["WORK_DOCK"], nodes=["W"])
    assert grants.take("pinky1", zones=["WORK_DOCK", "J_W"], nodes=["W", "X_W"])
    assert grants.held_by("pinky1") == ("WORK_DOCK", "J_W")
    assert grants.holders("WORK_DOCK") == ("pinky1",)


def test_a_node_holds_one_robot():
    grants, _ = make()
    assert grants.take("pinky1", zones=["HALL"], nodes=["W_Q"])
    assert not grants.take("pinky2", zones=["HALL"], nodes=["W_Q"])
    assert grants.holders("HALL") == ("pinky1",), "the hall itself had room, but nothing is taken"
    grants.release_node("pinky1", "W_Q")
    assert grants.take("pinky2", zones=["HALL"], nodes=["W_Q"])


def test_a_lane_runs_one_way_at_a_time():
    grants, now = make()
    assert grants.take("pinky1", zones=["EXPRESS"], lanes={"EXPRESS": "X_W"})
    assert grants.lane_direction("EXPRESS") == "X_W"
    assert not grants.take("pinky2", zones=["EXPRESS"], lanes={"EXPRESS": "X_E"})
    now[0] = 100.0
    assert not grants.take("pinky2", zones=["EXPRESS"], lanes={"EXPRESS": "X_E"})
    grants.release_zone("pinky1", "EXPRESS")
    assert grants.lane_direction("EXPRESS") == ""
    assert grants.take("pinky2", zones=["EXPRESS"], lanes={"EXPRESS": "X_E"})
    assert grants.lane_direction("EXPRESS") == "X_E"


def test_a_follower_enters_a_lane_the_same_way_after_a_gap_and_up_to_its_capacity():
    grants, now = make(gap=4.0)
    grants.take("pinky1", zones=["EXPRESS"], lanes={"EXPRESS": "X_W"})
    now[0] = 3.9
    assert not grants.take("pinky2", zones=["EXPRESS"], lanes={"EXPRESS": "X_W"})
    assert grants.blockers("pinky2", zones=["EXPRESS"], lanes={"EXPRESS": "X_W"}) == {
        "EXPRESS": ("pinky1",)}
    now[0] = 4.0
    assert grants.take("pinky2", zones=["EXPRESS"], lanes={"EXPRESS": "X_W"})
    assert grants.holders("EXPRESS") == ("pinky1", "pinky2")
    now[0] = 100.0
    assert not grants.take("pinky3", zones=["EXPRESS"], lanes={"EXPRESS": "X_W"}), "capacity 2"


def test_a_lane_zone_needs_its_direction():
    grants, _ = make()
    with pytest.raises(ValueError, match="direction"):
        grants.take("pinky1", zones=["EXPRESS"])


def test_release_gives_back_one_zone_or_node():
    grants, _ = make()
    grants.take("pinky1", zones=["HALL", "J_W"], nodes=["N_W", "X_W"])
    assert grants.release_zone("pinky1", "J_W") is True
    assert grants.release_zone("pinky1", "J_W") is False
    assert grants.release_node("pinky2", "N_W") is False
    assert grants.release_zone("pinky1", "HALL") is True
    assert grants.release_node("pinky1", "N_W") is True
    assert grants.release_node("pinky1", "X_W") is True
    assert grants.held_by("pinky1") == () and grants.nodes_of("pinky1") == ()
    assert grants.holders("HALL") == ()


def test_force_places_a_robot_only_where_there_is_room():
    grants, _ = make()
    assert grants.force("pinky1", zones=["WORK_DOCK"], nodes=["W"])
    assert not grants.force("pinky2", zones=["WORK_DOCK"], nodes=["W"])
    assert grants.held_by("pinky2") == ()
    assert grants.waiting() == {}, "placing is not waiting"


def test_acquired_at_is_when_the_longest_holder_got_the_zone():
    grants, now = make()
    assert grants.acquired_at("HALL") is None
    now[0] = 2.0
    grants.take("pinky1", zones=["HALL"])
    now[0] = 5.0
    grants.take("pinky2", zones=["HALL"])
    assert grants.acquired_at("HALL") == 2.0
    grants.release_zone("pinky1", "HALL")
    assert grants.acquired_at("HALL") == 5.0


def test_an_unknown_zone_or_node_is_a_programming_error():
    grants, _ = make()
    with pytest.raises(KeyError):
        grants.take("pinky1", zones=["NOWHERE"])
    with pytest.raises(KeyError):
        grants.take("pinky1", nodes=["NOWHERE"])
