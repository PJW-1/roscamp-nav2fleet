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
    assert grants.take("pode1", zones=["J_W", "WORK_DOCK"], nodes=["N_W", "X_W", "W"])
    assert grants.holders("J_W") == ("pode1",) and grants.holders("WORK_DOCK") == ("pode1",)
    assert grants.held_by("pode1") == ("J_W", "WORK_DOCK")
    assert grants.nodes_of("pode1") == ("N_W", "X_W", "W")
    assert grants.node_holder("X_W") == "pode1" and grants.node_holder("O") == ""


def test_a_stretch_with_one_busy_zone_takes_nothing_and_says_who_is_in_the_way():
    grants, _ = make()
    grants.take("pode1", zones=["WORK_DOCK"], nodes=["W"])
    assert not grants.take("pode2", zones=["J_W", "WORK_DOCK"], nodes=["N_W", "X_W", "W"])
    assert grants.held_by("pode2") == () and grants.nodes_of("pode2") == ()
    assert grants.holders("J_W") == ()
    assert grants.blockers("pode2", zones=["J_W", "WORK_DOCK"], nodes=["X_W", "W"]) == {
        "WORK_DOCK": ("pode1",), "W": ("pode1",)}
    assert grants.waiting() == {"pode2": ("WORK_DOCK", "W")}


def test_waiting_ends_when_the_stretch_is_taken_or_given_up():
    grants, _ = make()
    grants.take("pode1", zones=["WORK_DOCK"])
    grants.take("pode2", zones=["WORK_DOCK"])
    grants.take("pode3", zones=["WORK_DOCK"])
    grants.stop_waiting("pode3")
    assert grants.waiting() == {"pode2": ("WORK_DOCK",)}
    grants.release_zone("pode1", "WORK_DOCK")
    assert grants.take("pode2", zones=["WORK_DOCK"])
    assert grants.waiting() == {}


def test_a_shared_zone_takes_robots_up_to_its_capacity():
    grants, _ = make(D)
    assert grants.take("pode1", zones=["HALL_W"]) and grants.take("pode2", zones=["HALL_W"])
    assert grants.take("pode3", zones=["HALL_W"])
    assert grants.holders("HALL_W") == ("pode1", "pode2", "pode3")
    assert grants.take("pode1", zones=["HALL_W", "CORR_M", "HALL_E"])
    assert not grants.take("pode2", zones=["HALL_W", "CORR_M", "HALL_E"])
    assert grants.blockers("pode2", zones=["CORR_M"]) == {"CORR_M": ("pode1",)}


def test_zones_and_nodes_a_robot_already_holds_do_not_stand_in_its_own_way():
    grants, _ = make()
    grants.take("pode1", zones=["WORK_DOCK"], nodes=["W"])
    assert grants.take("pode1", zones=["WORK_DOCK", "J_W"], nodes=["W", "X_W"])
    assert grants.held_by("pode1") == ("WORK_DOCK", "J_W")
    assert grants.holders("WORK_DOCK") == ("pode1",)


def test_a_node_holds_one_robot():
    grants, _ = make()
    assert grants.take("pode1", zones=["HALL"], nodes=["W_Q"])
    assert not grants.take("pode2", zones=["HALL"], nodes=["W_Q"])
    assert grants.holders("HALL") == ("pode1",), "the hall itself had room, but nothing is taken"
    grants.release_node("pode1", "W_Q")
    assert grants.take("pode2", zones=["HALL"], nodes=["W_Q"])


def test_a_lane_runs_one_way_at_a_time():
    grants, now = make()
    assert grants.take("pode1", zones=["EXPRESS"], lanes={"EXPRESS": "X_W"})
    assert grants.lane_direction("EXPRESS") == "X_W"
    assert not grants.take("pode2", zones=["EXPRESS"], lanes={"EXPRESS": "X_E"})
    now[0] = 100.0
    assert not grants.take("pode2", zones=["EXPRESS"], lanes={"EXPRESS": "X_E"})
    grants.release_zone("pode1", "EXPRESS")
    assert grants.lane_direction("EXPRESS") == ""
    assert grants.take("pode2", zones=["EXPRESS"], lanes={"EXPRESS": "X_E"})
    assert grants.lane_direction("EXPRESS") == "X_E"


def test_a_follower_enters_a_lane_the_same_way_after_a_gap_and_up_to_its_capacity():
    grants, now = make(gap=4.0)
    grants.take("pode1", zones=["EXPRESS"], lanes={"EXPRESS": "X_W"})
    now[0] = 3.9
    assert not grants.take("pode2", zones=["EXPRESS"], lanes={"EXPRESS": "X_W"})
    assert grants.blockers("pode2", zones=["EXPRESS"], lanes={"EXPRESS": "X_W"}) == {
        "EXPRESS": ("pode1",)}
    now[0] = 4.0
    assert grants.take("pode2", zones=["EXPRESS"], lanes={"EXPRESS": "X_W"})
    assert grants.holders("EXPRESS") == ("pode1", "pode2")
    now[0] = 100.0
    assert not grants.take("pode3", zones=["EXPRESS"], lanes={"EXPRESS": "X_W"}), "capacity 2"


def test_a_lane_zone_needs_its_direction():
    grants, _ = make()
    with pytest.raises(ValueError, match="direction"):
        grants.take("pode1", zones=["EXPRESS"])


def test_release_gives_back_one_zone_or_node():
    grants, _ = make()
    grants.take("pode1", zones=["HALL", "J_W"], nodes=["N_W", "X_W"])
    assert grants.release_zone("pode1", "J_W") is True
    assert grants.release_zone("pode1", "J_W") is False
    assert grants.release_node("pode2", "N_W") is False
    assert grants.release_zone("pode1", "HALL") is True
    assert grants.release_node("pode1", "N_W") is True
    assert grants.release_node("pode1", "X_W") is True
    assert grants.held_by("pode1") == () and grants.nodes_of("pode1") == ()
    assert grants.holders("HALL") == ()


def test_force_places_a_robot_only_where_there_is_room():
    grants, _ = make()
    assert grants.force("pode1", zones=["WORK_DOCK"], nodes=["W"])
    assert not grants.force("pode2", zones=["WORK_DOCK"], nodes=["W"])
    assert grants.held_by("pode2") == ()
    assert grants.waiting() == {}, "placing is not waiting"


def test_acquired_at_is_when_the_longest_holder_got_the_zone():
    grants, now = make()
    assert grants.acquired_at("HALL") is None
    now[0] = 2.0
    grants.take("pode1", zones=["HALL"])
    now[0] = 5.0
    grants.take("pode2", zones=["HALL"])
    assert grants.acquired_at("HALL") == 2.0
    grants.release_zone("pode1", "HALL")
    assert grants.acquired_at("HALL") == 5.0


def test_an_unknown_zone_or_node_is_a_programming_error():
    grants, _ = make()
    with pytest.raises(KeyError):
        grants.take("pode1", zones=["NOWHERE"])
    with pytest.raises(KeyError):
        grants.take("pode1", nodes=["NOWHERE"])
