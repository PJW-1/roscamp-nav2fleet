"""한 줄 구역의 방향: 먼저 닿는 쪽, 직전 방향의 히스테리시스, 오래 기다린 쪽에 넘기기."""
from fleet_core.common.params import load_params
from fleet_core.fleet_map import load_fleet_map, route
from fleet_core.traffic.zone.controller import ZoneTraffic
from fleet_core.traffic.zone.lane_policy import LanePolicy, Want
from fleet_core.traffic.zone.trips import Trip

LANE = "EXPRESS"
P = load_params()


def want(direction, eta, since=0.0):
    return Want((LANE,), (), {LANE: direction}, since, {LANE: eta})


def test_with_no_rival_it_may_enter():
    assert LanePolicy(P).refusal("pinky1", LANE, want("X_W", 9.0), 0.0, {}) == ""


def test_the_one_that_reaches_the_entrance_first_enters():
    policy = LanePolicy(P)
    west, east = want("X_W", 3.0), want("X_E", 5.0)
    assert policy.refusal("pinky1", LANE, west, 0.0, {"pinky2": east}) == ""
    assert policy.refusal("pinky2", LANE, east, 0.0, {"pinky1": west}).startswith("direction")


def test_a_tie_goes_to_the_first_name():
    policy = LanePolicy(P)
    west, east = want("X_W", 4.0), want("X_E", 4.0)
    assert policy.refusal("pinky1", LANE, west, 0.0, {"pinky2": east}) == ""
    assert policy.refusal("pinky2", LANE, east, 0.0, {"pinky1": west}) != ""


def test_the_last_direction_is_kept_unless_the_other_side_is_much_earlier():
    policy = LanePolicy(P)
    assert policy.note(LANE, "X_W") is False, "the first run changes nothing"
    slack = P.lane_hysteresis_s
    west = want("X_W", 6.0)
    near, far = want("X_E", 6.0 - slack + 0.5), want("X_E", 6.0 - slack - 0.5)
    assert policy.refusal("pinky1", LANE, west, 0.0, {"pinky2": near}) == ""
    assert policy.refusal("pinky2", LANE, near, 0.0, {"pinky1": west}) != ""
    assert policy.refusal("pinky1", LANE, west, 0.0, {"pinky2": far}) != ""
    assert policy.refusal("pinky2", LANE, far, 0.0, {"pinky1": west}) == ""
    assert policy.note(LANE, "X_E") is True


def test_a_side_that_waited_long_gets_the_lane_before_a_newcomer():
    policy = LanePolicy(P)
    policy.note(LANE, "X_W")
    now = P.lane_handover_s + 1.0
    waiting, newcomer = want("X_E", 20.0, since=0.0), want("X_W", 1.0, since=now)
    assert policy.refusal("pinky1", LANE, newcomer, now, {"pinky2": waiting}).startswith("handover")
    assert policy.refusal("pinky2", LANE, waiting, now, {"pinky1": newcomer}) == "", \
        "by ETA and the last direction it would lose, the long wait wins"
    assert policy.handed_over(waiting, now) and not policy.handed_over(newcomer, now)


# ---- 제어기 안에서: 급행을 서로 반대로 바라는 두 대 ----

def across(site, robot_at, *nodes):
    """nodes를 차례로 지나는 길. 시험이 급행을 지나는 길을 손으로 고른다."""
    links = []
    for a, b in zip((robot_at, *nodes), nodes, strict=False):
        links += route(site, a, b)
    return links


def traffic_with_two():
    site = load_fleet_map("d2")
    traffic = ZoneTraffic(site, P, ["pinky1", "pinky3"], lambda: 10.0)
    traffic.on_robot_state("pinky1", *site.node_pose("P-1"), 0.0, "IDLE")
    traffic.on_robot_state("pinky3", *site.node_pose("P-3"), 0.0, "IDLE")
    robots = traffic._robots
    east = Trip("O_Q", across(site, "P-1", "N_W", "X_W", "X_E", "N_E", "O_Q"))
    west = Trip("W_Q", across(site, "P-3", "N_E", "X_E", "X_W", "N_W", "W_Q"))
    return traffic, robots["pinky1"], east, robots["pinky3"], west


def test_the_nearer_robot_takes_the_express_and_the_other_waits_outside():
    traffic, one, east, three, west = traffic_with_two()
    refused = traffic._take(three, west)
    assert refused is None, "alone it takes the whole stretch"
    assert traffic.grants.lane_direction(LANE) == "X_E"
    blocked = traffic._take(one, east)
    assert blocked is not None and traffic.grants.held_by("pinky1") == ("HALL",)
    assert blocked.lanes == {LANE: "X_W"}
    codes = [e.code for e in traffic.take_output()]
    assert codes == ["LANE_DIRECTION_SET", "ZONE_GRANTED"]


def test_a_rival_that_only_needs_the_lane_wins_when_it_is_nearer():
    traffic, one, east, three, west = traffic_with_two()
    traffic._wants["pinky3"] = Want(("HALL", "J_E", LANE, "J_W"), ("N_E", "X_E", "X_W", "N_W"),
                                    {LANE: "X_E"}, 10.0, {LANE: 2.0})
    refused = traffic._take(one, east)
    assert refused is not None, "pinky3 reaches its end of the express sooner"
    assert traffic.grants.holders(LANE) == ()
