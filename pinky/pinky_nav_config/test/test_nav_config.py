"""Nav2 설정이 핑키 약속을 지키는지 본다. ROS 없이 돈다.

근거: Drive 핑키_FSM_Nav2_구현_안내_2026-11-07(3절), 핑키_제어파트_전달사항_2026-11-06(R4, R12~R15).
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
PARAMS = yaml.safe_load((PKG / "params" / "nav2_params.yaml").read_text(encoding="utf-8"))
BT_FILE = PKG / "bt" / "navigate_to_pose_w_replanning_and_recovery_goal.xml"
BT = ET.parse(BT_FILE)
# Nav2 Jazzy's own tree, the original of BT_FILE (only there when ROS 2 Jazzy is installed)
NAV2_BT = Path("/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/"
               "navigate_to_pose_w_replanning_and_recovery.xml")
KEEPOUT = yaml.safe_load((PKG / "params" / "keepout.yaml").read_text(encoding="utf-8"))


def params(node, sub=None):
    section = PARAMS[node] if sub is None else PARAMS[node][sub]
    return section["ros__parameters"]


def test_controller_uses_rpp_with_the_agreed_start_values():
    c = params("controller_server")
    assert c["controller_plugins"] == ["FollowPath", "FollowPathThrough", "FollowPathReverse",
                                       "FollowPathGoal"]
    assert c["failure_tolerance"] == 1.0
    rpp = c["FollowPath"]
    assert rpp["plugin"] == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    assert rpp["rotate_to_heading_min_angle"] == 0.25  # R12: 올리지 않는다
    assert rpp["use_rotate_to_heading"] is True and rpp["allow_reversing"] is False
    assert (rpp["lookahead_dist"], rpp["min_lookahead_dist"], rpp["max_lookahead_dist"]) == (
        0.15, 0.10, 0.25)
    assert rpp["desired_linear_vel"] == 0.25
    assert rpp["use_collision_detection"] is True
    assert rpp["max_allowed_time_to_collision_up_to_carrot"] == 0.6


def test_the_through_controller_is_followpath_without_the_approach_slow_down():
    """pinky_fsm sends paths that end at a lock boundary (MoveTo.final_goal false) to
    FollowPathThrough. Jazzy RPP always applies the approach constraint, so only a short
    approach_velocity_scaling_dist turns it off; every other key must match FollowPath."""
    c = params("controller_server")
    normal, through = c["FollowPath"], c["FollowPathThrough"]
    assert through["approach_velocity_scaling_dist"] == 0.05
    assert normal["approach_velocity_scaling_dist"] == 0.6
    assert set(through) == set(normal)
    assert {k: v for k, v in through.items() if k != "approach_velocity_scaling_dist"} == {
        k: v for k, v in normal.items() if k != "approach_velocity_scaling_dist"}


REVERSE_ONLY = {"desired_linear_vel": 0.05, "use_rotate_to_heading": False,
                "allow_reversing": True}


def test_the_reverse_controller_is_followpath_driven_backwards_slowly():
    """Retreat with a path(계약 제안): pinky_fsm sends the driven way reversed to
    FollowPathReverse. Jazzy RPP cannot reverse while use_rotate_to_heading is on, so it is off
    (no turn in place before backing) and the robot backs at 0.05 m/s. The collision check stays
    on: it looks along the motion, so it is what stops the robot for something behind."""
    c = params("controller_server")
    normal, reverse = c["FollowPath"], c["FollowPathReverse"]
    assert set(reverse) == set(normal)
    assert {k: reverse[k] for k in REVERSE_ONLY} == REVERSE_ONLY
    assert {k: v for k, v in reverse.items() if k not in REVERSE_ONLY} == {
        k: v for k, v in normal.items() if k not in REVERSE_ONLY}
    assert reverse["use_collision_detection"] is True


def test_no_obstacle_beside_the_path_slows_the_robot():
    """Jazzy RPP scales the speed by cost_scaling_gain * d / cost_scaling_dist when d, the
    distance it reads from the cost under the robot, is below cost_scaling_dist, and d is at
    least the inscribed radius (half the 12 cm footprint) until the robot touches. With 0.6 m
    a box within the 0.15 m inflation pinned the robot to the 0.05 m/s floor in every arena gap
    (user, 2026-11-10): the scaling may no longer cut the speed short of a touch."""
    c = params("controller_server")
    footprint = yaml.safe_load(params("local_costmap", "local_costmap")["footprint"])
    inscribed = min(max(abs(x) for x, _ in footprint), max(abs(y) for _, y in footprint))
    for name in ("FollowPath", "FollowPathThrough"):
        rpp = c[name]
        assert rpp["use_cost_regulated_linear_velocity_scaling"] is True
        assert rpp["cost_scaling_dist"] == 0.05
        assert rpp["cost_scaling_gain"] * inscribed / rpp["cost_scaling_dist"] >= 1.0, name


def test_goal_checker_is_tighter_than_the_arrival_tolerance():
    c = params("controller_server")
    assert c["goal_checker_plugins"] == ["general_goal_checker"]
    goal = c["general_goal_checker"]
    assert goal["xy_goal_tolerance"] == 0.05 and goal["xy_goal_tolerance"] < 0.15  # R4
    assert goal["yaw_goal_tolerance"] == 0.12
    assert c["progress_checker_plugins"] == ["progress_checker"]
    assert c["progress_checker"]["required_movement_radius"] == 0.05


def test_speed_limit_topic_stays_for_version_2():
    assert params("controller_server")["speed_limit_topic"] == "speed_limit"


def test_local_costmap_is_fine_and_sent_whole_for_blocked():
    local = params("local_costmap", "local_costmap")
    assert local["resolution"] == 0.025
    assert local["always_send_full_costmap"] is True  # pinky_fsm이 칸 값으로 BLOCKED를 판단한다
    assert "obstacle_layer" in local["plugins"]


def test_costmap_topics_are_absolute_inside_the_robot_namespace():
    """Jazzy costmap 층은 상대 토픽을 costmap 노드 안으로 풀므로 launch가 절대 이름을 넣는다."""
    local = params("local_costmap", "local_costmap")
    glob = params("global_costmap", "global_costmap")
    assert local["obstacle_layer"]["scan"]["topic"] == "<robot_namespace>/scan"
    assert glob["obstacle_layer"]["scan"]["topic"] == "<robot_namespace>/scan"
    assert glob["static_layer"]["map_topic"] == "<robot_namespace>/map"


def test_global_costmap_has_the_lidar_layer_for_version_2_detours():
    glob = params("global_costmap", "global_costmap")
    assert glob["plugins"] == ["static_layer", "obstacle_layer", "inflation_layer"]


def test_planner_server_runs_for_compute_path_and_is_path_valid():
    assert params("planner_server")["planner_plugins"] == ["GridBased"]


def test_amcl_reads_the_fine_map():
    amcl = params("amcl")
    assert amcl["map_topic"] == "map_fine"  # R15
    assert amcl["sigma_hit"] == 0.05
    assert amcl["base_frame_id"] == "base_footprint" and amcl["scan_topic"] == "scan"


def test_the_goal_controller_is_an_rpp_with_the_collision_check_on():
    """MoveTo mode nav_goal (contract 0.11-draft proposal): the Nav2 tree drives with
    FollowPathGoal. It starts as a copy of FollowPath; a teammate may swap in MPPI or DWB, so only
    the name is fixed. The three FollowPath controllers stay as they were."""
    c = params("controller_server")
    goal = c["FollowPathGoal"]
    assert goal["plugin"] == c["FollowPath"]["plugin"]
    assert goal["use_collision_detection"] is True
    assert goal == c["FollowPath"]


def test_behavior_server_offers_what_the_tree_and_pinky_fsm_call():
    """Nav2's tree recovers with Spin, Wait and BackUp (user decision 2026-11-12, nav_goal only);
    pinky_fsm's follow_path self-recovery and Retreat use backup."""
    b = params("behavior_server")
    assert b["behavior_plugins"] == ["spin", "backup", "wait"]
    assert {name: b[name]["plugin"] for name in b["behavior_plugins"]} == {
        "spin": "nav2_behaviors::Spin", "backup": "nav2_behaviors::BackUp",
        "wait": "nav2_behaviors::Wait"}
    server_of = {"Spin": "spin", "BackUp": "backup", "Wait": "wait"}
    for tag, server in server_of.items():
        for node in BT.getroot().iter(tag):
            assert node.get("server_name", server) in b["behavior_plugins"]


def test_bt_navigator_serves_only_navigate_to_pose():
    assert params("bt_navigator")["navigators"] == ["navigate_to_pose"]


def test_the_tree_replans_at_1_hz_and_recovers_like_nav2s_default():
    tags = [el.tag for el in BT.iter()]
    for kept in ("Spin", "BackUp", "Wait", "ClearEntireCostmap", "RecoveryNode", "RoundRobin"):
        assert kept in tags
    [rate] = BT.getroot().iter("RateController")
    assert float(rate.get("hz")) == 1.0
    [planner] = BT.getroot().iter("ComputePathToPose")
    assert params("planner_server")["planner_plugins"] == ["GridBased"]
    [select] = BT.getroot().iter("PlannerSelector")
    assert select.get("default_planner") == "GridBased" and planner is not None
    [controller] = BT.getroot().iter("ControllerSelector")
    assert controller.get("default_controller") == "FollowPathGoal"


def _canon(tree):
    return [(el.tag, sorted(el.attrib.items())) for el in tree.iter()]


@pytest.mark.skipif(not NAV2_BT.exists(), reason="ROS 2 Jazzy nav2_bt_navigator not installed")
def test_the_tree_is_nav2s_default_tree_with_only_the_goal_controller_changed():
    ours = ET.parse(BT_FILE)
    [select] = ours.getroot().iter("ControllerSelector")
    select.set("default_controller", "FollowPath")
    assert _canon(ours) == _canon(ET.parse(NAV2_BT))


def test_the_old_plan_once_tree_is_gone():
    assert not (PKG / "bt" / "plan_once_no_recovery.xml").exists()


# ---- global costmap for Nav2 goals --------------------------------------------------------

LETHAL, INSCRIBED = 254, 253


def _inflated(free_cells: int, glob: dict) -> list[int]:
    """Nav2 inflation costs across a straight passage of free_cells cells between two wall cells
    (InflationLayer: inscribed within the inscribed radius, then 252 * exp(-k (d - r)))."""
    res = glob["resolution"]
    half = max(abs(v) for pt in yaml.safe_load(glob["footprint"]) for v in pt)
    inscribed = half + glob["footprint_padding"]
    infl = glob["inflation_layer"]
    out = []
    for i in range(1, free_cells + 1):
        d = min(i, free_cells + 1 - i) * res
        if d <= inscribed:
            out.append(INSCRIBED)
        elif d <= infl["inflation_radius"]:
            out.append(int(252 * math.exp(-infl["cost_scaling_factor"] * (d - inscribed))))
        else:
            out.append(0)
    return out


@pytest.mark.parametrize("free_cells", [6, 7])
def test_navfn_finds_a_way_through_a_0_38_m_passage(free_cells):
    """0.38 m on a 5 cm map is 6 or 7 free cells, depending on how the walls fall on the grid.
    NavFn treats cells at or above INSCRIBED (253) as walls; at least one cell must be below and
    the middle must be the cheapest, so goal paths keep to the middle of the passage."""
    glob = params("global_costmap", "global_costmap")
    costs = _inflated(free_cells, glob)
    assert min(costs) < INSCRIBED
    middle = costs[(free_cells - 1) // 2]
    assert middle == min(costs)


def test_the_wide_yard_has_cost_free_room_in_the_middle():
    glob = params("global_costmap", "global_costmap")
    assert 0 in _inflated(int(0.775 / glob["resolution"]), glob)


def test_goal_paths_in_the_yard_keep_away_from_walls_and_shelves():
    """Cells 0.20 m from a wall still cost something, so NavFn keeps goal paths about 0.25 m off
    walls, shelves and seen robots where there is room (the robot edge then clears by ~0.19 m)."""
    glob = params("global_costmap", "global_costmap")
    costs = _inflated(int(0.775 / glob["resolution"]), glob)
    assert costs[3] > 0  # the fourth free cell: 0.20 m from the wall cell


def test_the_local_costmap_that_followpath_uses_is_unchanged():
    local = params("local_costmap", "local_costmap")
    assert local["inflation_layer"] == {"plugin": "nav2_costmap_2d::InflationLayer",
                                        "cost_scaling_factor": 3.0, "inflation_radius": 0.15}
    assert "filters" not in local


# ---- keepout (express lane, dock zones) for Nav2 goals only -------------------------------


def test_the_base_params_have_no_keepout():
    """keepout_mask empty = no filter and no servers: nothing else changes."""
    assert "filters" not in params("global_costmap", "global_costmap")
    assert "costmap_filter_info_server" not in PARAMS and "keepout_mask_server" not in PARAMS


def test_the_keepout_overlay_filters_only_the_global_costmap():
    assert set(KEEPOUT) == {"global_costmap", "costmap_filter_info_server",
                            "keepout_mask_server"}
    glob = KEEPOUT["global_costmap"]["global_costmap"]["ros__parameters"]
    assert glob["filters"] == ["keepout_filter"]
    assert glob["keepout_filter"]["plugin"] == "nav2_costmap_2d::KeepoutFilter"
    assert set(glob) == {"filters", "keepout_filter"}


def test_the_keepout_topics_meet():
    """The costmap filter resolves relative topics inside the costmap node (Jazzy), so the
    filter side names absolute topics; both servers run in the robot namespace."""
    glob = KEEPOUT["global_costmap"]["global_costmap"]["ros__parameters"]
    info = KEEPOUT["costmap_filter_info_server"]["ros__parameters"]
    mask = KEEPOUT["keepout_mask_server"]["ros__parameters"]
    assert glob["keepout_filter"]["filter_info_topic"] == "<robot_namespace>/" + \
        info["filter_info_topic"]
    assert info["mask_topic"] == "<robot_namespace>/" + mask["topic_name"]
    assert info["type"] == 0  # keepout
    assert (info["base"], info["multiplier"]) == (0.0, 1.0)
    assert mask["frame_id"] == "map"


def test_no_velocity_smoother_or_collision_monitor_between_controller_and_cmd_vel():
    assert "velocity_smoother" not in PARAMS and "collision_monitor" not in PARAMS
