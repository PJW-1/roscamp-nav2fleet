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
    assert rpp["use_rotate_to_heading"] is True and rpp["allow_reversing"] is False
    assert (rpp["lookahead_dist"], rpp["min_lookahead_dist"], rpp["max_lookahead_dist"]) == (
        0.10, 0.10, 0.25)
    assert rpp["desired_linear_vel"] == 0.25
    assert rpp["use_collision_detection"] is True
    assert rpp["max_allowed_time_to_collision_up_to_carrot"] == 0.6


# 추가 지시서 3의 2-1. pinky_fsm turn_radius_m: 꺾이는 곳을 이 반지름의 호로 보낸다
TURN_RADIUS_M = 0.10
# pinky_description(pinky_pro 014a09f) base_link 충돌 메시의 바닥 윤곽(볼록 껍질, base_footprint
# 틀, m)과 바퀴(축 y ±0.04055에서 바깥으로 0.0145 m, 반지름 0.028 m)
BODY_OUTLINE = [(0.0387, 0.0197), (0.0364, 0.0274), (0.0177, 0.0391), (-0.0447, 0.0473),
                (-0.0700, 0.0399), (-0.0734, 0.0378), (-0.0745, 0.0347), (-0.0745, -0.0349),
                (-0.0734, -0.0378), (-0.0700, -0.0399), (0.0184, -0.0386), (0.0363, -0.0276),
                (0.0387, -0.0192)]
WHEELS = [(x, y) for x in (-0.028, 0.028) for y in (-0.055, -0.04055, 0.04055, 0.055)]


def _inside(polygon, x, y) -> bool:
    """점이 다각형 안(경계 포함)에 있다. 볼록이 아니어도 된다(짝홀 규칙)."""
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1], strict=True):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1) + 1e-9:
            inside = not inside
    return inside


def _area(polygon) -> float:
    return abs(sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2)
                   in zip(polygon, polygon[1:] + polygon[:1], strict=True))) / 2


def _hull(points) -> list:
    """볼록 껍질(monotone chain), 반시계 방향."""
    pts = sorted(set(points))

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]
    return half(pts) + half(pts[::-1])


def _edge_gap(polygon) -> float:
    """원점에서 다각형 변까지 가장 짧은 거리(Nav2의 inscribed radius)."""
    best = math.inf
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1], strict=True):
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, -(x1 * dx + y1 * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(x1 + t * dx, y1 + t * dy))
    return best


def test_the_local_footprint_is_a_polygon_round_the_real_body():
    """추가 지시서 3의 2-1 2번: 사각형 0.12 x 0.12 m 대신 몸 모양에 가까운 다각형. 충돌 검사는
    local costmap의 발자국을 쓴다. 몸과 바퀴를 모두 덮고, 다각형 밖으로 1 cm 넘게 나가지 않는다."""
    footprint = yaml.safe_load(params("local_costmap", "local_costmap")["footprint"])
    polygon = [tuple(p) for p in footprint]
    assert len(polygon) >= 8
    for x, y in BODY_OUTLINE + WHEELS:
        assert _inside(polygon, x, y), (x, y)
    assert _area(polygon) <= 1.1 * _area(_hull(BODY_OUTLINE + WHEELS)), "close to the body"
    assert max(math.hypot(x, y) for x, y in polygon) <= 0.085, "never wider than the old square"


def test_fixed_lines_follow_a_fixed_short_lookahead_and_take_the_arcs_without_pivoting():
    """추가 지시서 3의 2-1 3번: 앞보기 0.10 m 고정(작업 기록 4.2에서 모퉁이 앞 회전이 줄었다).
    1번: 반지름 TURN_RADIUS_M의 호 위에서 앞보기 점은 asin(L / 2R)만큼 옆에 있다. 이것이
    rotate_to_heading_min_angle보다 작아야 RPP가 호 위에서 서서 돌지 않고 호를 따라 돈다."""
    c = params("controller_server")
    for name in ("FollowPath", "FollowPathThrough", "FollowPathReverse"):
        rpp = c[name]
        assert rpp["use_velocity_scaled_lookahead_dist"] is False, name
        assert rpp["lookahead_dist"] == 0.10, name
        carrot = math.asin(rpp["lookahead_dist"] / (2 * TURN_RADIUS_M))
        assert carrot + 0.1 < rpp["rotate_to_heading_min_angle"] < math.pi / 4, name
    assert c["FollowPathGoal"]["rotate_to_heading_min_angle"] == 0.25, "the yard is not changed"


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
    least the inscribed radius (footprint edge gap) until the robot touches. With 0.6 m
    a box within the 0.15 m inflation pinned the robot to the 0.05 m/s floor in every arena gap
    (user, 2026-11-10): the scaling may no longer cut the speed short of a touch."""
    c = params("controller_server")
    footprint = yaml.safe_load(params("local_costmap", "local_costmap")["footprint"])
    inscribed = _edge_gap([tuple(p) for p in footprint])
    for name in ("FollowPath", "FollowPathThrough"):
        rpp = c[name]
        assert rpp["use_cost_regulated_linear_velocity_scaling"] is True
        assert rpp["cost_scaling_dist"] == 0.05
        assert rpp["cost_scaling_gain"] * inscribed / rpp["cost_scaling_dist"] >= 1.0, name


def test_goal_checker_is_tighter_than_the_arrival_tolerance():
    c = params("controller_server")
    assert c["goal_checker_plugins"] == ["general_goal_checker", "retreat_goal_checker"]
    goal = c["general_goal_checker"]
    assert goal["xy_goal_tolerance"] == 0.05 and goal["xy_goal_tolerance"] < 0.15  # R4
    assert goal["yaw_goal_tolerance"] == 0.12


def test_retreat_goal_checker_checks_the_position_only():
    """2-1: 후진(FollowPathReverse)은 제자리에서 방향을 고칠 수 없으니 끝 방향을 보지 않는다.
    위치 허용치는 앞으로 갈 때와 같다. pinky_fsm reverse_goal_checker_id가 이 이름을 쓴다."""
    c = params("controller_server")
    retreat = c["retreat_goal_checker"]
    assert retreat["plugin"] == "nav2_controller::SimpleGoalChecker"
    assert retreat["xy_goal_tolerance"] == c["general_goal_checker"]["xy_goal_tolerance"]
    assert retreat["yaw_goal_tolerance"] > math.pi  # 어떤 방향 오차도 통과
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
    # 추가 지시서 3의 2-1은 고정 경로 컨트롤러만 바꿨다. FollowPathGoal은 회피 담당의 값이다(7-1)
    fixed_line_only = {"lookahead_dist", "use_velocity_scaled_lookahead_dist",
                       "rotate_to_heading_min_angle"}
    assert {k: v for k, v in goal.items() if k not in fixed_line_only} == {
        k: v for k, v in c["FollowPath"].items() if k not in fixed_line_only}


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
    # goal checker가 둘이면(retreat_goal_checker, 2-1) Nav2는 이름 없는 FollowPath를 거절한다
    [follow] = BT.getroot().iter("FollowPath")
    assert follow.get("goal_checker_id") == "general_goal_checker"
    assert follow.get("goal_checker_id") in params("controller_server")["goal_checker_plugins"]


def _canon(tree):
    return [(el.tag, sorted(el.attrib.items())) for el in tree.iter()]


@pytest.mark.skipif(not NAV2_BT.exists(), reason="ROS 2 Jazzy nav2_bt_navigator not installed")
def test_the_tree_is_nav2s_default_tree_with_only_the_goal_controller_and_checker_set():
    ours = ET.parse(BT_FILE)
    [select] = ours.getroot().iter("ControllerSelector")
    select.set("default_controller", "FollowPath")
    [follow] = ours.getroot().iter("FollowPath")
    del follow.attrib["goal_checker_id"]
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
