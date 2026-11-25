"""Nav2 설정이 Pode 약속을 지키는지 본다. ROS 없이 돈다."""
import copy
import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]


def _module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


owners = _module("pode_nav_params", PKG / "launch" / "pode_nav_params.py")
USER = yaml.safe_load((PKG / "params" / "nav2_params.yaml").read_text(encoding="utf-8"))
# 기본 회피 패키지(pode_avoidance)의 회피 담당 파일. 다른 패키지로 바꿔도 키 검사는 같다
AVOIDANCE_FILE = PKG.parent / "pode_avoidance" / "params" / "avoidance.yaml"
AVOIDANCE = yaml.safe_load(AVOIDANCE_FILE.read_text(encoding="utf-8"))
# 로봇이 받는 설정: 주행 담당 파일에 회피 담당 파일을 넣은 것(nav2.launch.py가 하는 그대로)
PARAMS = owners.with_avoidance(copy.deepcopy(USER), AVOIDANCE)
BT_FILE = PKG / "bt" / "navigate_to_pose_w_replanning_and_recovery_goal.xml"
BT = ET.parse(BT_FILE)
# Nav2 Jazzy 자체 트리, BT_FILE의 원본(ROS 2 Jazzy가 설치된 경우만 있다)
NAV2_BT = Path("/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/"
               "navigate_to_pose_w_replanning_and_recovery.xml")
KEEPOUT = yaml.safe_load((PKG / "params" / "keepout.yaml").read_text(encoding="utf-8"))
# 주행 최고 속도의 단일 출처
NAV_SPEED = yaml.safe_load(
    (PKG / "params" / "nav_speed.yaml").read_text(encoding="utf-8"))["nav_speed_mps"]


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
    assert rpp["desired_linear_vel"] == NAV_SPEED
    assert rpp["use_collision_detection"] is True
    assert rpp["max_allowed_time_to_collision_up_to_carrot"] == 0.6


# pode_fsm turn_radius_m: 꺾이는 곳을 이 반지름의 호로 보낸다
TURN_RADIUS_M = 0.10
# base_link 충돌 메시의 바닥 윤곽(볼록 껍질, base_footprint 틀, m)과 바퀴(축 y ±0.04055에서 바깥으로 0.0145 m, 반지름 0.028 m)
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
    """몸 모양에 가까운 다각형 발자국(충돌 검사는 local costmap 것). 몸과 바퀴를 덮고 다각형 밖으로 1 cm 넘게 나가지 않는다."""
    footprint = yaml.safe_load(params("local_costmap", "local_costmap")["footprint"])
    polygon = [tuple(p) for p in footprint]
    assert len(polygon) >= 8
    for x, y in BODY_OUTLINE + WHEELS:
        assert _inside(polygon, x, y), (x, y)
    assert _area(polygon) <= 1.1 * _area(_hull(BODY_OUTLINE + WHEELS)), "close to the body"
    assert max(math.hypot(x, y) for x, y in polygon) <= 0.085, "never wider than the old square"


def test_the_local_footprint_is_not_padded():
    """padding을 안 적으면 Jazzy 기본 0.01이 붙어 RPP가 부푼 몸으로 본다. 다각형이 몸을 덮으므로 local은 0이고 global(NavFn)은 0.01이다."""
    assert params("local_costmap", "local_costmap")["footprint_padding"] == 0.0
    assert params("global_costmap", "global_costmap")["footprint_padding"] == 0.01


def _lookahead(rpp: dict, speed: float) -> float:
    return min(max(abs(speed) * rpp["lookahead_time"], rpp["min_lookahead_dist"]),
               rpp["max_lookahead_dist"])


def test_fixed_lines_look_the_same_time_ahead_at_every_speed_and_take_the_arcs_without_pivoting():
    """속도에 비례하는 앞보기(0.8초 앞, 0.10~0.25 m)가 반지름 TURN_RADIUS_M 호 위에서 서서 돌지 않게 한다.

    속도를 다 낮춘 뒤의 앞보기 점이 asin(L / 2R)만큼 옆에 있고, 이것이 rotate_to_heading_min_angle보다 작아야 한다.
    곡률 규칙의 후보 속도 = 원하는 속도 x R / min_radius(바닥은 regulated_linear_scaling_min_speed).
    """
    c = params("controller_server")
    for name in ("FollowPath", "FollowPathThrough", "FollowPathReverse"):
        rpp = c[name]
        assert rpp["use_velocity_scaled_lookahead_dist"] is True, name
        assert rpp["lookahead_time"] == 0.8, name
        assert _lookahead(rpp, 0.12) == 0.10, name
        assert _lookahead(rpp, 0.20) == pytest.approx(0.16), name
        assert _lookahead(rpp, 0.25) == pytest.approx(0.20), name
        assert rpp["use_regulated_linear_velocity_scaling"] is True, name
        on_arc = max(rpp["regulated_linear_scaling_min_speed"], rpp["desired_linear_vel"]
                     * TURN_RADIUS_M / rpp["regulated_linear_scaling_min_radius"])
        assert _lookahead(rpp, on_arc) == 0.10, name
        carrot = math.asin(_lookahead(rpp, on_arc) / (2 * TURN_RADIUS_M))
        assert carrot + 0.1 < rpp["rotate_to_heading_min_angle"] < math.pi / 4, name


def test_the_through_controller_is_followpath_without_the_approach_slow_down():
    """끝이 잠금 경계인 경로(final_goal 거짓)는 FollowPathThrough로 간다. approach_velocity_scaling_dist만 짧고 나머지 키는 FollowPath와 같아야 한다."""
    c = params("controller_server")
    normal, through = c["FollowPath"], c["FollowPathThrough"]
    assert through["approach_velocity_scaling_dist"] == 0.05
    assert normal["approach_velocity_scaling_dist"] > through["approach_velocity_scaling_dist"]
    assert set(through) == set(normal)
    assert {k: v for k, v in through.items() if k != "approach_velocity_scaling_dist"} == {
        k: v for k, v in normal.items() if k != "approach_velocity_scaling_dist"}


def test_a_fixed_line_slows_down_only_in_its_last_0_1_m():
    """고정 경로가 짧아(0.14~0.40 m) 감속은 끝 0.10 m 앞부터, 속도는 0.08 m/s 아래로 내리지 않는다. 후진은 0.05 m/s라 영향이 없다."""
    c = params("controller_server")
    for name in ("FollowPath", "FollowPathReverse"):
        assert c[name]["approach_velocity_scaling_dist"] == 0.10, name
        assert c[name]["min_approach_linear_velocity"] == 0.08, name


REVERSE_ONLY = {"desired_linear_vel": 0.05, "use_rotate_to_heading": False,
                "allow_reversing": True}


def test_the_reverse_controller_is_followpath_driven_backwards_slowly():
    """FollowPathReverse는 제자리 회전을 끄고 0.05 m/s로 물러나며, 충돌 검사는 켜 둔다(뒤의 물체에 선다)."""
    c = params("controller_server")
    normal, reverse = c["FollowPath"], c["FollowPathReverse"]
    assert set(reverse) == set(normal)
    assert {k: reverse[k] for k in REVERSE_ONLY} == REVERSE_ONLY
    assert {k: v for k, v in reverse.items() if k not in REVERSE_ONLY} == {
        k: v for k, v in normal.items() if k not in REVERSE_ONLY}
    assert reverse["use_collision_detection"] is True


def test_no_obstacle_beside_the_path_slows_the_robot():
    """RPP는 비용에서 읽은 거리 d가 cost_scaling_dist보다 작으면 속도를 줄인다. 경로 옆 상자가 속도를 바닥에 묶지 않도록 닿기 전에는 줄이지 않게 한다."""
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
    assert c["goal_checker_plugins"] == ["general_goal_checker", "retreat_goal_checker",
                                         "free_heading_goal_checker"]
    goal = c["general_goal_checker"]
    assert goal["xy_goal_tolerance"] == 0.05 and goal["xy_goal_tolerance"] < 0.15
    assert goal["yaw_goal_tolerance"] == 0.12


def test_free_heading_goal_checker_stops_like_the_general_one_without_the_turn():
    """다음 조각이 Nav2 goal인 고정 경로는 끝 방향을 맞추지 않는다. 위치 허용치는 general과 같고 pode_fsm free_heading_goal_checker_id가 이 이름을 쓴다."""
    c = params("controller_server")
    free = c["free_heading_goal_checker"]
    assert free["plugin"] == "nav2_controller::SimpleGoalChecker"
    assert free["xy_goal_tolerance"] == c["general_goal_checker"]["xy_goal_tolerance"]
    assert free["yaw_goal_tolerance"] > math.pi
    assert free["stateful"] is True


def test_retreat_goal_checker_checks_the_position_only():
    """후진은 끝 방향을 보지 않고 위치 허용치는 0.02로 앞으로 갈 때(0.05)보다 좁다. pode_fsm reverse_goal_checker_id가 이 이름을 쓴다."""
    c = params("controller_server")
    retreat = c["retreat_goal_checker"]
    assert retreat["plugin"] == "nav2_controller::SimpleGoalChecker"
    assert retreat["xy_goal_tolerance"] == 0.02
    assert retreat["xy_goal_tolerance"] < c["general_goal_checker"]["xy_goal_tolerance"]
    assert retreat["yaw_goal_tolerance"] > math.pi  # 어떤 방향 오차도 통과
    assert c["progress_checker_plugins"] == ["progress_checker"]
    assert c["progress_checker"]["required_movement_radius"] == 0.05


def test_speed_limit_topic_stays_for_version_2():
    assert params("controller_server")["speed_limit_topic"] == "speed_limit"


def test_local_costmap_is_fine_and_sent_whole_for_blocked():
    local = params("local_costmap", "local_costmap")
    assert local["resolution"] == 0.025
    assert local["always_send_full_costmap"] is True  # pode_fsm이 칸 값으로 BLOCKED를 판단한다
    assert "obstacle_layer" in local["plugins"]


def test_costmap_topics_are_absolute_inside_the_robot_namespace():
    """Jazzy costmap 층은 상대 토픽을 costmap 노드 안으로 풀므로 launch가 절대 이름을 넣는다."""
    local = params("local_costmap", "local_costmap")
    glob = params("global_costmap", "global_costmap")
    assert local["obstacle_layer"]["scan"]["topic"] == "<robot_namespace>/scan"
    assert glob["obstacle_layer"]["scan"]["topic"] == "<robot_namespace>/scan"
    assert glob["static_layer"]["map_topic"] == "<robot_namespace>/map"


def test_the_filtered_scan_clears_along_the_beams_the_filter_dropped():
    """거르개가 지운 빔(inf)도 칸을 지운다. 거르개를 켤 때 덧붙는 scan_filter.yaml에만 있고 기본은 Nav2 기본(false)이다."""
    added = yaml.safe_load((PKG / "params" / "scan_filter.yaml").read_text(encoding="utf-8"))
    for costmap in ("local_costmap", "global_costmap"):
        scan = added[costmap][costmap]["ros__parameters"]["obstacle_layer"]["scan"]
        assert scan["topic"] == "<robot_namespace>/scan_filtered"
        assert scan["inf_is_valid"] is True
        assert "inf_is_valid" not in params(costmap, costmap)["obstacle_layer"]["scan"]


def test_global_costmap_has_the_lidar_layer_for_version_2_detours():
    glob = params("global_costmap", "global_costmap")
    assert glob["plugins"] == ["static_layer", "obstacle_layer", "inflation_layer"]


def test_planner_server_runs_for_compute_path_and_is_path_valid():
    assert params("planner_server")["planner_plugins"] == ["GridBased"]


def test_the_planner_does_not_swap_a_blocked_goal_for_a_far_point():
    """막힌 goal 대신 먼 점으로 경로를 내면 그 점에 서고 가짜 성공이 난다. 대체점을 찾는 사각형 반폭은 도착 판정 허용치를 넘지 않는다."""
    tolerance = params("planner_server")["GridBased"]["tolerance"]
    assert tolerance <= params("controller_server")["general_goal_checker"]["xy_goal_tolerance"]


def test_amcl_reads_the_fine_map():
    amcl = params("amcl")
    assert amcl["map_topic"] == "map_fine"
    assert amcl["sigma_hit"] == 0.05
    assert amcl["base_frame_id"] == "base_footprint" and amcl["scan_topic"] == "scan"


def test_the_goal_controller_is_named_and_stops_when_blocked():
    """MoveTo mode nav_goal은 FollowPathGoal로 달린다. 값은 회피 담당의 것이라 표준이 요구하는 것만 확인하고, RPP인 동안은 충돌 검사를 켜야 한다."""
    goal = params("controller_server")["FollowPathGoal"]
    assert goal["plugin"], "a controller plugin is named"
    if goal["plugin"].endswith("RegulatedPurePursuitController"):
        assert goal["use_collision_detection"] is True


def test_the_goal_controller_keeps_its_carrot_short_at_0_20_and_brakes_late():
    """RPP인 동안 넓은 구역 컨트롤러는 고정 선처럼 0.8초 앞을 보고, 급한 굽이에서 줄이며, goal 0.2 m 앞부터 줄인다. 다른 플러그인에는 적용하지 않는다."""
    goal = params("controller_server")["FollowPathGoal"]
    if not goal["plugin"].endswith("RegulatedPurePursuitController"):
        pytest.skip("not an RPP")
    assert goal["use_velocity_scaled_lookahead_dist"] is True
    assert _lookahead(goal, 0.20) == pytest.approx(0.16)
    assert _lookahead(goal, 0.25) == pytest.approx(0.20)
    assert _lookahead(goal, 0.25) < goal["max_lookahead_dist"]
    assert goal["use_regulated_linear_velocity_scaling"] is True
    assert goal["regulated_linear_scaling_min_radius"] == 0.25
    assert goal["approach_velocity_scaling_dist"] == 0.2


def test_every_controller_turns_in_place_at_1_rad_s():
    """제자리 회전은 1.0 rad/s다(Gazebo 구동 플러그인이 회전을 1.0 rad/s로 자른다)."""
    c = params("controller_server")
    goal = c["FollowPathGoal"]
    if not goal["plugin"].endswith("RegulatedPurePursuitController"):
        pytest.skip("not an RPP")
    assert (goal["rotate_to_heading_angular_vel"], goal["max_angular_accel"]) == (1.0, 3.2)
    for name in ("FollowPath", "FollowPathThrough", "FollowPathReverse"):
        assert c[name]["rotate_to_heading_angular_vel"] == 1.0, name


def test_behavior_server_offers_what_the_tree_and_pode_fsm_call():
    """Nav2 트리는 지도 지우기와 기다리기만 하고, 후진은 pode_fsm이 backup으로 한다. Spin은 없다."""
    b = params("behavior_server")
    assert b["behavior_plugins"] == ["backup", "wait"]
    assert {name: b[name]["plugin"] for name in b["behavior_plugins"]} == {
        "backup": "nav2_behaviors::BackUp", "wait": "nav2_behaviors::Wait"}
    assert "spin" not in b
    server_of = {"BackUp": "backup", "Wait": "wait"}
    for tag, server in server_of.items():
        for node in BT.getroot().iter(tag):
            assert node.get("server_name", server) in b["behavior_plugins"]


def test_bt_navigator_serves_only_navigate_to_pose():
    assert params("bt_navigator")["navigators"] == ["navigate_to_pose"]


def test_the_tree_replans_at_1_hz_and_recovers_without_moving_the_robot():
    """트리에 Spin과 BackUp이 없다: 후진은 pode_fsm의 로봇단 처리뿐이라 둘이 동시에 후진하지 않는다."""
    tags = [el.tag for el in BT.iter()]
    for kept in ("Wait", "ClearEntireCostmap", "RecoveryNode", "RoundRobin"):
        assert kept in tags
    assert "Spin" not in tags and "BackUp" not in tags
    [recoveries] = [el for el in BT.iter("RoundRobin")]
    assert [child.tag for child in recoveries] == ["Sequence", "Wait"]
    [rate] = BT.getroot().iter("RateController")
    assert float(rate.get("hz")) == 1.0
    [planner] = BT.getroot().iter("ComputePathToPose")
    assert params("planner_server")["planner_plugins"] == ["GridBased"]
    [select] = BT.getroot().iter("PlannerSelector")
    assert select.get("default_planner") == "GridBased" and planner is not None
    [controller] = BT.getroot().iter("ControllerSelector")
    assert controller.get("default_controller") == "FollowPathGoal"
    # goal checker가 둘이면(retreat_goal_checker) Nav2는 이름 없는 FollowPath를 거절한다
    [follow] = BT.getroot().iter("FollowPath")
    assert follow.get("goal_checker_id") == "general_goal_checker"
    assert follow.get("goal_checker_id") in params("controller_server")["goal_checker_plugins"]


def _canon(tree):
    return [(el.tag, sorted(el.attrib.items())) for el in tree.iter()]


@pytest.mark.skipif(not NAV2_BT.exists(), reason="ROS 2 Jazzy nav2_bt_navigator not installed")
def test_the_tree_is_nav2s_default_tree_with_the_goal_controller_checker_and_no_moving_recovery():
    ours = ET.parse(BT_FILE)
    [select] = ours.getroot().iter("ControllerSelector")
    select.set("default_controller", "FollowPath")
    [follow] = ours.getroot().iter("FollowPath")
    del follow.attrib["goal_checker_id"]
    # 회복 Clear 노드의 server_timeout은 우리가 더한 것이라 비교에서 뺀다
    for clear in ours.getroot().iter("ClearEntireCostmap"):
        clear.attrib.pop("server_timeout", None)
    default = ET.parse(NAV2_BT)
    for parent in default.getroot().iter("RoundRobin"):
        for child in [c for c in parent if c.tag in ("Spin", "BackUp")]:
            parent.remove(child)
    assert _canon(ours) == _canon(default)


def test_the_old_plan_once_tree_is_gone():
    assert not (PKG / "bt" / "plan_once_no_recovery.xml").exists()


# ---- 경로 뒤집힘 비교용 갈래 트리: 기본 트리와 한 곳만 다르다 ----

def test_the_keep_tree_differs_from_the_default_tree_only_in_the_planner_server():
    keep = ET.parse(PKG / "bt" / "navigate_to_pose_w_replanning_and_recovery_goal_keep.xml")
    [planner] = keep.getroot().iter("ComputePathToPose")
    assert planner.attrib.pop("server_name") == "compute_path_keep"
    # 길 유지 노드는 rclpy라 goal 승인이 전역 기본값 20 ms보다 늦을 수 있다
    assert planner.attrib.pop("server_timeout") == "1000"
    assert params("bt_navigator")["default_server_timeout"] == 20
    assert _canon(keep) == _canon(ET.parse(BT_FILE))


def test_the_3_s_tree_differs_from_the_default_tree_only_in_the_replanning_rate():
    slow = ET.parse(PKG / "bt" / "navigate_to_pose_w_replanning_and_recovery_goal_replan_3s.xml")
    [rate] = slow.getroot().iter("RateController")
    assert abs(float(rate.get("hz")) - 1 / 3) < 1e-3
    rate.set("hz", "1.0")
    assert _canon(slow) == _canon(ET.parse(BT_FILE))


# ---- Nav2 goal용 global costmap ------------------------------------------------------

LETHAL, INSCRIBED = 254, 253


def _inflated(free_cells: int, glob: dict) -> list[int]:
    """폭 free_cells칸 통로의 Nav2 inflation 비용(InflationLayer: inscribed 안은 inscribed, 그 뒤 252 * exp(-k (d - r)))."""
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


def test_the_global_costmap_is_the_a_plus_inflation_setting():
    """2.5 cm 지도, padding 0.01, inflation 15.0/0.15를 시뮬과 실물이 같이 쓴다."""
    glob = params("global_costmap", "global_costmap")
    assert glob["resolution"] == 0.025 and glob["footprint_padding"] == 0.01
    infl = glob["inflation_layer"]
    assert (infl["cost_scaling_factor"], infl["inflation_radius"]) == (15.0, 0.15)


@pytest.mark.parametrize("free_cells", [14, 15])
def test_navfn_finds_a_way_through_a_0_38_m_passage(free_cells):
    """0.38 m 통로는 2.5 cm 지도에서 14~15칸이다. INSCRIBED(253) 이상 칸은 벽이라, 그보다 낮은 칸이 있고 가운데가 가장 싸야 경로가 가운데로 간다."""
    glob = params("global_costmap", "global_costmap")
    costs = _inflated(free_cells, glob)
    assert min(costs) < INSCRIBED
    middle = costs[(free_cells - 1) // 2]
    assert middle == min(costs)


def test_the_wide_yard_has_cost_free_room_in_the_middle():
    glob = params("global_costmap", "global_costmap")
    assert 0 in _inflated(int(0.775 / glob["resolution"]), glob)


def test_goal_paths_keep_some_room_but_do_not_swing_wide():
    """벽 칸에서 0.10 m인 칸은 값이 있고 0.20 m인 칸은 0이라 경로가 벽에서 멀리 돌지 않는다."""
    glob = params("global_costmap", "global_costmap")
    res = glob["resolution"]
    costs = _inflated(int(0.775 / res), glob)
    assert costs[round(0.10 / res) - 1] > 0
    assert costs[round(0.20 / res) - 1] == 0


def test_the_local_costmap_that_followpath_uses_is_unchanged():
    local = params("local_costmap", "local_costmap")
    assert local["inflation_layer"] == {"plugin": "nav2_costmap_2d::InflationLayer",
                                        "cost_scaling_factor": 3.0, "inflation_radius": 0.15}
    assert "filters" not in local


# ---- Nav2 goal 전용 keepout(급행 레인, 도크 구역) -------------------------------------


def test_the_base_params_have_no_keepout():
    """keepout_mask가 비면 필터도 서버도 없고 다른 것은 바뀌지 않는다."""
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
    """Jazzy costmap 필터는 상대 토픽을 costmap 노드 안으로 풀므로 필터 쪽은 절대 토픽을 쓰고, 두 서버는 로봇 namespace에서 돈다."""
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


# ---- 설정의 주인 ----------------------------------------------------------------------

def _layer(data, costmap):
    return data[costmap][costmap]["ros__parameters"]["obstacle_layer"]


def test_the_avoidance_file_holds_only_the_avoidance_keys():
    assert owners.AVOIDANCE_KEYS == ("follow_path_goal", "vision_obstacles", "peer_layer")
    assert sorted(AVOIDANCE) == ["follow_path_goal", "peer_layer", "vision_obstacles"]
    assert owners.foreign_keys(AVOIDANCE) == []


def test_the_peer_layer_key_may_be_left_out():
    """회피 담당이 peer 층을 자기 방식으로 바꾸면 이 키를 뺄 수 있다."""
    mine = {k: v for k, v in AVOIDANCE.items() if k != "peer_layer"}
    assert owners.foreign_keys(mine) == []
    owners.with_avoidance(copy.deepcopy(USER), mine)


def test_vision_without_the_vision_key_is_refused():
    mine = {k: v for k, v in AVOIDANCE.items() if k != "vision_obstacles"}
    owners.with_avoidance(copy.deepcopy(USER), mine)
    with pytest.raises(ValueError, match="vision_obstacles"):
        owners.with_avoidance(copy.deepcopy(USER), mine, vision=True)


def test_a_goal_controller_is_needed_from_somewhere():
    mine = {k: v for k, v in AVOIDANCE.items() if k != "follow_path_goal"}
    with pytest.raises(ValueError, match="follow_path_goal"):
        owners.with_avoidance(copy.deepcopy(USER), mine)
    owners.with_avoidance(copy.deepcopy(PARAMS), mine)  # 시뮬이 이미 넣어 둔 경우


@pytest.mark.parametrize("key", ["controller_server", "local_costmap", "global_costmap",
                                 "FollowPath", "footprint", "bt_navigator", "planner_server"])
def test_a_user_owned_key_in_the_avoidance_file_is_refused(key):
    """목표 컨트롤러 안쪽과 vision 관측원을 뺀 나머지는 주행 담당의 것이다."""
    bad = {**AVOIDANCE, key: {"anything": 1}}
    assert owners.foreign_keys(bad) == [key]
    with pytest.raises(ValueError, match=key):
        owners.with_avoidance(copy.deepcopy(USER), bad)


def test_the_users_file_holds_no_avoidance_values():
    assert owners.avoidance_values_in(USER) == []
    assert "FollowPathGoal" in USER["controller_server"]["ros__parameters"]["controller_plugins"], \
        "the name is the user's and stays"
    taken = copy.deepcopy(USER)
    taken["controller_server"]["ros__parameters"]["FollowPathGoal"] = {"plugin": "x"}
    _layer(taken, "local_costmap")["vision"] = {"topic": "x"}
    assert owners.avoidance_values_in(taken) == [
        "controller_server.FollowPathGoal", "local_costmap.obstacle_layer.vision"]


def test_the_goal_controller_is_what_the_avoidance_file_says():
    goal = PARAMS["controller_server"]["ros__parameters"]["FollowPathGoal"]
    assert goal == AVOIDANCE["follow_path_goal"]


def test_a_goal_controller_already_in_the_file_is_kept():
    """시뮬레이션이 목표 컨트롤러에 속도 상한이 이미 들어간 파일을 줄 때."""
    capped = copy.deepcopy(PARAMS)
    capped["controller_server"]["ros__parameters"]["FollowPathGoal"]["desired_linear_vel"] = 0.12
    out = owners.with_avoidance(capped, AVOIDANCE)
    goal = out["controller_server"]["ros__parameters"]["FollowPathGoal"]
    assert goal["desired_linear_vel"] == 0.12


def test_the_vision_source_goes_into_both_costmaps_only_when_asked():
    for costmap in ("local_costmap", "global_costmap"):
        assert _layer(PARAMS, costmap)["observation_sources"] == "scan"
        assert "vision" not in _layer(PARAMS, costmap)
    seen = owners.with_avoidance(copy.deepcopy(USER), AVOIDANCE, vision=True)
    for costmap in ("local_costmap", "global_costmap"):
        layer = _layer(seen, costmap)
        assert layer["observation_sources"] == "scan vision"
        assert layer["vision"] == AVOIDANCE["vision_obstacles"]
        assert layer["scan"] == _layer(USER, costmap)["scan"], "the lidar source is the user's"
    assert AVOIDANCE["vision_obstacles"]["topic"] == "<robot_namespace>/vision_obstacles"


# ---- 다른 로봇의 몸을 costmap에(peer 층, 임시: 주행 시험용) ----------------------------------

PEERS = {"peer_layer": AVOIDANCE["peer_layer"]}  # 회피 담당 파일의 peer_layer 키


def _costmap(data, costmap):
    return data[costmap][costmap]["ros__parameters"]


def test_the_peer_layer_goes_into_both_costmaps_only_when_asked():
    for costmap in ("local_costmap", "global_costmap"):
        assert "peer_layer" not in _costmap(PARAMS, costmap)["plugins"]
        assert "peer_layer" not in _costmap(PARAMS, costmap)
    seen = owners.with_peers(copy.deepcopy(PARAMS), PEERS)
    for costmap in ("local_costmap", "global_costmap"):
        before, after = _costmap(PARAMS, costmap), _costmap(seen, costmap)
        at = after["plugins"].index("peer_layer")
        assert after["plugins"][at + 1] == "inflation_layer", "the bodies are inflated too"
        assert [p for p in after["plugins"] if p != "peer_layer"] == before["plugins"]
        assert after["peer_layer"] == PEERS["peer_layer"]
        assert after["obstacle_layer"] == before["obstacle_layer"], "the lidar layer is untouched"


def test_the_peer_layer_can_be_taken_from_the_whole_avoidance_file():
    seen = owners.with_peers(copy.deepcopy(PARAMS), AVOIDANCE)
    assert _costmap(seen, "local_costmap")["peer_layer"] == AVOIDANCE["peer_layer"]


def test_the_peer_layer_is_added_once():
    seen = owners.with_peers(owners.with_peers(copy.deepcopy(PARAMS), PEERS), PEERS)
    for costmap in ("local_costmap", "global_costmap"):
        assert _costmap(seen, costmap)["plugins"].count("peer_layer") == 1


def test_the_peer_layer_marks_the_bodies_and_sweeps_its_own_cells_every_update():
    layer = PEERS["peer_layer"]
    assert layer["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert layer["observation_sources"].split() == ["peers", "sweep"]
    peers, sweep = layer["peers"], layer["sweep"]
    assert peers["topic"] == "<robot_namespace>/peer_obstacles"
    assert peers["data_type"] == "PointCloud2"
    assert peers["marking"] is True and peers["clearing"] is False
    assert sweep["topic"] == "<robot_namespace>/peer_sweep"
    assert sweep["data_type"] == "LaserScan"
    assert sweep["marking"] is False and sweep["clearing"] is True
    assert sweep["inf_is_valid"] is True, "the sweep beams are all inf: they only clear"
    for source in (peers, sweep):
        assert source["expected_update_rate"] == 0.0, "a silent topic stops nothing"
        assert source["observation_persistence"] == 0.0, "only the newest message counts"


def test_the_sweep_reaches_every_cell_of_the_local_window():
    local = _costmap(PARAMS, "local_costmap")
    corner = math.hypot(local["width"], local["height"]) / 2.0
    assert PEERS["peer_layer"]["sweep"]["raytrace_max_range"] >= corner


def test_a_body_point_is_marked_anywhere_on_the_arena():
    """점구름은 map 틀이라 obstacle_max_range를 경기장 대각선(2.3 m)보다 넉넉히 둔다."""
    assert PEERS["peer_layer"]["peers"]["obstacle_max_range"] >= 5.0


def test_the_far_point_that_keeps_the_cloud_from_being_empty_is_never_marked():
    """pode_peer_obstacles는 빈 점구름을 피하려고 map 원점에서 1000 m 넘게 먼 점 하나를 늘 낸다."""
    assert PEERS["peer_layer"]["peers"]["obstacle_max_range"] <= 100.0


def test_recovery_clear_nodes_have_their_own_longer_server_timeout():
    """모든 Clear 노드는 자기 server_timeout(ms)을 가진다. 전역 기본값 20은 그대로."""
    clears = list(BT.getroot().iter("ClearEntireCostmap"))
    assert len(clears) == 4
    for node in clears:
        assert int(node.get("server_timeout", "0")) >= 500, node.get("name")
    assert params("bt_navigator")["default_server_timeout"] == 20


# ---- 비교용 MPPI(FollowPathMPPI) ----
TASK_SPEC = PKG.parents[1] / "learning" / "goal_reach" / "task_spec.py"
SIM_URDF = PKG.parents[1] / "sim" / "fleet_gz_sim" / "urdf" / "pode_sim.urdf.xacro"


def _action_limits():
    import ast
    tree = ast.parse(TASK_SPEC.read_text(encoding="utf-8"))
    [cls] = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "ActionLimits"]
    return {n.target.id: ast.literal_eval(n.value) if not isinstance(n.value, ast.Name)
            else n.value.id for n in cls.body if isinstance(n, ast.AnnAssign)}


def test_the_mppi_block_is_there_but_the_real_robot_does_not_load_it():
    c = params("controller_server")
    assert "FollowPathMPPI" not in c["controller_plugins"]
    mppi = c["FollowPathMPPI"]
    assert mppi["plugin"] == "nav2_rotation_shim_controller::RotationShimController"
    assert mppi["primary_controller"] == "nav2_mppi_controller::MPPIController"
    assert mppi["motion_model"] == "DiffDrive"
    assert mppi["model_dt"] == pytest.approx(1.0 / c["controller_frequency"])
    assert mppi["time_steps"] * mppi["model_dt"] * mppi["vx_max"] >= 0.50 - 0.01
    for critic in mppi["critics"]:
        assert mppi[critic]["enabled"] is True, critic


def test_the_mppi_limits_match_the_goal_controller_isaac_and_gazebo():
    import re
    mppi = params("controller_server")["FollowPathMPPI"]
    goal = params("controller_server")["FollowPathGoal"]
    isaac = _action_limits()
    urdf = SIM_URDF.read_text(encoding="utf-8")
    gz_w = float(re.search(r"<max_velocity>([^<]+)</max_velocity>", urdf).group(1))
    gz_acc = float(re.search(r"<max_acceleration>([^<]+)</max_acceleration>", urdf).group(1))
    assert mppi["vx_max"] == goal["desired_linear_vel"] == NAV_SPEED
    assert isaac["v_max"] == "NAV_SPEED_MPS"
    assert mppi["vx_min"] == isaac["v_min"]
    assert mppi["wz_max"] == goal["rotate_to_heading_angular_vel"] == isaac["w_max"] == gz_w
    assert mppi["ax_max"] == -mppi["ax_min"] == isaac["acc_v"] == gz_acc
    assert mppi["az_max"] == isaac["acc_w"] == gz_acc
    assert mppi["rotate_to_heading_angular_vel"] == goal["rotate_to_heading_angular_vel"]
    assert mppi["max_angular_accel"] == mppi["az_max"]


def test_the_tree_still_drives_with_the_goal_controller_unless_told_otherwise():
    [controller] = BT.getroot().iter("ControllerSelector")
    assert controller.get("default_controller") == "FollowPathGoal"
    assert controller.get("topic_name") == "controller_selector"
    repo = PKG.parents[1]
    talkers = [path for top in ("pc", "pode", "sim", "tools")
               for path in (repo / top).rglob("*.py")
               if path.resolve() != Path(__file__).resolve()
               and path.resolve() != (repo / "tools" / "nav_bench" / "bench_node.py").resolve()
               and "controller_selector" in path.read_text(encoding="utf-8", errors="ignore")]
    assert talkers == []


def test_the_goal_controller_is_still_the_avoidance_block():
    assert "FollowPathGoal" not in USER["controller_server"]["ros__parameters"]
    assert params("controller_server")["FollowPathGoal"] == AVOIDANCE["follow_path_goal"]


# ---- 주행 최고 속도의 단일 출처 ----
REPO = PKG.parents[1]
FAKES = REPO / "sim" / "fake_robots"


def _literal_default(path: Path, cls_name: str, field: str):
    import ast
    tree = ast.parse(path.read_text(encoding="utf-8"))
    [cls] = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls_name]
    [node] = [n for n in cls.body if isinstance(n, ast.AnnAssign) and n.target.id == field]
    return ast.literal_eval(node.value)


def test_every_copy_of_the_top_speed_is_the_one_source():
    """nav_speed.yaml을 바꾸면 여기의 사본이 틀려 실패한다(그때 맞춘다). 읽어서 따르는 곳은 각자의 시험이 본다."""
    c = params("controller_server")
    fleet_file = REPO / "pc" / "fleet_core" / "fleet_core" / "data" / "fleet_params.yaml"
    fleet = yaml.safe_load(fleet_file.read_text(encoding="utf-8"))
    params_py = REPO / "pc" / "fleet_core" / "fleet_core" / "common" / "params.py"
    bench = yaml.safe_load((REPO / "tools" / "nav_bench" / "scenarios.yaml")
                           .read_text(encoding="utf-8"))
    import re
    start = (REPO / "tools" / "real_pode" / "start.sh").read_text(encoding="utf-8")
    copies = {
        "nav2 FollowPath": c["FollowPath"]["desired_linear_vel"],
        "nav2 FollowPathThrough": c["FollowPathThrough"]["desired_linear_vel"],
        "avoidance FollowPathGoal": AVOIDANCE["follow_path_goal"]["desired_linear_vel"],
        "nav2 FollowPathMPPI vx_max": c["FollowPathMPPI"]["vx_max"],
        "fleet_params nav_speed_mps": fleet["nav_speed_mps"],
        "fleet_params follow_speed_mps": fleet["follow_speed_mps"],
        "params.py nav_speed_mps": _literal_default(params_py, "FleetParams", "nav_speed_mps"),
        "params.py follow_speed_mps": _literal_default(params_py, "FleetParams",
                                                       "follow_speed_mps"),
        "nav_bench robot mover": bench["movers"]["robot"]["speed"],
        "pode_fsm horizon_cap_mps": _literal_default(
            REPO / "pode" / "pode_fsm" / "pode_fsm" / "params.py", "FsmParams", "horizon_cap_mps"),
        "start.sh speed": float(re.search(r"^speed=\$\{5:-([0-9.]+)\}$", start, re.M).group(1)),
        "fake_robots nav_speed_mps()": _module(
            "fake_nav_speed", FAKES / "fake_robots" / "nav_speed.py").nav_speed_mps(),
    }
    assert {name: value for name, value in copies.items() if value != NAV_SPEED} == {}
    files = [*(FAKES / "scenarios").glob("*.yaml"),
             *(REPO / "pc" / "fleet_manager" / "test").glob("*.yaml")]
    robots = {f"{path.name} {name}": raw or {} for path in sorted(files)
              for name, raw in (yaml.safe_load(path.read_text(encoding="utf-8"))
                                .get("robots") or {}).items()}
    assert robots
    assert {name: raw for name, raw in robots.items()
            if "speed_mps" in raw or not 0 < raw.get("speed_scale", 1.0) <= 1.0} == {}
