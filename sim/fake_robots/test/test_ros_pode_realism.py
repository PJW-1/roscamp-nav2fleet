"""가짜 Pode를 실물에 가깝게: 끝 노드 앞에서 넘어가기, 끝에서 제자리 돌기, 시뮬 시간 진행 소식.

넘김 거리와 회전 각속도는 관제 파일(fleet_params.yaml의 nav_pass_m, nav_turn_rps)에서 읽는다.
"""
import math
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import wait_until  # noqa: E402
from fake_robots.common import load_fleet_params  # noqa: E402
from fleet_interfaces.action import MoveTo  # noqa: E402
from fleet_interfaces.msg import Segment  # noqa: E402
from geometry_msgs.msg import Pose2D, PoseStamped  # noqa: E402
from test_ros_pode import (  # noqa: E402
    FAST_CLOCK,
    MOVE_MARGIN,
    SITE,
    move_wait,
    path_m,
    result_of,
    start,
    watch_truth,
)

PARAMS = load_fleet_params()
PASS_M = PARAMS["nav_pass_m"]
TURN_RPS = PARAMS["nav_turn_rps"]
ROBOT = {"pode1": {"start": "P-1", "speed_scale": 1.0, "pose_noise_m": 0.0, "stop_error_m": 0.0}}
NODES = {1: "N_W", 2: "N_E"}


def turn_wait(rad):
    """제자리 회전을 기다릴 실제 초(이동 대기와 같은 여유와 시계 배율)."""
    return rad / TURN_RPS * MOVE_MARGIN / FAST_CLOCK


def nav_seg(seq, yaw=0.0):
    x, y = SITE.node_pose(NODES[seq])
    msg = Segment(seq_id=seq, mode=Segment.MODE_NAV_GOAL, end_node_id=NODES[seq])
    msg.target_pose = PoseStamped()
    msg.target_pose.header.frame_id = "map"
    msg.target_pose.pose.position.x, msg.target_pose.pose.position.y = x, y
    msg.target_pose.pose.orientation.z = math.sin(yaw / 2)
    msg.target_pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


def nav_order(p, segs):
    goal = MoveTo.Goal(order_id="pode1-1", base=segs)
    goal.target_id = segs[-1].end_node_id
    fut = p.move.send_goal_async(goal)
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    return fut.result()


def watch_truth_yaw(p):
    from fake_robots.common import TRUTH_TOPIC
    from fleet_core.common.contract import to_ros_qos
    from fleet_interfaces.msg import RobotState
    from test_ros_pode import CONTRACT_OBJ
    seen = []
    p.node.create_subscription(
        RobotState, TRUTH_TOPIC.format(robot="pode1"),
        lambda m: seen.append((time.monotonic(), m.pose.x, m.pose.y, m.pose.theta)),
        to_ros_qos(CONTRACT_OBJ.qos("robot_state")))
    return seen


def test_a_nav_goal_segment_with_the_next_one_held_passes_its_end_early(harness, tmp_path):
    """pode_fsm order_fsm._passes: 다음 조각을 이미 받았으면 끝 노드 nav_pass_m 앞에서 넘어간다."""
    p = start(harness, tmp_path, clock=FAST_CLOCK, robots=ROBOT)
    assert wait_until(lambda: p.states, 5.0)
    truth = watch_truth(p)
    handle = nav_order(p, [nav_seg(1), nav_seg(2)])
    res = result_of(handle, move_wait(path_m("P-1", "N_W", "N_E"), clock=FAST_CLOCK)).result
    assert (res.success, res.last_done_seq_id) == (True, 2)
    nearest = min(math.dist((x, y), SITE.node_pose("N_W")) for _, x, y in truth)
    assert 0.5 * PASS_M < nearest <= PASS_M + 0.01, "it went to the end node itself"


def test_the_last_nav_goal_segment_goes_to_its_end_and_turns_there(harness, tmp_path):
    """다음 조각이 없으면 끝 노드까지 가서 제자리에서 nav_turn_rps로 끝 방향으로 돈다."""
    p = start(harness, tmp_path, clock=FAST_CLOCK, robots=ROBOT)
    assert wait_until(lambda: p.states, 5.0)
    seen = watch_truth_yaw(p)
    end = SITE.node_pose("N_W")
    handle = nav_order(p, [nav_seg(1, yaw=math.pi)])
    wait = move_wait(path_m("P-1", "N_W"), clock=FAST_CLOCK) + turn_wait(math.pi)
    res = result_of(handle, wait).result
    done = time.monotonic()
    assert res.success is True
    at_end = [(w, yaw) for w, x, y, yaw in seen if math.dist((x, y), end) < 1e-6]
    assert at_end and abs(math.remainder(at_end[-1][1] - math.pi, math.tau)) < 1e-6
    approach = math.atan2(end[1] - SITE.node_pose("P-1")[1], end[0] - SITE.node_pose("P-1")[0])
    turn_s = abs(math.remainder(math.pi - approach, math.tau)) / TURN_RPS
    stood = (done - at_end[0][0]) * FAST_CLOCK
    assert stood == pytest.approx(turn_s, abs=0.6), "it did not turn at nav_turn_rps"
    assert len({round(yaw, 3) for _, yaw in at_end}) > 10, "it turned at once"


def test_a_nav_goal_move_turns_to_its_target_heading(harness, tmp_path):
    p = start(harness, tmp_path, clock=FAST_CLOCK, robots=ROBOT)
    assert wait_until(lambda: p.states, 5.0)
    seen = watch_truth_yaw(p)
    x, y = SITE.node_pose("N_W")
    msg = MoveTo.Goal(target_id="N_W", target_pose=Pose2D(x=x, y=y, theta=-2.0),
                      mode=MoveTo.Goal.MODE_NAV_GOAL)
    fut = p.move.send_goal_async(msg)
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    wait = move_wait(path_m("P-1", "N_W"), clock=FAST_CLOCK) + turn_wait(math.pi)
    assert result_of(fut.result(), wait).result.success is True
    assert wait_until(lambda: abs(math.remainder(seen[-1][3] + 2.0, math.tau)) < 1e-6, 2.0)


def test_progress_feedback_runs_on_sim_time(harness, tmp_path):
    """FEEDBACK_PERIOD_S는 시뮬 초다. 시계가 4배면 실제 1초에 소식이 4배로 온다."""
    from fake_robots.pode_goals import FEEDBACK_PERIOD_S
    p = start(harness, tmp_path, clock=FAST_CLOCK, robots=ROBOT)
    assert wait_until(lambda: p.states, 5.0)
    p.goal("P-3")
    assert wait_until(lambda: p.feedback, 5.0)
    first = len(p.feedback)
    time.sleep(1.0)
    per_wall_s = len(p.feedback) - first
    assert per_wall_s >= 0.6 * FAST_CLOCK / FEEDBACK_PERIOD_S, per_wall_s


def test_a_segment_heading_reads_the_full_quaternion():
    """실물 quaternion_yaw와 같다: x, y 성분이 있어도 yaw만 꺼낸다."""
    from fake_robots.pode_order import segment
    roll, pitch, yaw = 0.3, 0.2, 1.0
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    msg = nav_seg(1)
    q = msg.target_pose.pose.orientation
    q.w, q.x = cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy
    q.y, q.z = cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy
    assert segment(msg).yaw == pytest.approx(yaw)


def test_the_pass_rule_measures_from_the_reported_pose():
    """실물은 넘김과 다음 path의 연결 검사를 둘 다 보고 자세로 잰다(drift가 들어간 자리)."""
    from types import SimpleNamespace

    from fake_robots.motion import Motion
    from fake_robots.pode_order import FOLLOW_PATH, NAV_GOAL, PodeOrder, Seg
    end = Seg(1, NAV_GOAL, "a", (), 1.0, 0.0)
    # 진짜 자리(0.9, 0)는 다음 path에서 0.141 m, 보고 자리(0.9, 0.05)는 0.180 m(허용 0.15).
    # 끝 노드까지는 둘 다 넘김 거리 안이다
    nxt = Seg(2, FOLLOW_PATH, "b", ((1.0, -0.1), (2.0, -0.1)), 2.0, -0.1)
    goal = SimpleNamespace(seg_mode=NAV_GOAL, seg=end, order=SimpleNamespace(queue=[nxt]))

    def node(drift):
        return SimpleNamespace(_motion=Motion(0.9, 0.0, yaw=0.0), _pass_m=0.15,
                               _connect_tolerance=0.15,
                               _faults=SimpleNamespace(drift=lambda t: drift))
    assert PodeOrder._passes(node(0.0), goal, 0.0) is True
    assert PodeOrder._passes(node(0.05), goal, 0.0) is False, "the next path would be refused"


def test_feedback_is_due_again_after_the_sim_clock_goes_back():
    from fake_robots.pode_goals import FEEDBACK_PERIOD_S, feedback_due
    assert feedback_due(10.0, 9.75) and FEEDBACK_PERIOD_S < 0.25
    assert not feedback_due(10.0, 9.9)
    assert feedback_due(0.5, 100.0), "a rewound clock would stop feedback for 100 s"
