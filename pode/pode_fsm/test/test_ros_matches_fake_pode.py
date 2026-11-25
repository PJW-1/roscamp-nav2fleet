"""같은 목표 순서에 FakePode와 pode_fsm(+FakeNav)이 같은 결과와 상태 값을 내는지 본다.

FakePode는 관제가 시험받은 기준 동작이다. fake_robots는 이 비교에만 쓰고 pode_fsm 코드는 import하지 않는다.
비교: 레인 goal, 교체, 취소, 점 goal, heartbeat 끊김, PATH_NOT_CONNECTED, BLOCKED, 시작 때 LOCALIZING.
알려진 차이는 KNOWN_DIFFERENCES에 있다."""
import time

import pytest
import yaml

pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs.action")
pytest.importorskip("fake_robots.fake_pode_node")

# 옛 레인 지도로 쓴 시험이라 FakePode를 띄울 수 없다. 구역 지도로 다시 써야 한다
pytestmark = pytest.mark.skip(reason="written for the lane map; not yet rewritten for the zone map")

from conftest import CONTRACT, Harness, wait_until  # noqa: E402
from fake_nav import FakeNav  # noqa: E402
from fleet_interfaces.action import Dock  # noqa: E402
from fleet_interfaces.msg import RobotState  # noqa: E402
from probe import Probe, lane_goal  # noqa: E402

SPEED = 0.5
LOCALIZING_S = 3.0  # FakePode는 시작 뒤 이만큼 LOCALIZING을 보고한다
OBSTACLE = (0.35, 1.75)  # P-3의 로봇 앞 0.2 m, P-4로 가는 레인 위

# 일부러 다르거나 아직 없는 동작. 시험과 함께 고친다
KNOWN_DIFFERENCES = {
    "dock": "pode_fsm has no pode_dock yet and fails every dock (DOCK_UNAVAILABLE in the log); "
            "FakePode succeeds after dock.duration_s. Precise docking is a separate task.",
    "current_goal while docking": "both report the dock_id (Pode contract section 2), but "
                                  "pode_fsm never reaches DOCKING, so it is not compared.",
    "goal while LOCALIZING": "pode_fsm fails it with NAV_ABORTED; FakePode drives it. The fleet "
                             "manager sends no goal to a LOCALIZING robot.",
    "pose lost while moving": "pode_fsm stops and fails with NAV_ABORTED; FakePode has no fault "
                              "that loses the pose.",
}


class World:
    """한 단계가 로봇 둘레에서 바꿀 수 있는 것: 앞 장애물, 자세가 알려짐."""

    def __init__(self, block, localize):
        self.block = block
        self.localize = localize


def scenario_file(tmp_path, localizing_s=0.0):
    robot = {"speed_mps": SPEED, "pose_noise_m": 0.0}
    data = {
        "seed": 3, "speed_factor": 1.0, "timeout_s": 600,
        "stock": {"arm_left": {"A": 3}, "arm_right": {"C": 3}},
        "robots": {"pode1": {"start": "P-3", "localizing_s": localizing_s, **robot},
                   "pode2": {"start": "P-5", **robot},
                   "pode3": {"start": "P-4", **robot}},
        "arm": {"seconds_per_item": 0.2, "swap_seconds": 0.4, "fail_rate": 0.0},
        "dock": {"duration_s": 0.6, "error_cm": 1.0},
    }
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return str(path)


def fake_pode(harness, tmp_path, heartbeat_timeout_s, localizing):
    from fake_robots.fake_pode_node import FakePodeNode
    node = harness.add(FakePodeNode(parameter_overrides=harness.params(
        robot_id="pode1", contract_file=str(CONTRACT),
        scenario_file=scenario_file(tmp_path, LOCALIZING_S if localizing else 0.0),
        state_rate_hz=10.0, heartbeat_timeout_s=heartbeat_timeout_s)))
    blocked = [False]
    # 시험용 훅: 장애물은 결함 일정이 아니라 단계가 시킬 때 오간다
    node._faults.obstacle = lambda _t: blocked[0]
    # FakePode는 LOCALIZING_S 뒤 LOCALIZING 보고를 스스로 멈춘다
    return World(block=lambda on: blocked.__setitem__(0, on), localize=lambda: None)


def real_fsm(harness, _tmp_path, heartbeat_timeout_s, localizing):
    from pode_fsm.node import PodeFsmNode
    nav = harness.add(FakeNav(speed=SPEED, publish_tf=not localizing))
    harness.add(PodeFsmNode(parameter_overrides=harness.params(
        robot_id="pode1", contract_file=str(CONTRACT),
        heartbeat_timeout_s=heartbeat_timeout_s)))

    def localize():
        nav.publish_tf = True

    return World(block=lambda on: setattr(nav, "obstacle", OBSTACLE if on else None),
                 localize=localize)


def run(rclpy_session, tmp_path, make, steps, heartbeat_timeout_s=0.0, localizing=False):
    harness = Harness(rclpy_session)
    try:
        world = make(harness, tmp_path, heartbeat_timeout_s, localizing)
        probe = harness.add(Probe())
        assert probe.ready()
        if not localizing:
            assert wait_until(lambda: (s := probe.last_state()) is not None
                              and s.state == RobotState.STATE_IDLE, 10.0)
        return steps(probe, world)
    finally:
        harness.close()


def outcome(probe, handle, t0):
    res = probe.result(handle).result
    time.sleep(0.3)  # 끝난 뒤의 상태도 본다
    return res.success, res.reason, sorted(set(probe.states_since(t0)))


def last_state_is(probe, state, timeout):
    return wait_until(lambda: (s := probe.last_state()) is not None and s.state == state, timeout)


def stood_still(probe, since):
    poses = {(round(m.pose.x, 3), round(m.pose.y, 3)) for t, m in probe.states if t >= since}
    return len(poses) == 1


def lane_steps(probe, _world):
    out = {}
    t0 = time.monotonic()
    out["arrive"] = outcome(probe, probe.send(lane_goal(["P-3", "P-4"])), t0)

    t0 = time.monotonic()
    first = probe.send(lane_goal(["P-4", "P-5"]))
    time.sleep(0.3)
    second = probe.send(lane_goal(["P-4", "P-5", "P-6"]))
    out["replaced"] = probe.result(first).result.success, probe.result(first).result.reason
    out["extended"] = outcome(probe, second, t0)

    t0 = time.monotonic()
    h = probe.send(lane_goal(["P-6", "N"]))
    time.sleep(0.4)
    h.cancel_goal_async()
    out["canceled"] = outcome(probe, h, t0)

    t0 = time.monotonic()
    out["point"] = outcome(probe, probe.send(lane_goal(["P-6", "N"], path=False)), t0)
    return out


def heartbeat_steps(probe, _world):
    t0 = time.monotonic()
    return {"lost": outcome(probe, probe.send(lane_goal(["P-3", "P-4", "P-5", "P-6"])), t0)}


def path_steps(probe, _world):
    out = {}
    t0 = time.monotonic()
    out["idle"] = outcome(probe, probe.send(lane_goal(["P-5", "P-6"])), t0)  # 0.67 m 떨어짐

    t0 = time.monotonic()
    first = probe.send(lane_goal(["P-3", "P-4", "P-5", "P-6"]))
    time.sleep(0.3)
    far = probe.send(lane_goal(["P-6", "N"]))
    out["replaced"] = probe.result(first).result.success, probe.result(first).result.reason
    out["moving"] = outcome(probe, far, t0)
    mark = time.monotonic()
    time.sleep(0.3)
    out["still"] = stood_still(probe, mark)
    return out


def blocked_steps(probe, world):
    out = {}
    goal = lane_goal(["P-3", "P-4", "P-5"])
    world.block(True)
    h = probe.send(goal)
    assert last_state_is(probe, RobotState.STATE_BLOCKED, 8.0)
    h.cancel_goal_async()
    res = probe.result(h).result
    mark = time.monotonic()
    time.sleep(0.5)
    out["canceled"] = res.success, res.reason, sorted(set(probe.states_since(mark)))
    world.block(False)
    out["cleared"] = last_state_is(probe, RobotState.STATE_IDLE, 8.0)

    world.block(True)
    t0 = time.monotonic()
    h = probe.send(goal)
    assert last_state_is(probe, RobotState.STATE_BLOCKED, 8.0)
    world.block(False)
    out["resumed"] = outcome(probe, h, t0)
    return out


def localizing_steps(probe, world):
    assert wait_until(lambda: probe.states, 5.0)
    first = probe.states[0][1].state
    time.sleep(1.0)
    world.localize()
    assert last_state_is(probe, RobotState.STATE_IDLE, 10.0)
    localizing = [m for _, m in probe.states if m.state == RobotState.STATE_LOCALIZING]
    return {"first": first, "states": sorted(set(probe.states_since(0.0))),
            "zero_pose": all(m.pose.x == 0.0 and m.pose.y == 0.0 for m in localizing)}


def dock_steps(probe, _world):
    return probe.result(probe.send(Dock.Goal(dock_id="WORK"), client=probe.dock)).result.success


def test_lane_goals_give_the_same_results_as_fake_pode(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pode, lane_steps)
    real = run(rclpy_session, tmp_path, real_fsm, lane_steps)
    assert real == fake
    assert fake["arrive"] == (True, "", ["IDLE", "MOVING"])
    assert fake["replaced"] == (False, "CANCELED")
    assert fake["canceled"][:2] == (False, "CANCELED")


def test_heartbeat_loss_gives_the_same_result_as_fake_pode(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pode, heartbeat_steps, heartbeat_timeout_s=1.0)
    real = run(rclpy_session, tmp_path, real_fsm, heartbeat_steps, heartbeat_timeout_s=1.0)
    assert real == fake
    assert fake["lost"][:2] == (False, "HEARTBEAT_LOST")


def test_a_path_away_from_the_robot_gives_the_same_result_as_fake_pode(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pode, path_steps)
    real = run(rclpy_session, tmp_path, real_fsm, path_steps)
    assert real == fake
    assert fake["idle"] == (False, "PATH_NOT_CONNECTED", ["IDLE"])
    assert fake["replaced"] == (False, "CANCELED")
    assert fake["moving"][:2] == (False, "PATH_NOT_CONNECTED")
    assert fake["still"] is True


def test_an_obstacle_gives_the_same_blocked_behaviour_as_fake_pode(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pode, blocked_steps)
    real = run(rclpy_session, tmp_path, real_fsm, blocked_steps)
    assert real == fake
    assert fake["canceled"] == (False, "CANCELED", ["BLOCKED"])
    assert fake["cleared"] is True
    assert fake["resumed"] == (True, "", ["BLOCKED", "IDLE", "MOVING"])


def test_localizing_at_start_matches_fake_pode(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pode, localizing_steps, localizing=True)
    real = run(rclpy_session, tmp_path, real_fsm, localizing_steps, localizing=True)
    assert real == fake
    assert fake == {"first": "LOCALIZING", "states": ["IDLE", "LOCALIZING"], "zero_pose": True}


def test_dock_is_still_a_known_difference(rclpy_session, tmp_path):
    """KNOWN_DIFFERENCES["dock"]을 고정한다. pode_dock이 생기면 실패하니 그때 도킹을 비교한다."""
    assert run(rclpy_session, tmp_path, fake_pode, dock_steps) is True
    assert run(rclpy_session, tmp_path, real_fsm, dock_steps) is False
