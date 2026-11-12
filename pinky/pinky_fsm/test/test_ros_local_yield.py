"""로봇단 교착 처리(추가 지시서 3의 7-4, 계약 0.13-draft)를 실제 토픽과 액션으로 본다.

/fleet/robots(RobotStatusArray)로 앞을 막은 로봇과 그 priority를 주고, MoveTo.priority로 이 로봇의
값을 준다. 상태(RobotState)의 state와 local_yield를 본다. Nav2 대신 FakeNav를 쓴다.
"""
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")
pytest.importorskip("nav2_msgs.action")

from conftest import wait_until  # noqa: E402
from fleet_interfaces.msg import RobotState, RobotStatus, RobotStatusArray  # noqa: E402
from pinky_fsm.contract import to_ros_qos  # noqa: E402
from probe import C, lane_goal  # noqa: E402
from test_ros_fsm import start  # noqa: E402


def fleet_robots(probe, peers):
    """관제처럼 /fleet/robots를 0.5 s마다 낸다. peers는 (이름, x, y, priority)."""
    pub = probe.create_publisher(RobotStatusArray, C.name("robots"), to_ros_qos(C.qos("robots")))
    msg = RobotStatusArray(robots=[RobotStatus(robot_id=n, x=x, y=y, priority=p)
                                   for n, x, y, p in peers])
    probe.create_timer(0.5, lambda: pub.publish(msg))
    pub.publish(msg)


def steps(probe):
    return [(m.state, m.local_yield) for _, m in probe.states]


def test_the_lower_robot_waits_backs_up_and_reports_each_step(harness):
    # FakeNav의 레인은 짧다(P-3에서 0.15 m에 선다). release_distance_m=0으로 물러날 자리를 만든다
    fake, probe = start(harness, local_yield=True, release_distance_m=0.0)
    fleet_robots(probe, [("pinky2", 0.35, 1.90, 5)])
    fake.obstacle = (0.35, 1.95)
    goal = lane_goal(["P-3", "P-4", "P-5"])
    goal.priority = 1
    h = probe.send(goal)
    assert wait_until(lambda: ("RECOVERING", "wait") in steps(probe), 5.0)
    assert wait_until(lambda: ("RECOVERING", "backup") in steps(probe), 5.0)
    assert [d for _, d, _ in fake.backups] == [pytest.approx(0.10, abs=0.005)]
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_BLOCKED, 12.0)
    assert probe.last_state().local_yield == ""
    assert len(fake.backups) == 1 and not h.get_result_async().done()
    fake.obstacle = None
    assert probe.result(h, timeout=10.0).result.success is True


def test_the_higher_robot_only_waits_then_reports_blocked(harness):
    fake, probe = start(harness, local_yield=True, release_distance_m=0.0)
    fleet_robots(probe, [("pinky2", 0.35, 1.90, 0)])
    fake.obstacle = (0.35, 1.95)
    goal = lane_goal(["P-3", "P-4", "P-5"])
    goal.priority = 1
    probe.send(goal)
    assert wait_until(lambda: ("RECOVERING", "wait") in steps(probe), 5.0)
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_BLOCKED, 5.0)
    assert fake.backups == [] and ("RECOVERING", "backup") not in steps(probe)
