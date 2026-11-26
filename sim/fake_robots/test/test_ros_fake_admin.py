"""FakePode의 관리 명령 서버: set_lock · force · charge_end가 답하고 상태 칸에 실린다."""
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import wait_until  # noqa: E402
from fleet_interfaces.srv import ForceInput, RobotCommand, SetLock  # noqa: E402
from test_ros_pode import CONTRACT_OBJ, _set_mode, start  # noqa: E402


def ask(p, srv, service, field, **request):
    client = p.node.create_client(srv, CONTRACT_OBJ.name(service, **{field: "pode1"}))
    assert client.wait_for_service(10.0)
    fut = client.call_async(srv.Request(**request))
    assert wait_until(fut.done, 5.0)
    return fut.result()


def test_set_lock_releases_a_standing_robot_and_the_state_shows_it(harness, tmp_path):
    p = start(harness, tmp_path)
    assert wait_until(lambda: p.states and p.states[-1][1].lock, 5.0)
    res = ask(p, SetLock, "robot_set_lock", "robot", request_id="l1", robot_id="pode1",
              locked=False)
    assert res.accepted and res.request_id == "l1"
    assert wait_until(lambda: not p.states[-1][1].lock, 5.0)
    assert p.states[-1][1].last_request_id == "l1"
    # 같은 번호는 같은 답, 다시 반영하지 않는다
    again = ask(p, SetLock, "robot_set_lock", "robot", request_id="l1", robot_id="pode1",
                locked=True)
    assert again.accepted
    assert not p.states[-1][1].lock


def test_set_lock_refuses_a_robot_with_a_goal(harness, tmp_path):
    p = start(harness, tmp_path)
    assert p.goal("O_Q").accepted
    res = ask(p, SetLock, "robot_set_lock", "robot", request_id="l2", robot_id="pode1",
              locked=False)
    assert (res.accepted, res.reason) == (False, "NOT_STOPPED")


def test_force_and_charge_end_need_the_test_mode(harness, tmp_path):
    p = start(harness, tmp_path)
    res = ask(p, ForceInput, "robot_force", "robot", request_id="f1", robot_id="pode1",
              kind="CONDITION", key="battery", value="20")
    assert res.reason == "NOT_TEST_MODE"
    res = ask(p, RobotCommand, "pode_charge_end", "pode", request_id="c1", robot_id="pode1")
    assert res.reason == "NOT_TEST_MODE"
    assert _set_mode(p, "TEST", "m1").accepted
    res = ask(p, ForceInput, "robot_force", "robot", request_id="f2", robot_id="pode1",
              kind="CONDITION", key="battery", value="20")
    assert res.accepted
    assert wait_until(lambda: list(p.states[-1][1].forced) == ["battery"], 5.0)
    assert ask(p, RobotCommand, "pode_charge_end", "pode", request_id="c2",
               robot_id="pode1").accepted
