"""goal을 받은 뒤 상태 기계에 넘기기 전에 락이 풀리면 시작하지 않는다 (받기 검사와 등록 사이 틈)."""
import threading
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")

from pode_fsm.server_ros import GoalServers  # noqa: E402


class _Fsm:
    def __init__(self):
        self.calls = []

    def move(self, *args, **kwargs):
        self.calls.append("move")
        return []


def test_a_goal_whose_lock_was_released_before_it_started_never_reaches_the_state_machine():
    servers = GoalServers.__new__(GoalServers)
    warnings, executed = [], []
    servers._node = SimpleNamespace(get_logger=lambda: SimpleNamespace(warn=warnings.append))
    servers._lock, servers._locked, servers._slots = threading.Lock(), lambda: False, {}
    servers._fsm, servers._run, servers._nm = _Fsm(), lambda cmds: None, None
    handle = SimpleNamespace(goal_id=SimpleNamespace(uuid=b"\x01" * 16), request=None,
                             execute=lambda: executed.append(1))
    servers._accepted(handle, "move")
    assert servers._fsm.calls == [] and executed == [1]
    assert servers._slots[b"\x01" * 16].done.is_set()
    assert warnings and "lock released" in warnings[0]
