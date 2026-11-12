"""CheckClear를 부르지 못하면 tick 밖으로 예외를 내지 않고 NO_ANSWER를 준다(그래프 없이)."""
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces")

from fleet_core.common.commands import NO_ANSWER, CheckClearCmd  # noqa: E402
from fleet_manager.fleet_manager_node import FleetManagerNode  # noqa: E402


def node_with(client):
    answers = []
    core = SimpleNamespace(on_clear_check=lambda *args: answers.append(args))
    node = SimpleNamespace(_check_clear={"pinky1": client}, core=core,
                           get_logger=lambda: MagicMock())
    return node, answers


def test_a_call_that_raises_is_no_answer():
    client = MagicMock()
    client.service_is_ready.return_value = True
    client.call_async.side_effect = RuntimeError("context is shutting down")
    node, answers = node_with(client)
    FleetManagerNode._ask_clear(node, CheckClearCmd("pinky1", 7, 1.0, 0.65, 0.10))
    assert answers == [("pinky1", 7, NO_ANSWER)]


def test_a_missing_service_is_no_answer():
    client = MagicMock()
    client.service_is_ready.return_value = False
    node, answers = node_with(client)
    FleetManagerNode._ask_clear(node, CheckClearCmd("pinky1", 8, 1.0, 0.65, 0.10))
    assert answers == [("pinky1", 8, NO_ANSWER)] and not client.call_async.called
