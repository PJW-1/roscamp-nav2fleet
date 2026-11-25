"""콜백 안에서 서비스 응답을 기다리면 단일 스레드 실행기가 교착되는지 재현한다.

플릿 코드와 무관한 작은 노드 둘만 쓴다. 그래프에 참여하므로 로봇 네트워크와 분리된 도메인에서만 돈다.
"""
import os
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("std_srvs")

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402
from rclpy.context import Context  # noqa: E402
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_DOMAIN_ID", "") in ("", "6")
    or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST",
    reason="set ROS_DOMAIN_ID (not 6) and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST first")

LIMIT = 2.0  # 초
SRV = "/executor_test_trigger"


class Harness:
    """서버 노드는 자기 실행기에서, 클라이언트 노드는 시험용 실행기에서 돌린다."""

    def __init__(self, client_executor, client_group=None, timer_group=None):
        self.context = Context()
        rclpy.init(context=self.context)
        self.server = Node("exec_test_server", context=self.context)
        self.server.create_service(Trigger, SRV, self._handle)
        self.client_node = Node("exec_test_client", context=self.context)
        self.client = self.client_node.create_client(Trigger, SRV, callback_group=client_group)
        self.timer_group = timer_group
        self.server_exec = SingleThreadedExecutor(context=self.context)
        self.server_exec.add_node(self.server)
        self.client_exec = client_executor(context=self.context)
        self.client_exec.add_node(self.client_node)
        self.finished = threading.Event()
        self.done = None
        self.elapsed = None
        self.timer = None
        self.threads = [
            threading.Thread(target=self.server_exec.spin, daemon=True),
            threading.Thread(target=self.client_exec.spin, daemon=True),
        ]

    @staticmethod
    def _handle(_req, res):
        res.success = True
        return res

    def start(self, callback):
        """서버가 보일 때까지 기다린 뒤 타이머 콜백 한 번을 실행한다."""
        for t in self.threads:
            t.start()
        assert self.client.wait_for_service(timeout_sec=LIMIT * 2)

        def once():
            self.timer.cancel()
            callback()
            self.finished.set()

        self.timer = self.client_node.create_timer(0.05, once, callback_group=self.timer_group)

    def wait(self):
        """콜백이 끝나기를 제한 시간 안에서 기다린다."""
        return self.finished.wait(LIMIT * 3)

    def close(self):
        self.client_exec.shutdown()
        self.server_exec.shutdown()
        for t in self.threads:
            t.join(timeout=2.0)
        self.client_node.destroy_node()
        self.server.destroy_node()
        rclpy.shutdown(context=self.context)


def _wait_in_callback(h):
    """콜백 안에서 call_async 뒤 future.done()을 제한 시간까지 확인한다."""
    t0 = time.monotonic()
    future = h.client.call_async(Trigger.Request())
    while not future.done() and time.monotonic() - t0 < LIMIT:
        time.sleep(0.01)
    h.done = future.done()
    h.elapsed = time.monotonic() - t0


@pytest.fixture
def make():
    made = []

    def factory(*args, **kwargs):
        h = Harness(*args, **kwargs)
        made.append(h)
        return h

    yield factory
    for h in made:
        h.close()


def test_a_wait_in_a_callback_hangs_a_single_thread(make):
    """단일 스레드에서는 응답을 처리할 스레드가 없어 제한 시간까지 끝나지 않는다."""
    h = make(SingleThreadedExecutor)
    h.start(lambda: _wait_in_callback(h))
    assert h.wait()
    assert h.done is False
    assert h.elapsed >= LIMIT


def test_b_call_async_with_done_callback_works_on_a_single_thread(make):
    """call_async와 완료 콜백은 단일 스레드에서도 제한 시간 안에 응답을 받는다."""
    h = make(SingleThreadedExecutor)
    got = threading.Event()
    results = []

    def call():
        future = h.client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: (results.append(f.result().success), got.set()))

    h.start(call)
    assert got.wait(LIMIT)
    assert results == [True]


def test_c_separate_callback_groups_let_a_wait_finish(make):
    """다중 스레드와 서로 다른 콜백 그룹이면 같은 대기도 제한 시간 안에 끝난다."""
    h = make(MultiThreadedExecutor,
             client_group=MutuallyExclusiveCallbackGroup(),
             timer_group=MutuallyExclusiveCallbackGroup())
    h.start(lambda: _wait_in_callback(h))
    assert h.wait()
    assert h.done is True
    assert h.elapsed < LIMIT
