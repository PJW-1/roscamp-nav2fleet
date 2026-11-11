"""1단계(뼈대)에서 비워 둔 두 자리와, 그 자리를 기다리는 시험.

교통 층(3단계)과 지도(2단계)가 오기 전에는 그것이 필요한 시험을 건너뛴다. 이유는 아래 두
문자열뿐이고, 건너뛰는 시험 모듈은 WAITING 표 한 곳에 적는다. 되살릴 때는 이 이름
(NEEDS_ZONE_TRAFFIC, NEEDS_FLEET_MAP)으로 찾는다. 함수 하나만 건너뛸 때는
`@pytest.mark.skip(reason=NEEDS_...)`를 쓴다.

각 패키지의 test/conftest.py가 skip_waiting_module을 pytest_pycollect_makemodule로 건다.
pytest는 그 함수 안에서만 읽어, 이 모듈은 pytest 없이도 읽힌다.
"""
from __future__ import annotations

NEEDS_ZONE_TRAFFIC = "needs the zone traffic layer (stage 3)"
NEEDS_FLEET_MAP = "needs a fleet map (stage 2)"

# (패키지 폴더, 시험 파일) -> 이유. 교통 층과 지도가 둘 다 필요하면 교통 층(나중 단계)으로 적는다.
WAITING = {
    **dict.fromkeys((("fleet_core", name) for name in (
        "test_auto_isolation.py", "test_battery_charge.py", "test_deadlock_cycle.py",
        "test_final_goal.py", "test_fleet_e2e.py", "test_fleet_fuzz.py", "test_fleet_unit.py",
        "test_goal_retry.py", "test_idle_watch.py",
        "test_path_connection.py", "test_queue_exit.py", "test_record_replay.py",
        "test_reinstate.py", "test_retreat_plumbing.py", "test_self_recovery.py",
        "test_sim_battery.py", "test_sim_world_object.py", "test_sim_world_retreat.py",
        "test_stall_watch.py", "test_traffic_controller.py")), NEEDS_ZONE_TRAFFIC),
    ("fleet_core", "test_selector.py"): NEEDS_FLEET_MAP,
}


def waiting_reason(module_path) -> str | None:
    """이 시험 모듈이 기다리는 이유, 기다리지 않으면 None. 경로는 <패키지>/test/<파일>이다."""
    return WAITING.get((module_path.parent.parent.name, module_path.name))


def skip_waiting_module(module_path, parent):
    """pytest_pycollect_makemodule 훅 본문. 기다리는 모듈은 읽지 않고, 모듈마다 건너뛴 시험
    하나(`waiting`)로 보고한다. 보고의 자리는 그 시험 파일이다.

    읽지 않으므로 지운 모듈(lane_map, controller 등)을 임포트하는 옛 시험도 그대로 둘 수 있다.
    """
    reason = waiting_reason(module_path)
    if reason is None:
        return None
    import pytest

    class Waiting(pytest.Item):
        def runtest(self):
            raise pytest.skip.Exception(reason, _use_item_location=True)

        def reportinfo(self):
            return self.path, 0, f"{self.path.name}: {reason}"

    class WaitingModule(pytest.Module):
        def collect(self):
            return [Waiting.from_parent(self, name="waiting")]

    return WaitingModule.from_parent(parent, path=module_path)
