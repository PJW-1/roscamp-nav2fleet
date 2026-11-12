"""교착 안전망의 순수 함수: 대기 순환 찾기와 누가 양보하는지.

구역 교통 층 위의 교착 감시는 test_zone_watch.py와 test_zone_edges.py에 있다.
"""
from fleet_core.common.params import load_params
from fleet_core.traffic.deadlock import choose_retreat, find_cycle, priority


def test_t06_forced_cycle_is_found():
    """T-06: 두 로봇이 서로 상대가 쥔 구역을 기다린다."""
    graph = {"r1": {"r2"}, "r2": {"r1"}}
    assert set(find_cycle(graph)) == {"r1", "r2"}


def test_t06_three_robot_cycle_is_found():
    """T-06: 더 긴 순환은 구성원만 보고하고 주변 로봇은 뺀다."""
    graph = {"a": {"b"}, "b": {"c"}, "c": {"a"}, "d": {"a"}}
    assert set(find_cycle(graph)) == {"a", "b", "c"}


def test_t06_plain_queue_is_not_a_deadlock():
    """T-06: 대기 중이 아닌 점유자 뒤에 줄 선 것은 오탐이 아니다."""
    assert find_cycle({"r2": {"r1"}, "r3": {"r1", "r2"}}) is None
    assert find_cycle({}) is None


def test_t06_retreat_picks_lowest_priority():
    """T-06: 스테이션 +200, 적재 +100, 합이 가장 낮은 쪽이 물러난다."""
    params = load_params()
    scores = {
        "docked": priority(params, at_station=True, carrying=False),
        "full": priority(params, at_station=False, carrying=True),
        "empty": priority(params, at_station=False, carrying=False),
        "both": priority(params, at_station=True, carrying=True),
    }
    assert scores == {"docked": 200, "full": 100, "empty": 0, "both": 300}
    assert choose_retreat(["docked", "full", "empty"], scores.get) == "empty"
    assert choose_retreat(["docked", "full"], scores.get) == "full"
    assert choose_retreat(["both", "docked"], scores.get) == "docked"
