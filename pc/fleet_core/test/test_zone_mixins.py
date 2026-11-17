"""구역 교통의 믹스인들은 같은 이름의 메서드를 두 번 정의하지 않는다.

믹스인이 열 개가 넘는다. 뜻이 다른 같은 이름이 생기면 앞에 선 믹스인의 것이 조용히 이긴다.
브랜치를 합칠 때 실제로 있었다: 도크 차례의 _ahead와 보기(views)의 _ahead. 일부러 덮어쓰는
것도 두지 않는다(예전의 Silent._event 가로채기는 ZoneBase._event가 SilentWatch.note를 직접
부르게 바꿨다). 주문 코어와 FleetManager도 믹스인으로 짜여 있어 같은 검사를 한다.
"""
from collections import defaultdict

from fleet_core.manager import FleetManager
from fleet_core.orders.orchestrator import OrderOrchestrator
from fleet_core.traffic.zone.controller import ZoneTraffic


def _defined_twice(top):
    owners = defaultdict(list)
    for cls in top.__mro__[:-1]:
        for name, value in vars(cls).items():
            if callable(value) or isinstance(value, staticmethod | classmethod | property):
                owners[name].append(cls.__name__)
    return {name: who for name, who in owners.items() if len(who) > 1}


def test_no_two_mixins_define_the_same_method():
    assert _defined_twice(ZoneTraffic) == {}


def test_the_order_core_mixins_define_no_method_twice():
    assert [cls.__name__ for cls in OrderOrchestrator.__mro__[1:-1]] == ["Faults", "Stations"]
    assert _defined_twice(OrderOrchestrator) == {}


def test_the_fleet_manager_mixins_define_no_method_twice():
    assert [cls.__name__ for cls in FleetManager.__mro__[1:-1]] == [
        "ManagerRoute", "ManagerViews", "ManagerRecord"]
    assert _defined_twice(FleetManager) == {}


def test_the_check_can_fail():
    class First:
        def step(self):
            pass

    class Second:
        def step(self):
            pass

    class Both(First, Second):
        pass

    assert _defined_twice(Both) == {"step": ["First", "Second"]}
