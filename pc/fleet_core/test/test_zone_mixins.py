"""구역 교통의 믹스인들은 같은 이름의 메서드를 두 번 정의하지 않는다(앞 믹스인의 것이 조용히 이기므로). 주문 코어와 FleetManager도 같은 검사를 한다."""
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
