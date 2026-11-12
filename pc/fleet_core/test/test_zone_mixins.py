"""구역 교통의 믹스인들은 같은 이름의 메서드를 두 번 정의하지 않는다.

믹스인이 열 개가 넘는다. 뜻이 다른 같은 이름이 생기면 앞에 선 믹스인의 것이 조용히 이긴다.
브랜치를 합칠 때 실제로 있었다: 도크 차례의 _ahead와 보기(views)의 _ahead. 일부러 덮어쓰는
것만 OVERRIDES에 적는다.
"""
from collections import defaultdict

from fleet_core.traffic.zone.controller import ZoneTraffic

OVERRIDES = {"_event": ["Silent", "ZoneBase"]}  # Silent가 사건을 듣고 super()로 넘긴다


def test_no_two_mixins_define_the_same_method():
    owners = defaultdict(list)
    for cls in ZoneTraffic.__mro__[:-1]:
        for name, value in vars(cls).items():
            if callable(value) or isinstance(value, staticmethod | classmethod | property):
                owners[name].append(cls.__name__)
    assert {name: who for name, who in owners.items() if len(who) > 1} == OVERRIDES
