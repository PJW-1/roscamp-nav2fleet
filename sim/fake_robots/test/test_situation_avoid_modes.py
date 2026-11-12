"""열 상황(지시서 6-2)을 회피 흉내 none, mixed로(추가 지시서 2의 1·3절).

perfect는 test_situation_*.py가 같은 판정(table.judge, Situation.for_mode)으로 돌린다.
none과 mixed는 3절 기준이다: 안전 불변식과 관제 반응 시간, 늦더라도 주문이 모두 끝남.

관제 탓으로 못 끝나는 칸은 아래 FLEET_FAILS에 모두 적고 strict xfail로 둔다. 관제를 고쳐 한 칸이
통과하면 XPASS로 실패하니 그 줄을 지운다. 그 칸에 주문 못 끝냄 말고 다른 실패(불변식, 반응 시간)가
생기면 xfail로 숨지 않고 실패한다. 처음의 열세 칸 가운데 열한 칸은 선 로봇이 쥔 지점을
거치는 길에 대기 시간을 치면서(goals._node_wait) 끝나게 됐다.
"""
import pytest
from fake_robots.world.situations import NAMES
from fake_robots.world.table import run_situation
from situation_helpers import ALL_LAYOUTS, SEEDS, check

MODES = ("none", "mixed")

# O_Q에서 출고 도크를 기다리는 로봇이 P-3 -> N_E 정적 지도 길 위에 있다. 막힌 로봇이 도크를
# 잡고 있고, _make_way는 비킬 주차 자리를 못 찾는다(P-2는 남이 잡음, P-1은 clear_of로 뺌).
QUEUE = "robot queued at O_Q on the static path P-3->N_E has no parking spot to yield to"

# (상황, 배치, 모드, 씨드): 짧은 이유
FLEET_FAILS = {
    ("express_obstacle", "d", "none", 1): QUEUE,
    ("express_obstacle", "d", "none", 3): QUEUE,
}


class Unfinished(AssertionError):
    """적어 둔 이유대로 주문만 못 끝냈다."""


def _cells():
    for name in NAMES:
        for layout in ALL_LAYOUTS:
            for mode in MODES:
                for seed in SEEDS:
                    reason = FLEET_FAILS.get((name, layout, mode, seed))
                    marks = ([pytest.mark.xfail(strict=True, raises=Unfinished, reason=reason)]
                             if reason else [])
                    yield pytest.param(name, layout, mode, seed, marks=marks,
                                       id=f"{name}-{layout}-{mode}-{seed}")


def test_every_listed_failure_is_a_real_cell():
    for name, layout, mode, seed in FLEET_FAILS:
        assert name in NAMES and layout in ALL_LAYOUTS and mode in MODES and seed in SEEDS


@pytest.mark.parametrize(("name", "layout", "mode", "seed"), list(_cells()))
def test_situation_in_avoid_mode(name, layout, mode, seed):
    if (name, layout, mode, seed) not in FLEET_FAILS:
        check(name, layout, seed, avoid_mode=mode)
        return
    failures = run_situation(name, layout, seed, avoid_mode=mode).failures
    others = [f for f in failures if not f.startswith("finished ")]
    assert others == []
    if failures:
        raise Unfinished(failures[0])
