"""열 상황(지시서 6-2)을 회피 흉내 none, mixed로(추가 지시서 2의 1·3절).

perfect는 test_situation_*.py가 같은 판정(table.judge, Situation.for_mode)으로 돌린다.
none과 mixed는 3절 기준이다: 안전 불변식과 관제 반응 시간, 늦더라도 주문이 모두 끝남.

관제 탓으로 못 끝나는 칸이 생기면 아래 FLEET_FAILS에 적고 strict xfail로 둔다. 관제를 고쳐 그 칸이
통과하면 XPASS로 실패하니 그 줄을 지운다. 그 칸에 주문 못 끝냄 말고 다른 실패(불변식, 반응 시간)가
생기면 xfail로 숨지 않고 실패한다.

처음에는 none의 열세 칸이 여기 있었다. 주차했거나 차례를 기다리며 선 로봇의 몸이 지도 길을 막은
경우들이다. 선 로봇이 쥔 지점을 거치는 길에 대기 시간을 치고(goals._node_wait), 먼저 불린 로봇이
뒤 차례 로봇이 선 지점을 기다리면 뒤 차례가 먼저 도크에 가게 해서(turns._is_next) 모두 끝난다.
"""
import pytest
from fake_robots.world.situations import NAMES
from fake_robots.world.table import run_situation
from situation_helpers import ALL_LAYOUTS, SEEDS, check

MODES = ("none", "mixed")

# (상황, 배치, 모드, 씨드): 짧은 이유
FLEET_FAILS: dict = {}


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


@pytest.mark.parametrize(("name", "seed"), [("order_burst", 2), ("comm_loss_reinstate", 2)])
def test_the_yard_back_off_does_not_stand_a_robot_on_a_node_it_does_not_hold(name, seed):
    """yard_backoff를 켠 none(d2)에서 물러난 로봇이 N_E 위에 섰던 두 칸(불변식 I8). 하나는 남이 이미
    잡은 지점이었고, 하나는 물러난 뒤에 남이 잡았다. 2-1에서 D2의 갈림목 선과 대기 자리가 옮겨진 뒤
    comm_loss_reinstate 3은 물러나기를 쓰지 않으므로 물러나기를 쓰는 씨드 2로 바꿨다."""
    outcome = run_situation(name, "d2", seed, overrides={"yard_backoff": True}, avoid_mode="none")
    assert outcome.failures == []
    assert outcome.world.metrics.count("HALL_BACKOFF_DONE") >= 1, "the back-off was used"
