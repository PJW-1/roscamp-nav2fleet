"""열 상황을 회피 흉내 none, mixed로 돌린다.

perfect는 test_situation_*.py가 돌린다. none과 mixed는 rough 기준이다(안전 불변식, 반응 시간, 늦더라도 주문 완료).
관제 탓으로 못 끝나는 칸은 아래 FLEET_FAILS에 적고 strict xfail로 둔다. 통과하면 XPASS로 실패하니 그 줄을 지운다.
"""
import pytest
from fake_robots.world.situations import NAMES
from fake_robots.world.table import run_situation
from situation_helpers import (
    ALL_LAYOUTS,
    POSE_JUMPS_OFF_OPEN,
    POSE_JUMPS_OPEN,
    SEEDS,
    PoseJumpsOpen,
    check,
    check_pose_jumps,
    pose_jumps_off_marks,
)

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
                    if name == "pose_jumps":
                        marks = pose_jumps_off_marks(layout, seed)
                    yield pytest.param(name, layout, mode, seed, None, marks=marks,
                                       id=f"{name}-{layout}-{mode}-{seed}")
                    if name == "pose_jumps":
                        # 위는 끈 길(order 끔)로 돈다. 켠 길은 열린 문제로 strict xfail이다
                        yield pytest.param(name, layout, mode, seed, True, marks=[
                            pytest.mark.xfail(strict=True, raises=PoseJumpsOpen,
                                              reason=POSE_JUMPS_OPEN)],
                            id=f"{name}-{layout}-{mode}-{seed}-order_on")


def test_every_listed_failure_is_a_real_cell():
    for name, layout, mode, seed in FLEET_FAILS:
        assert name in NAMES and layout in ALL_LAYOUTS and mode in MODES and seed in SEEDS


@pytest.mark.parametrize(("name", "layout", "mode", "seed", "order"), list(_cells()))
def test_situation_in_avoid_mode(name, layout, mode, seed, order):
    if order:
        check_pose_jumps(layout, seed, avoid_mode=mode)
        return
    if name == "pose_jumps" and (layout, seed) in POSE_JUMPS_OFF_OPEN:
        check_pose_jumps(layout, seed, avoid_mode=mode, order=False)
        return
    off = {"use_order_model": False} if name == "pose_jumps" else None
    if (name, layout, mode, seed) not in FLEET_FAILS:
        check(name, layout, seed, avoid_mode=mode, overrides=off)
        return
    failures = run_situation(name, layout, seed, avoid_mode=mode).failures
    others = [f for f in failures if not f.startswith("finished ")]
    assert others == []
    if failures:
        raise Unfinished(failures[0])
