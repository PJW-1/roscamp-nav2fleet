"""상황 시험 도우미: 상황 하나를 돌리고 못 지킨 기대를 마지막 사건과 함께 보여 준다."""
import pytest
from fake_robots.world.table import run_situation

SEEDS = (1, 2, 3)
ALL_LAYOUTS = ("d2", "d", "c")


# 열린 문제: 튄 보고 위치로 관제가 로봇이 아직 서 있는 구역을 놓는다(I3).
# 관제를 고쳐 통과하면 XPASS로 실패하니 표시를 지운다
POSE_JUMPS_OPEN = ("열린 문제: 튄 보고 위치로 로봇이 서 있는 구역을 놓는다(I3_OUTSIDE_HELD). "
                   "order 모델에서 잦다. 비전 위치에 묶거나 LOCALIZATION_SUSPECT 뒤 구역을 "
                   "쥐는 고침은 사용자 결정을 기다린다")
# 끈 길에서 이 열린 문제가 나는 (배치, 씨드). c 씨드 2는 도크 후진 중 튄 보고로 WORK_DOCK을 놓는다. 이 칸은 끈 길도 strict xfail이다
POSE_JUMPS_OFF_OPEN = {("c", 2)}


def pose_jumps_off_marks(layout, seed) -> list:
    if (layout, seed) not in POSE_JUMPS_OFF_OPEN:
        return []
    return [pytest.mark.xfail(strict=True, raises=PoseJumpsOpen, reason=POSE_JUMPS_OPEN)]


class PoseJumpsOpen(AssertionError):
    """pose_jumps가 I3_OUTSIDE_HELD 말고는 다 지켰다."""


def check(name, layout, seed, avoid_mode="perfect", overrides=None):
    outcome = run_situation(name, layout, seed, avoid_mode=avoid_mode, overrides=overrides)
    last = [f"{t:.1f} {e.code} {e.robot} {e.detail}" for t, e in outcome.world.events[-15:]]
    assert outcome.failures == [], "\n".join([*outcome.failures, "-- last events --", *last])
    return outcome


def check_pose_jumps(layout, seed, avoid_mode="perfect", order=True):
    """order를 켠 pose_jumps: I3_OUTSIDE_HELD는 PoseJumpsOpen으로, 다른 실패는 그대로 낸다."""
    failures = run_situation("pose_jumps", layout, seed, avoid_mode=avoid_mode,
                             overrides={"use_order_model": order}).failures
    assert [f for f in failures if not f.startswith("I3_OUTSIDE_HELD")] == [], failures
    if failures:
        raise PoseJumpsOpen(failures[0])
