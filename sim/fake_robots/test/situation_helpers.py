"""상황 시험 도우미: 상황 하나를 돌리고 못 지킨 기대를 마지막 사건과 함께 보여 준다."""
import pytest
from fake_robots.world.table import run_situation

SEEDS = (1, 2, 3)
ALL_LAYOUTS = ("d2", "d", "c")


# order를 켠 pose_jumps(2026-11-16): suspect_hold=check(사용자 결정 B)로 튄 뒤 구역을 쥐어 I3가
# 없다. d의 씨드 1, 3은 로봇이 대기 자리(W_Q)에 선 동안 보고 위치가 20 s 틀린 채로 남는다. base 끝
# 확인이 NOT_AT_TARGET 세 번 뒤 그 로봇을 격리해 주문을 다 못 끝낸다(설계대로). 그 칸은 strict
# xfail이다. 통과하면 XPASS로 실패하니 줄을 지운다
POSE_JUMPS_ISOLATED = {("d", 1), ("d", 3)}
POSE_JUMPS_ISOLATED_WHY = ("B: 대기 자리에서 보고 위치가 20 s 틀린 채로 남아 NOT_AT_TARGET 세 번 뒤 "
                           "격리된다. 주문을 다 못 끝낸다")


class PoseJumpsIsolated(AssertionError):
    """pose_jumps가 불변식은 다 지켰고, NOT_AT_TARGET 격리로 주문만 다 못 끝냈다."""


def check(name, layout, seed, avoid_mode="perfect", overrides=None):
    outcome = run_situation(name, layout, seed, avoid_mode=avoid_mode, overrides=overrides)
    last = [f"{t:.1f} {e.code} {e.robot} {e.detail}" for t, e in outcome.world.events[-15:]]
    assert outcome.failures == [], "\n".join([*outcome.failures, "-- last events --", *last])
    return outcome


def check_pose_jumps(layout, seed, avoid_mode="perfect", order=True):
    """order를 켠 pose_jumps: 불변식(I3 포함)은 늘 지킨다. 주문 못 끝냄은 NOT_AT_TARGET 격리가
    있었을 때만 PoseJumpsIsolated로 낸다."""
    outcome = run_situation("pose_jumps", layout, seed, avoid_mode=avoid_mode,
                            overrides={"use_order_model": order})
    failures = outcome.failures
    assert [f for f in failures if not f.startswith("finished ")] == [], failures
    if failures:
        isolated = [e for _, e in outcome.world.events
                    if e.code == "ROBOT_ISOLATED" and e.detail.startswith("NOT_AT_TARGET")]
        assert isolated, failures
        raise PoseJumpsIsolated(failures[0])


def pose_jumps_marks(layout, seed) -> list:
    """격리로 못 끝내는 칸만 strict xfail."""
    if (layout, seed) not in POSE_JUMPS_ISOLATED:
        return []
    return [pytest.mark.xfail(strict=True, raises=PoseJumpsIsolated,
                              reason=POSE_JUMPS_ISOLATED_WHY)]
