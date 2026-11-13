"""상황 시험 도우미: 상황 하나를 돌리고 못 지킨 기대를 마지막 사건과 함께 보여 준다."""
from fake_robots.world.table import run_situation

SEEDS = (1, 2, 3)
ALL_LAYOUTS = ("d2", "d", "c")

# order 모델에서 pose_jumps가 깨는 것. 관제를 고쳐 통과하면 XPASS로 실패하니 표시를 지운다
POSE_TRUSTED = ("order 모델은 로봇이 끝냈다고 한 조각(SEGMENT_DONE)을 "
                "보고 위치와 맞춰 보지 않는다. 튄 보고로도 조각을 이어 가고, "
                "그 보고로 로봇이 실제로 서 있는 구역을 놓는다(I3). "
                "끄면 조각 끝에서 NOT_AT_TARGET으로 서서 격리된다. 정할 일이다")


class PoseTrusted(AssertionError):
    """pose_jumps가 I3_OUTSIDE_HELD 말고는 다 지켰다."""


def check(name, layout, seed, avoid_mode="perfect"):
    outcome = run_situation(name, layout, seed, avoid_mode=avoid_mode)
    last = [f"{t:.1f} {e.code} {e.robot} {e.detail}" for t, e in outcome.world.events[-15:]]
    assert outcome.failures == [], "\n".join([*outcome.failures, "-- last events --", *last])
    return outcome


def check_pose_jumps(layout, seed, avoid_mode="perfect"):
    """I3_OUTSIDE_HELD는 PoseTrusted로, 다른 실패는 그대로 실패로 낸다."""
    failures = run_situation("pose_jumps", layout, seed, avoid_mode=avoid_mode).failures
    assert [f for f in failures if not f.startswith("I3_OUTSIDE_HELD")] == []
    if failures:
        raise PoseTrusted(failures[0])
