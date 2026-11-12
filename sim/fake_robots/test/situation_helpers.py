"""상황 시험 도우미: 상황 하나를 돌리고 못 지킨 기대를 마지막 사건과 함께 보여 준다."""
from fake_robots.world.table import run_situation

SEEDS = (1, 2, 3)
ALL_LAYOUTS = ("d2", "d", "c")


def check(name, layout, seed, avoid_mode="perfect"):
    outcome = run_situation(name, layout, seed, avoid_mode=avoid_mode)
    last = [f"{t:.1f} {e.code} {e.robot} {e.detail}" for t, e in outcome.world.events[-15:]]
    assert outcome.failures == [], "\n".join([*outcome.failures, "-- last events --", *last])
    return outcome
