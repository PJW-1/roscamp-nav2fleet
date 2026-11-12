"""order 모델이 꺼져 있으면(use_order_model false) 세계는 order를 넣기 전과 똑같이 돈다.

data/world_off_golden.txt는 order 지원을 넣기 전(world-order에 world를 합친 때)의 사건 기록이다.
"""
from pathlib import Path

from fake_robots.world.table import run_situation

GOLDEN = Path(__file__).parent / "data" / "world_off_golden.txt"


def test_with_the_order_model_off_the_event_log_is_the_same_as_before():
    lines = []
    for name in ("three_at_junction", "outbound_slow"):
        outcome = run_situation(name, "d2", 1)
        assert not outcome.world.params.use_order_model
        lines += [f"{name} {t:.1f} {e.level} {e.robot} {e.code}: {e.detail}"
                  for t, e in outcome.world.events]
    assert lines == GOLDEN.read_text(encoding="utf-8").splitlines()
