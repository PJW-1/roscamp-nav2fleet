"""order 모델이 꺼져 있으면(use_order_model false) 세계는 order를 넣기 전과 똑같이 돈다.

data/world_off_golden.txt는 order 지원이 없는 nav2-base의 코드로 만든 사건 기록이다. 지금 것은
06cc2db의 것이다(추가 지시서 2로 관제의 동작을 바꾼 뒤. 처음은 world-order에 world를 합친 때였다).
추가 지시서 3의 2-1에서 D2의 갈림목 선, 도크 후진(마당까지), 대기 자리를 옮겨 다시 만들었다.
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
