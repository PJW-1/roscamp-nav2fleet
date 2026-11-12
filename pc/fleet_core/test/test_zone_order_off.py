"""use_order_model이 꺼져 있으면 order 모델을 넣기 전과 사건·명령 기록이 같다.

기준 기록(data/zone_off_golden.txt)은 order 모델이 없는 nav2-base의 코드와 시험 틀로 만든다.
지금 것은 추가 지시서 3의 2-1로 D2 지도를 바꾼 뒤의 것이다(갈림목 선 x 0.35와 1.63, 도크에서
마당까지 내려가는 후진). 그 앞은 06cc2db(추가 지시서 2), 처음은 59937d9였다.
교통 층의 동작을 일부러 바꿨을 때만 ZONE_GOLDEN_WRITE=1로 다시 만든다.
"""
import os
from pathlib import Path

import pytest
from zone_scenarios import SCENARIOS, play

GOLDEN = Path(__file__).parent / "data" / "zone_off_golden.txt"


def recorded() -> str:
    parts = []
    for name in SCENARIOS:
        world = play(name)
        assert not world.problems, world.problems[:3]
        parts.append(f"== {name}\n" + "\n".join(line.rstrip() for line in world.log))
    return "\n".join(parts) + "\n"


def test_the_order_model_is_off_by_default():
    assert play("blocked_express").params.use_order_model is False


def test_with_the_order_model_off_the_log_is_the_same_as_before():
    text = recorded()
    if os.environ.get("ZONE_GOLDEN_WRITE") == "1":
        GOLDEN.parent.mkdir(exist_ok=True)
        GOLDEN.write_text(text, encoding="utf-8", newline="\n")
        pytest.skip("golden log rewritten")
    assert text == GOLDEN.read_text(encoding="utf-8")
