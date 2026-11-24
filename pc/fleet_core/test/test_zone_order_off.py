"""use_order_model을 끄면 order 모델을 넣기 전과 사건·명령 기록이 같다(기본값은 켜짐이라 이 파일은 끈 값을 직접 넘긴다).
기준 기록(data/zone_off_golden.txt)은 교통 층의 동작을 일부러 바꿨을 때만 ZONE_GOLDEN_WRITE=1로 다시 만든다."""
import os
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from zone_scenarios import SCENARIOS, play

GOLDEN = Path(__file__).parent / "data" / "zone_off_golden.txt"
OFF = replace(load_params(), use_order_model=False)


def recorded() -> str:
    parts = []
    for name in SCENARIOS:
        world = play(name, OFF)
        assert not world.problems, world.problems[:3]
        parts.append(f"== {name}\n" + "\n".join(line.rstrip() for line in world.log))
    return "\n".join(parts) + "\n"


def test_the_order_model_is_on_by_default():
    assert load_params().use_order_model is True
    assert play("blocked_express").params.use_order_model is True


def test_with_the_order_model_off_the_log_is_the_same_as_before():
    text = recorded()
    if os.environ.get("ZONE_GOLDEN_WRITE") == "1":
        GOLDEN.parent.mkdir(exist_ok=True)
        GOLDEN.write_text(text, encoding="utf-8", newline="\n")
        pytest.skip("golden log rewritten")
    assert text == GOLDEN.read_text(encoding="utf-8")
