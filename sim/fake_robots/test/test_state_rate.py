"""가짜 로봇의 상태 발행 주기는 실물 pode_fsm 기본값과 같다."""
import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]


def _default(path: Path, call: str) -> float:
    text = path.read_text(encoding="utf-8")
    m = re.search(call + r'\("state_rate_hz",\s*([0-9.]+)\)', text)
    assert m, path
    return float(m.group(1))


def test_fake_pode_publishes_state_at_the_pode_fsm_rate():
    fake = _default(REPO / "sim/fake_robots/fake_robots/fake_pode_node.py", r"self\.declare_parameter")
    real = _default(REPO / "pode/pode_fsm/pode_fsm/node.py", r"declare")
    assert fake == real == 5.0
