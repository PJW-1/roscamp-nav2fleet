"""체크아웃에서 `python tools/<script>.py -h`가 경로 준비 없이 실행된다."""
import subprocess
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("script", ["situations.py", "order_situations.py"])
def test_script_help_runs_from_checkout(script):
    result = subprocess.run(
        [sys.executable, str(TOOLS / script), "-h"],
        capture_output=True, text=True, cwd=TOOLS.parent,
    )
    assert result.returncode == 0, result.stderr
