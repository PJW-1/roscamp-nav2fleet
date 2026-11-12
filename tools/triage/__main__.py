import sys
from pathlib import Path

# `python tools/triage ...`로 부르면 패키지 밖에서 시작하므로 tools/를 길에 넣는다
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from triage.cli import main  # noqa: E402

sys.exit(main())
