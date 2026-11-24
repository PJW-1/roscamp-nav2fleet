import json
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "nav_bench"))
import debug_extract as de  # noqa: E402
import metrics as mt  # noqa: E402

EMPTY = {"log": [], "costmap": [], "odom": []}


def count(legs_dir, only=None):
    body = mt.footprint()
    cells = defaultdict(lambda: {"legs": 0, "rotations": 0, "early": 0, "minutes": 0.0})
    for p in sorted(Path(legs_dir).glob("*.json")):
        rec = json.loads(p.read_text())
        if only and rec["controller"] not in only:
            continue
        rows, _ = de.leg_events(rec, EMPTY, body)
        c = cells[(rec["scenario"], rec["controller"])]
        c["legs"] += 1
        c["rotations"] += sum(1 for r in rows if not r["goal_turn"])
        c["early"] += sum(1 for r in rows if r["early"])
        c["minutes"] += (rec["t_end"] - rec["t_start"]) / 60.0
    return dict(cells)


def main():
    legs_dir = sys.argv[1]
    only = sys.argv[2:] or None
    print("| 시나리오 | 컨트롤러 | 판 | 출발·재계획 회전 | 이른 넘김 | 이른 넘김 /판 |")
    print("|---|---|---|---|---|---|")
    for (scenario, ctl), c in sorted(count(legs_dir, only).items()):
        print(f"| {scenario} | {ctl} | {c['legs']} | {c['rotations']} | {c['early']} | "
              f"{c['early'] / c['legs']:.1f} |")


if __name__ == "__main__":
    main()
