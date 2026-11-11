"""order 추가지시서 6-2의 상황 표: order 모델을 끄고 켜서 구역 세계에서 돌린다.

    python3 tools/order_situations.py --seeds 1 2 3
"""
from __future__ import annotations

import argparse

from fake_robots.world.order_situations import (
    ORDER_SITUATIONS,
    order_markdown,
    run_order_situation,
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-s", "--situations", nargs="*", default=list(ORDER_SITUATIONS))
    parser.add_argument("--seeds", nargs="*", type=int, default=[1, 2, 3])
    parser.add_argument("-l", "--layout", default="d2")
    args = parser.parse_args(argv)
    rows, failed = [], 0
    for name in args.situations:
        for order_model in (False, True):
            for seed in args.seeds:
                outcome = run_order_situation(name, order_model, seed, args.layout)
                rows.append(outcome.row)
                failed += bool(outcome.failures)
    print(order_markdown(rows))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
