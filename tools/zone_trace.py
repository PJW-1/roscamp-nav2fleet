"""구역 교통 시험 판(zone_scenarios.SCENARIOS)을 모두 돌려 매 tick을 텍스트로 찍는다.

    python tools/zone_trace.py > trace.txt
    python tools/zone_trace.py -s serve_three --orders manager --models on

정리 전후의 출력을 diff로 비교하는 데 쓴다(ZoneTraffic 정리 지시서 0단계). 판마다 tick 순서
(zone, manager)와 order 모델(켬, 끔)을 바꿔 가며 돌린다. 한 tick에 쓰는 것:
    - `t <시각> out <항목>`: 그 tick에 나온 명령, 이벤트, 결과를 나온 순서대로 한 목록에
    - `t <시각> <로봇> held=... nodes=... view=... status=...`:
      보유 구역·노드, robot_view, status_view
판이 끝나지 않으면 그 사실을 적고 다음 판으로 간다(실패도 기준의 일부다).
"""
from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
for sub in ("pc/fleet_core", "pc/fleet_core/test"):
    if str(REPO / sub) not in sys.path:
        sys.path.insert(0, str(REPO / sub))

from fleet_core.common.params import load_params  # noqa: E402
from zone_harness import TICK_ORDERS, Harness  # noqa: E402
from zone_scenarios import SCENARIOS  # noqa: E402

MODELS = {"on": True, "off": False}


class TracedHarness(Harness):
    """하네스 그대로. 나온 항목과 매 tick 끝의 상태를 write로 넘긴다."""

    def __init__(self, write, *args, **kwargs):
        self._write = write
        self._items: list = []
        super().__init__(*args, **kwargs)

    def _apply(self, item) -> None:
        self._items.append(item)
        super()._apply(item)

    def step(self) -> None:
        super().step()
        stamp = f"t {self.now:.1f}"
        items, self._items = self._items, []
        for item in items:
            self._write(f"{stamp} out {item!r}")
        grants = self.traffic.grants
        for robot in self.bots:
            self._write(f"{stamp} {robot} held={list(grants.held_by(robot))} "
                        f"nodes={list(grants.nodes_of(robot))} "
                        f"view={self.traffic.robot_view(robot)!r} "
                        f"status={self.traffic.status_view(robot)!r}")


def trace(name: str, order: str, model: bool, write) -> None:
    scenario, starts = SCENARIOS[name]
    params = dataclasses.replace(load_params(), use_order_model=model)
    write(f"== {name} tick_order={order} use_order_model={model}")
    world = TracedHarness(write, starts=starts, params=params, tick_order=order)
    finished = scenario(world)
    write(f"== end {name}: finished={finished} problems={world.problems!r}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-s", "--scenarios", nargs="+", default=list(SCENARIOS),
                        choices=list(SCENARIOS))
    parser.add_argument("--orders", nargs="+", default=list(TICK_ORDERS), choices=TICK_ORDERS)
    parser.add_argument("--models", nargs="+", default=list(MODELS), choices=list(MODELS))
    args = parser.parse_args(argv)
    out = sys.stdout
    out.reconfigure(encoding="utf-8", newline="\n")
    write = lambda line: out.write(line + "\n")  # noqa: E731
    for name in args.scenarios:
        for order in args.orders:
            for model in args.models:
                trace(name, order, MODELS[model], write)
    return 0


if __name__ == "__main__":
    sys.exit(main())
