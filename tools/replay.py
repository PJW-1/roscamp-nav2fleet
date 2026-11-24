#!/usr/bin/env python3
"""FMS 기록(fleet_record.jsonl)을 ROS 없이 두 코어에 재생하고 결과를 비교한다.

    python3 tools/replay.py run.jsonl   # 종료 코드: 같으면 0, 다르면 1, 재생 불가 2
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, fields
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))

from fleet_core.common.commands import OrderRecord  # noqa: E402
from fleet_core.common.params import FleetParams  # noqa: E402
from fleet_core.manager import FleetManager  # noqa: E402
from fleet_core.traffic.modes import NoZoneTraffic  # noqa: E402
from fleet_core.traffic.site import site_from_text  # noqa: E402

# 기록에 없는 키는 옛 기본값으로 재생한다
RECORD_DEFAULTS = {"stall_s": 6.0, "use_order_model": False}


@dataclass(frozen=True)
class Result:
    inputs: int
    inside: int
    commands: int
    compared: bool  # 입력만 있는 기록이면 False
    same: bool
    line: int = 0  # 처음 다른 줄(1부터)
    recorded: str = ""
    produced: str = ""
    uncompared: int = 0  # 기록이 끝난 뒤에 나온 줄 수
    cut_line: bool = False  # 마지막 줄이 반쯤만 써져 빼 둠
    skipped: tuple = ()


def _count(lines: list[str], key: str) -> int:
    return sum(key in json.loads(line) for line in lines[1:])


def _arguments(entry: dict) -> list:
    if entry["in"] == "on_order_board":
        mode, orders = entry["args"]
        if isinstance(orders, list):  # 주문이 아니던 원소는 기록에 문자열로 남아 그대로 둔다
            orders = [OrderRecord(**o) if isinstance(o, dict) else o for o in orders]
        return [mode, orders]
    return entry["args"]


def _read(path) -> tuple[list[str], list[dict], bool]:
    """줄 목록, 내용, 끝의 반쯤 쓴 줄을 뺐는지 여부."""
    text = Path(path).read_text(encoding="utf-8")
    lines = text.splitlines()
    entries = []
    for number, line in enumerate(lines, start=1):
        try:
            entries.append(json.loads(line))
        except json.JSONDecodeError as error:
            if number == len(lines) and not text.endswith("\n"):
                return lines[:-1], entries, True
            raise ValueError(f"line {number} is not JSON: {error}") from error
    return lines, entries, False


def _known(recorded: dict) -> dict:
    names = {field.name for field in fields(FleetParams)}
    return {key: value for key, value in recorded.items() if key in names}


def _skipped(head: dict) -> tuple:
    return tuple(sorted(set(head["params"]) - set(_known(head["params"]))))


def start_core(head: dict, clock) -> FleetManager:
    missing = {"robots", "params", "map"} - set(head)
    if missing:
        raise ValueError(f"the first line has no {sorted(missing)}")
    try:
        params = FleetParams(**{**RECORD_DEFAULTS, **_known(head["params"])})
    except TypeError as error:
        raise ValueError(f"these parameters do not fit this code: {error}") from error
    site = site_from_text(head["map"])
    return FleetManager(site, params, head["robots"], clock)


def replay(path) -> Result:
    lines, entries, cut_line = _read(path)
    if not entries:
        raise ValueError("no first line, so this is not a record")
    head, *rest = entries
    if not any("in" in entry for entry in rest):
        raise ValueError("no inputs, so there is nothing to play back")
    now = [0.0]
    core = start_core(head, lambda: now[0])
    skipped = _skipped(head)
    produced: list[str] = []
    core.record_to(produced.append)
    for number, entry in enumerate(rest, start=2):
        if "unrecordable" in entry:
            raise ValueError(f"line {number}: this input could not be recorded "
                             f"({entry['unrecordable']})")
        if "in" in entry:
            now[0] = entry["t"]
            getattr(core, entry["in"])(*_arguments(entry))
    counts = tuple(_count(produced, key) for key in ("in", "inside", "out"))
    if not head.get("recorded") and not any("inside" in e or "out" in e for e in rest):
        return Result(*counts, False, True, cut_line=cut_line, skipped=skipped)
    # 첫 줄은 코어를 만든 입력이지 코어의 판단이 아니라 제외한다.
    for number, (old, new) in enumerate(zip(lines[1:], produced[1:], strict=False), start=2):
        if old != new:
            return Result(*counts, True, False, number, old, new, cut_line=cut_line,
                          skipped=skipped)
    if len(lines) > len(produced):
        return Result(*counts, True, False, len(produced) + 1, lines[len(produced)],
                      "(nothing came out)", cut_line=cut_line, skipped=skipped)
    return Result(*counts, True, True, uncompared=len(produced) - len(lines), cut_line=cut_line,
                  skipped=skipped)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("record", help="the JSONL file the fleet manager node wrote with --record")
    try:
        result = replay(parser.parse_args().record)
    except (ValueError, OSError, NoZoneTraffic) as error:
        print(f"cannot play this back: {error}")
        return 2
    if result.skipped:
        print(f"warning: skipped {len(result.skipped)} parameters this code no longer has: "
              f"{', '.join(result.skipped)}")
    print(f"{result.inputs} inputs, {result.inside} inside calls, {result.commands} commands")
    if result.cut_line:
        print("the last line was cut off and left out")
    if not result.compared:
        print("inputs only: nothing to compare with")
    elif result.same and result.uncompared:
        print(f"same as recorded, as far as it goes: {result.uncompared} lines past its end "
              "were not compared")
    elif result.same and result.cut_line:
        print("same as recorded, as far as it goes")
    elif result.same:
        print("same as recorded")
    else:
        print(f"line {result.line} differs\n recorded: {result.recorded}\n produced: "
              f"{result.produced}")
    return 0 if result.same else 1


if __name__ == "__main__":
    sys.exit(main())
