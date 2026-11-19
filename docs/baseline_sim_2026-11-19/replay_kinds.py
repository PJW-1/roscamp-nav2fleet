"""FMS 기록을 지금 코드로 재생해, 처음 달라지는 곳과 종류별 출력 수를 요약한다.

사용: python3 replay_kinds.py NAME=fleet_record.jsonl [NAME=fleet_record.jsonl ...]
tools/replay.py는 처음 다른 줄에서 멈춘다. 이 스크립트는 끝까지 재생하고 기록된 출력과 나온 출력을
종류(명령 이름, 이벤트 code)별로 세어 차이를 보여 준다. 판단이 바뀌는 수정을 볼 때 쓴다.
주의: 기록의 입력(로봇 상태, 결과)은 그때 나간 명령에 대한 것이다. 처음 달라진 뒤로는 새 코드의
명령과 맞지 않으므로, 그 뒤의 차이는 "실제로 이렇게 달라진다"가 아니라 "여기서부터 갈린다"로 읽는다.
"""
import importlib.util
import json
import sys
from collections import Counter
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("replay", REPO / "tools" / "replay.py")
replay = importlib.util.module_from_spec(spec)
sys.modules["replay"] = replay
spec.loader.exec_module(replay)


def kind(entry):
    if "out" in entry:
        return f'{entry["out"]}:{entry["code"]}' if entry["out"] == "EventOut" else entry["out"]
    return f'inside {entry["inside"]}' if "inside" in entry else None


def play(path):
    lines, entries, _ = replay._read(path)
    head, *rest = entries
    now = [0.0]
    core = replay.start_core(head, lambda: now[0])
    produced = []
    core.record_to(produced.append)
    error = ""
    for number, entry in enumerate(rest, start=2):
        if "unrecordable" in entry:  # tools/replay.py도 여기서 재생을 그만둔다
            error = f"line {number}: this input could not be recorded ({entry['unrecordable']})"
            break
        if "in" in entry:
            now[0] = entry["t"]
            try:
                getattr(core, entry["in"])(*replay._arguments(entry))
            except Exception as problem:  # noqa: BLE001 재생이 죽은 자리를 보고한다
                error = f"line {number} t {entry['t']}: {type(problem).__name__}: {problem}"
                break
    return lines, produced, error


def brief(line):
    entry = json.loads(line)
    what = kind(entry) or f'in {entry["in"]}'
    more = {k: entry[k] for k in ("robot", "order_id", "update_id", "stitch_node", "detail")
            if k in entry}
    if "base" in entry:
        more["base"] = [s["end_node"] for s in entry["base"]]
    return f't {entry["t"]} {what} {more}'


def main():
    for arg in sys.argv[1:]:
        name, path = arg.split("=", 1)
        lines, produced, error = play(path)
        first = next((i for i, (a, b) in enumerate(zip(lines[1:], produced[1:], strict=False))
                      if a != b), None)
        print(f"\n### {name}: {len(lines)} recorded lines, {len(produced)} produced"
              + (f", ERROR {error}" if error else ", no error"))
        if first is None and len(lines) == len(produced):
            print("same as recorded")
            continue
        if first is None:  # 한쪽이 다른 쪽의 앞부분과 같다
            first = min(len(lines), len(produced)) - 1
            print(f"same up to line {first + 1}, then only one side goes on")
        else:
            print(f"first difference at line {first + 2}")
        for name_, side in (("recorded", lines), ("produced", produced)):
            print(f"  {name_}: {brief(side[first + 1]) if first + 1 < len(side) else '(nothing)'}")
        after = [line for line in produced[first + 1:] if kind(json.loads(line))][:6]
        print("  what the code put out from there:")
        for line in after:
            print(f"    {brief(line)}")
        old = Counter(k for k in map(kind, map(json.loads, lines[1:])) if k)
        new = Counter(k for k in map(kind, map(json.loads, produced[1:])) if k)
        print("| kind | recorded | produced |")
        print("|---|---|---|")
        for key in sorted(set(old) | set(new)):
            if old[key] != new[key]:
                print(f"| {key} | {old[key]} | {new[key]} |")
        same = sum(1 for key in set(old) | set(new) if old[key] == new[key])
        print(f"({same} kinds have the same count)")


if __name__ == "__main__":
    main()
