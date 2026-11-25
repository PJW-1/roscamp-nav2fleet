#!/usr/bin/env python3
"""py-spy raw(접힌 스택) 출력을 일한 곳별로 묶는다.

py-spy는 C 안에서 기다리는 스레드(wait set 대기, 스레드 풀 큐 대기)도 일하는 것으로 센다. 그 표본과
시작 때 import 표본은 뺀다. 남은 표본 수는 같은 길이의 기록끼리 CPU에 비례한다.
사용: spy_sum.py <raw파일> [상위 N]"""
import collections
import sys

WAITS = ("Executor._wait_for_ready_callbacks (rclpy/executors.py:781)",
         "_worker (concurrent/futures/thread.py:89)")


def group_of(frames):
    s = ";".join(frames)
    if "FrequentInputs._spin" in s:
        return "입력 노드 스레드(TF, odom)"
    if "Task.__call__" in s or "_WorkItem.run" in s:
        return "콜백(타이머, 구독, 서비스)"
    if "_wait_for_ready_callbacks" in s or "_spin_once_impl" in s:
        return "주 실행기: wait set 만들기와 고르기"
    return "기타"


def main():
    path = sys.argv[1]
    top = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    groups, leaf, total = collections.Counter(), collections.Counter(), 0
    dropped = collections.Counter()
    for line in open(path, encoding="utf-8"):
        stack, _, n = line.rstrip("\n").rpartition(" ")
        if not n.isdigit():
            continue
        n, frames = int(n), stack.split(";")
        if frames[-1] in WAITS:
            dropped["C 대기"] += n
            continue
        if any("_find_and_load" in f for f in frames):
            dropped["시작 import"] += n
            continue
        total += n
        groups[group_of(frames)] += n
        leaf[frames[-1]] += n
    print(f"일한 표본 {total} (뺀 것: {dict(dropped)})")
    print("\n== 묶음")
    for k, v in groups.most_common():
        print(f"{v / total * 100:6.1f} %  {k}")
    print("\n== 잎 함수")
    for k, v in leaf.most_common(top):
        print(f"{v / total * 100:6.1f} %  {k[:140]}")


if __name__ == "__main__":
    main()
