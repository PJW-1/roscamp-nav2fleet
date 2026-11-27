#!/usr/bin/env python3
"""관제 기록에서 구간 끝 정지(BASE_END_WAIT)를 끝 노드별로 센다.
분모: 그 노드에서 끝난 Segment 수(SEGMENT_DONE). 선 시간: 진행 피드백의 waiting_at_base_end가 참인 동안.
사용: python3 base_end_stops.py 기록.jsonl"""
import json, re, sys
from collections import Counter, defaultdict

rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
t0 = next(r["t"] for r in rows if "t" in r)
done = Counter(); waits = []
fb = defaultdict(list)   # robot -> [(t, waiting)]
ev = [r for r in rows if r.get("out") == "EventOut"]
for r in rows:
    if r.get("in") == "on_move_feedback":
        fb[r["args"][0]].append((r["t"], bool(r["args"][3])))
for e in ev:
    if e["code"] == "SEGMENT_DONE":
        m = re.search(r"\((\w+)\) to (\S+)$", e["detail"])
        done[m.group(2)] += 1
    elif e["code"] == "BASE_END_WAIT":
        node = re.match(r"stands at ([^,]+),", e["detail"]).group(1)
        f = [(t, w) for t, w in fb[e["robot"]] if t >= e["t"] - 1.0]
        start = next((t for t, w in f if w), None)
        end = next((t for t, w in f if start and t > start and not w), None)
        nxt = [x for x in ev if x["t"] > e["t"] and x.get("robot") == e["robot"]][:3]
        waits.append((node, e["robot"], e["t"] - t0, (end - start) if start and end else None,
                      [(round(x["t"] - t0, 1), x["code"], x["detail"][:70]) for x in nxt]))
print("node: stood / segments ending there")
for node in sorted(set(done) | {w[0] for w in waits}):
    print(f"  {node}: {sum(w[0] == node for w in waits)} / {done[node]}")
print(f"  total: {len(waits)} / {sum(done.values())}")
for node, robot, t, dur, nxt in waits:
    print(f"wait {robot} at {node} t=+{t:.1f} stood={dur if dur is None else round(dur, 1)} s; next: {nxt}")
