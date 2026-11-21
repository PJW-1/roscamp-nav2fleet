#!/usr/bin/env bash
# 한 묶음(8판)의 표: 판 통과, Route 시간, Q2 정지, 갱신 시점(step1_report), 실제 Q2 정지(q2_dwell),
# nav_goal Segment 남은 시간 오차(eta_error). 판 폴더 이름은 sim_batch.sh가 만든 것을 그대로 쓴다.
# 사용: report.sh BATCH_DIR [run ...]   (run을 안 주면 rush1~rush4 v7a~v7d)
# eta_error는 이 스크립트가 있는 checkout의 코드로 재생한다. 브랜치 checkout에서 돌린다.
S=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
BASE=$S/../baseline_sim_2026-11-19
B=$1; shift
RUNS=${*:-rush1 rush2 rush3 rush4 v7a v7b v7c v7d}
results=(); truths=()
for r in $RUNS; do
  [ -f "$B/$r/result/summary.json" ] || { echo "skip $r: no result/summary.json"; continue; }
  results+=("$r=$B/$r/result"); truths+=("$r=$B/$r/truth.csv")
done
echo "## step1_report ($B)"
python3 "$BASE/step1_report.py" "${results[@]}"
echo; echo "## q2_dwell"
python3 "$BASE/q2_dwell.py" "${truths[@]}"
echo; echo "## eta_error"
python3 "$S/eta_error.py" "${results[@]}" --per-run 2>&1 | grep -v '^fleet map '
