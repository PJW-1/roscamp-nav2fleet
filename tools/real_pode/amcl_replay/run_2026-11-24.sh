#!/usr/bin/env bash
# 11-24 pode2 bag(1124_amcl_base_pode2)으로 지도별 AMCL을 N번씩 다시 돌린다.
# 쓰기: run_2026-11-24.sh FILTERED_BAG OUT_DIR [N] [FIRST_DOMAIN]
# 자리: Q2는 bag 시작부터(P-1 초기 자세, 300 s에 잼), O_IN은 1111 s의 initialpose부터(1200 s에 잼).
# 정답(줄자)은 truth_2026-11-24.json. 결과 표: amcl_replay.py score
set -euo pipefail
BAG=$1 OUT=$2 N=${3:-5} D=${4:-101}
HERE=$(cd "$(dirname "$0")" && pwd)
MAPS=$HERE/../maps
T0=1795523086.469  # bag 시작(초)
mkdir -p "$OUT"
declare -A MAP=([slam]=$MAPS/arena_real_slam.yaml [drawing]=$MAPS/arena_v7_fine.yaml)
pids=()
for m in slam drawing; do
  for i in $(seq "$N"); do
    bash "$HERE/run_one.sh" "$D" "$BAG" "${MAP[$m]}" "$m" 0 303 0.120 0.100 1.5708 \
      "$(python3 -c "print(f'{$T0+300:.3f}')")" "$OUT/${m}_Q2_$i.json" & pids+=($!); D=$((D+1))
    bash "$HERE/run_one.sh" "$D" "$BAG" "${MAP[$m]}" "$m" 1111 92 0.116 0.142 1.5655 \
      "$(python3 -c "print(f'{$T0+1200:.3f}')")" "$OUT/${m}_OIN_$i.json" & pids+=($!); D=$((D+1))
  done
done
fail=0
for p in "${pids[@]}"; do wait "$p" || fail=$((fail+1)); done
echo "[run] 끝: 실패 $fail / ${#pids[@]}"
