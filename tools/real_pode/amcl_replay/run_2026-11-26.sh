#!/usr/bin/env bash
# 11-26 pode2 bag(1126_real) 19:01:20~19:14:00 구간으로 AMCL을 한 도메인에서 N번 차례로 돌린다.
# 쓰기: run_2026-11-26.sh FILTERED_BAG OUT_DIR LABEL [N] [DOMAIN]
# bag 시작부터 P-1 초기 자세(19:01:13 launch 뒤 로봇은 120 s까지 서 있음), 19:14:05 손으로 옮기기 전까지.
# 잴 시각: Q2_1 19:03:43, O_IN 19:07:30, Q2_2 19:13:30. 정답은 truth_2026-11-26.json(라이다 가운데)
# 결과 표: amcl_replay.py score --sensor-x -0.017
set -euo pipefail
BAG=$1 OUT=$2 LABEL=$3 N=${4:-5} D=${5:-73}
HERE=$(cd "$(dirname "$0")" && pwd)
MAP=$HERE/../maps/arena_v7_fine.yaml  # 키트 기본 AMCL 지도(main 5b0b0e3부터)
TIMES=1795687423.000,1795687650.000,1795688010.000
mkdir -p "$OUT"
fail=0
for i in $(seq "$N"); do
  bash "$HERE/run_one.sh" "$D" "$BAG" "$MAP" "$LABEL" 0 759 0.120 0.100 1.5708 \
    "$TIMES" "$OUT/${LABEL}_$i.json" || fail=$((fail+1))
done
echo "[run] $LABEL 끝: 실패 $fail / $N"
