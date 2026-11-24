#!/usr/bin/env bash
# 주행 순서(시뮬 19 s, 램프 명령)의 처리량을 스텝별 기록 없이 잰다.
# 사용: bash learning/sim_check/run_bench.sh <out_dir> [env 수...]
set -u
OUT=${1:?out dir}
shift
COUNTS=${*:-16 256 1024 4096}
HERE=$(cd "$(dirname "$0")" && pwd)
PY=/d/isaac/IsaacLab/.venv/Scripts/python.exe
export PYTHONUTF8=1 OMNI_KIT_ACCEPT_EULA=YES PXR_WORK_THREAD_LIMIT=1
mkdir -p "$OUT" && OUT=$(cd "$OUT" && pwd) || exit 1
cd /d/isaac/IsaacLab || exit 1
for n in $COUNTS; do
  tag="bench_n${n}"
  nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader -lms 500 > "$OUT/${tag}_gpu.csv" &
  SMI=$!
  start=$(date +%s)
  "$PY" "$HERE/pinky_newton_drive.py" --num_envs "$n" --out "$OUT" --tag "$tag" --log_every 0 --accel 1.0 --alpha 3.0 \
    --viz none > "$OUT/${tag}_stdout.log" 2>&1
  echo "$tag exit=$? wall_total_s=$(( $(date +%s) - start ))" | tee -a "$OUT/bench_status.txt"
  kill $SMI
done
