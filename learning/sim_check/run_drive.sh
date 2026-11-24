#!/usr/bin/env bash
# Newton 주행 점검(스텝/램프 명령)을 돌리고 분석한다.
# 사용: bash learning/sim_check/run_drive.sh <out_dir> [num_envs]
set -u
OUT=${1:?out dir}
N=${2:-16}
HERE=$(cd "$(dirname "$0")" && pwd)
PY=/d/isaac/IsaacLab/.venv/Scripts/python.exe
export PYTHONUTF8=1 OMNI_KIT_ACCEPT_EULA=YES PXR_WORK_THREAD_LIMIT=1
mkdir -p "$OUT" && OUT=$(cd "$OUT" && pwd) || exit 1
cd /d/isaac/IsaacLab || exit 1
for spec in "step:0:0" "ramp:1.0:3.0"; do
  IFS=: read -r tag acc alp <<<"$spec"
  nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader -lms 500 > "$OUT/${tag}_gpu.csv" &
  SMI=$!
  start=$(date +%s)
  "$PY" "$HERE/pinky_newton_drive.py" --num_envs "$N" --out "$OUT" --tag "$tag" --accel "$acc" --alpha "$alp" \
    --viz none > "$OUT/${tag}_stdout.log" 2>&1
  echo "$tag exit=$? wall_total_s=$(( $(date +%s) - start ))" | tee -a "$OUT/run_status.txt"
  kill $SMI
  "$PY" "$HERE/analyze_drive.py" "$OUT/${tag}_log.csv" "$OUT" > "$OUT/${tag}_analysis.md" 2>&1
done
