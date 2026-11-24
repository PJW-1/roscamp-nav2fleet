#!/usr/bin/env bash
# goal-reach 정책을 Newton 에서 headless 로 학습하고 GPU 메모리를 함께 기록한다.
# 사용: bash learning/goal_reach/run_train.sh <out_dir> [num_envs] [max_iterations] [train.py 추가 인자...]
set -u
OUT=${1:?out dir}
N=${2:-4096}
ITERS=${3:-300}
shift $(( $# < 3 ? $# : 3 ))
HERE=$(cd "$(dirname "$0")" && pwd)
PY=/d/isaac/IsaacLab/.venv/Scripts/python.exe
export PYTHONUTF8=1 OMNI_KIT_ACCEPT_EULA=YES PXR_WORK_THREAD_LIMIT=1
mkdir -p "$OUT" && OUT=$(cd "$OUT" && pwd) || exit 1
cd /d/isaac/IsaacLab || exit 1
nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader -lms 1000 > "$OUT/gpu.csv" &
SMI=$!
start=$(date +%s)
"$PY" "$HERE/train.py" --num_envs "$N" --max_iterations "$ITERS" --out "$OUT" --viz none "$@" > "$OUT/stdout.log" 2>&1
rc=$?
kill $SMI
echo "exit=$rc wall_total_s=$(( $(date +%s) - start ))" | tee "$OUT/run_status.txt"
exit $rc
