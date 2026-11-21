#!/usr/bin/env bash
# Train the goal-reach policy headless on Newton and log GPU memory next to it.
# Usage: bash learning/goal_reach/run_train.sh <out_dir> [num_envs] [max_iterations] [extra train.py args...]
# e.g.   bash learning/goal_reach/run_train.sh /d/isaac/runs/goal_reach/base 4096 300
#        bash learning/goal_reach/run_train.sh /d/isaac/runs/goal_reach/wtime 4096 300 --set w_time=-0.1
# Uses the Isaac Lab venv python directly (no `uv run`: see learning/sim_check/README.md).
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
