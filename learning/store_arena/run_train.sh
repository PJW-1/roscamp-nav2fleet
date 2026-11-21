#!/usr/bin/env bash
# Train the store-arena policy in chunks under the shared heavy-run lock, with a GPU memory log.
# Usage: bash learning/store_arena/run_train.sh <run_dir> <chunks> <iters_per_chunk> [num_envs] [train.py args...]
# e.g.   bash learning/store_arena/run_train.sh /d/isaac/runs/store_arena/ppo_a 4 250 4096
#        INIT=<4a run>/c1/model_999.pt bash learning/store_arena/run_train.sh <run> 1 1000 4096
# Chunk c goes to <run_dir>/c<c> and resumes from the last model of chunk c-1. The lock
# (/c/Users/pjw/ros2camp/.heavy_run.lock, also used by the Gazebo track) is taken per chunk and
# released after it; the script then waits PAUSE_S (180 s, the Gazebo track waits the same) so a
# waiting track can take it. Keep a chunk to about 30 minutes. Lock retries every 60 s.
# Uses the Isaac Lab venv python directly (no `uv run`: see learning/sim_check/README.md).
set -u
RUN=${1:?run dir}
CHUNKS=${2:?chunks}
ITERS=${3:?iterations per chunk}
N=${4:-4096}
shift $(( $# < 4 ? $# : 4 ))
HERE=$(cd "$(dirname "$0")" && pwd)
PY=/d/isaac/IsaacLab/.venv/Scripts/python.exe
LOCK=/c/Users/pjw/ros2camp/.heavy_run.lock
PAUSE_S=${PAUSE_S:-180}
export PYTHONUTF8=1 OMNI_KIT_ACCEPT_EULA=YES PXR_WORK_THREAD_LIMIT=1
mkdir -p "$RUN" && RUN=$(cd "$RUN" && pwd) || exit 1
cd /d/isaac/IsaacLab || exit 1
resume="${INIT:-}"  # INIT=<model_*.pt> starts chunk 1 from a checkpoint (curriculum)
for c in $(seq 1 "$CHUNKS"); do
  OUT=$RUN/c$c
  mkdir -p "$OUT"
  until mkdir "$LOCK" 2>/dev/null; do
    echo "chunk $c: lock held by $(cat "$LOCK/owner" 2>/dev/null); retry in 60 s"
    sleep 60
  done
  echo "store-arena-rl train $RUN c$c $(date -Iseconds)" > "$LOCK/owner"
  trap 'rm -rf "$LOCK"' EXIT
  nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader -lms 1000 > "$OUT/gpu.csv" &
  SMI=$!
  start=$(date +%s)
  "$PY" "$HERE/train.py" --num_envs "$N" --max_iterations "$ITERS" --out "$OUT" --viz none \
    ${resume:+--resume "$resume"} "$@" > "$OUT/stdout.log" 2>&1
  rc=$?
  kill $SMI
  trap - EXIT  # before the release: never remove a lock another track took after it
  rm -rf "$LOCK"
  echo "chunk $c exit=$rc wall_total_s=$(( $(date +%s) - start ))" | tee "$OUT/run_status.txt"
  [ $rc -eq 0 ] || exit $rc
  resume=$(ls -1v "$OUT"/model_*.pt | tail -1)
  [ "$c" -lt "$CHUNKS" ] && sleep "$PAUSE_S"
done
exit 0
