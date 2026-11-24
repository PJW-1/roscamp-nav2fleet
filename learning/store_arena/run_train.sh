#!/usr/bin/env bash
# 매장 아레나 정책을 공유 무거운 작업 락 아래 청크로 학습하고 GPU 메모리를 기록한다.
# 사용: bash learning/store_arena/run_train.sh <run_dir> <chunks> <iters_per_chunk> [num_envs] [train.py 인자...]
# 청크 c 는 <run_dir>/c<c> 에 쓰고 c-1 의 마지막 모델에서 이어 가며, 락은 청크마다 잡고 놓은 뒤 PAUSE_S 쉰다.
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
resume="${INIT:-}"  # INIT=<model_*.pt> 면 청크 1 을 그 체크포인트에서 시작 (커리큘럼)
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
  trap - EXIT  # 해제 전에: 해제 후 다른 트랙이 잡은 락을 지우지 않도록
  rm -rf "$LOCK"
  echo "chunk $c exit=$rc wall_total_s=$(( $(date +%s) - start ))" | tee "$OUT/run_status.txt"
  [ $rc -eq 0 ] || exit $rc
  resume=$(ls -1v "$OUT"/model_*.pt | tail -1)
  [ "$c" -lt "$CHUNKS" ] && sleep "$PAUSE_S"
done
exit 0
