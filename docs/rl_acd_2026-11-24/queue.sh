#!/usr/bin/env bash
# 작업 4 작업 대기열: ppo_k에서 a+c+d 학습(시드 1-3)과 item 5/6 평가를 차례로 돌린다.
# Isaac 작업마다 .heavy_run.lock을 잡고 놓는다(사이 PAUSE_S). .heavy_run.queue/에 파일이 있으면 기다린다.
# 사용: bash docs/rl_acd_2026-11-24/queue.sh <작업 목록 파일>
# 한 줄에 작업 하나: tag|kind|args. kind: train, resume:<tag>(그 tag 마지막 모델에서 이어 학습),
# eval:<tag>(그 tag 마지막 모델로 play.py), eval(args에 --checkpoint). run.json·eval.json이 있으면 건너뛴다.
set -u
LIST=${1:?job list}
HERE=$(cd "$(dirname "$0")/../.." && pwd)/learning/store_arena
PY=/d/isaac/IsaacLab/.venv/Scripts/python.exe
LOCK=/c/Users/pjw/ros2camp/.heavy_run.lock
QDIR=/c/Users/pjw/ros2camp/.heavy_run.queue  # 파일이 있으면 다른 트랙 차례: 잠금을 잡지 않음
R=${R:-/d/isaac/runs/store_arena/acd24}
PAUSE_S=${PAUSE_S:-90}
export PYTHONUTF8=1 OMNI_KIT_ACCEPT_EULA=YES PXR_WORK_THREAD_LIMIT=1
mkdir -p "$R"
cd /d/isaac/IsaacLab || exit 1
release() { [ "$(cut -d' ' -f1-2 "$LOCK/owner" 2>/dev/null)" = "rl-acd $1" ] && rm -rf "$LOCK"; }
last_model() { ls -1v "$R/$1"/model_*.pt | tail -1; }
first=1
while IFS='|' read -r tag kind rest; do
  [ -z "$tag" ] && continue
  case "$tag" in \#*) continue ;; esac
  OUT=$R/$tag
  if [ -f "$OUT/run.json" ] || [ -f "$OUT/eval.json" ]; then echo "$tag skip (done)"; continue; fi
  case "$kind" in
    train) script=train.py; extra="" ;;
    resume:*) script=train.py; extra="--resume $(last_model "${kind#resume:}")" ;;
    eval:*) script=play.py; extra="--checkpoint $(last_model "${kind#eval:}")" ;;
    eval) script=play.py; extra="" ;;
    *) echo "$tag: unknown kind $kind"; exit 2 ;;
  esac
  [ $first -eq 1 ] || sleep "$PAUSE_S"
  first=0
  until [ -z "$(ls -A "$QDIR" 2>/dev/null)" ] && mkdir "$LOCK" 2>/dev/null; do
    echo "$tag: waiting (queue: $(ls -A "$QDIR" 2>/dev/null | tr '\n' ' '); lock: $(cat "$LOCK/owner" 2>/dev/null)); retry in 30 s"
    sleep 30
  done
  echo "rl-acd $tag $(date -Iseconds) (track 4 overnight RL, pd-acd)" > "$LOCK/owner"
  trap "release $tag" EXIT
  mkdir -p "$OUT"
  start=$(date +%s)
  # shellcheck disable=SC2086
  "$PY" "$HERE/$script" --out "$OUT" --viz none $extra $rest < /dev/null > "$OUT/stdout.log" 2>&1
  rc=$?
  trap - EXIT
  release "$tag"
  echo "$tag exit=$rc wall_s=$(( $(date +%s) - start )) $(date -Iseconds)" | tee -a "$R/queue_status.txt"
  [ $rc -eq 0 ] || { echo "$tag failed; stopping the queue"; exit $rc; }
done < "$LIST"
exit 0
