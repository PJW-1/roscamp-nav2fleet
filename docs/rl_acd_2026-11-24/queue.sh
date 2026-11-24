#!/usr/bin/env bash
# 작업 4 (2026-11-24): a+c+d from ppo_k, seeds 1-3 x 500 iterations (2 x 250), then the item 5/6 evals.
# Every Isaac job takes /c/Users/pjw/ros2camp/.heavy_run.lock and releases it after; PAUSE_S between jobs.
# Before taking it, the queue waits while /c/Users/pjw/ros2camp/.heavy_run.queue/ holds any file.
# Usage: bash docs/rl_acd_2026-11-24/queue.sh <job-list-file>
# One job per line: tag|kind|args. kind: train (args given), resume:<tag> (train from the last
# model of <tag>), eval:<tag> (play.py on the last model of <tag>), eval (play.py, args give
# --checkpoint). A job whose output already has run.json (train) or eval.json (eval) is skipped.
set -u
LIST=${1:?job list}
HERE=$(cd "$(dirname "$0")/../.." && pwd)/learning/store_arena
PY=/d/isaac/IsaacLab/.venv/Scripts/python.exe
LOCK=/c/Users/pjw/ros2camp/.heavy_run.lock
QDIR=/c/Users/pjw/ros2camp/.heavy_run.queue  # any file here: another track is next, do not take the lock
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
