#!/usr/bin/env bash
# Sweep the wheel velocity-drive damping (kd) and effort limit, ramped and step commands, 4 envs each.
# Usage: bash learning/sim_check/sweep_wheel_gain.sh <out_dir>
set -u
OUT=${1:?out dir}
HERE=$(cd "$(dirname "$0")" && pwd)
PY=/d/isaac/IsaacLab/.venv/Scripts/python.exe
export PYTHONUTF8=1 OMNI_KIT_ACCEPT_EULA=YES PXR_WORK_THREAD_LIMIT=1
mkdir -p "$OUT" && OUT=$(cd "$OUT" && pwd) || exit 1
cd /d/isaac/IsaacLab || exit 1
for kd in 0.02 0.05 0.1 0.2 0.5; do
  for eff in 0.2 0.5; do
    for mode in step ramp; do
      if [ "$mode" = ramp ]; then acc=1.0; alp=3.0; else acc=0; alp=0; fi
      tag="kd${kd}_eff${eff}_${mode}"
      "$PY" "$HERE/pinky_newton_drive.py" --num_envs 4 --out "$OUT" --tag "$tag" --wheel_damping "$kd" \
        --wheel_effort "$eff" --accel "$acc" --alpha "$alp" --viz none > "$OUT/${tag}_stdout.log" 2>&1
      echo "$tag exit=$?" >> "$OUT/sweep_status.txt"
      "$PY" "$HERE/analyze_drive.py" "$OUT/${tag}_log.csv" "$OUT" > "$OUT/${tag}_analysis.md" 2>&1
    done
  done
done
