#!/usr/bin/env bash
# 작업 4 Gazebo 비교(pd-plugin2 item8_run.sh를 줄임). WSL에서: bash gz_run.sh [OUT]
# RL+shim 4b(store_arena_4b) 대 RL+shim a+c+d(store_arena_acd2), Q2 -> E-3과 c_moving, 칸마다 20판, GUI 없음.
# 잠금은 run_bench.sh가 시나리오마다 잡고 놓는다(사이 180 s). 끝나면 표와 이른 넘김 수를 만든다.
SRC=${SRC:-$HOME/nf_src/pd-acd}
B=${B:-$HOME/gg_build/pd-acd}
OUT=${1:-$HOME/rl_gz_check/acd2_$(date +%m%d_%H%M%S)}
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_deps_ws/install/setup.bash"
source "$HOME/pinky_deps_ws/install/local_setup.bash"
export CMAKE_PREFIX_PATH=$HOME/ort_probe/x/opt/ros/jazzy:${CMAKE_PREFIX_PATH:-}
export AMENT_PREFIX_PATH=$HOME/ort_probe/x/opt/ros/jazzy:${AMENT_PREFIX_PATH}
source "$B/install/setup.bash"
set -u
for p in store_arena_4b store_arena_acd2; do
  f=$B/install/pode_rl_controller/share/pode_rl_controller/policies/$p/policy.onnx
  [ -f "$f" ] || { echo "missing $f"; exit 2; }
done
HERE=$SRC/docs/rl_acd_2026-11-24
BENCH=$SRC/tools/nav_bench
mkdir -p "$OUT"
export BENCH_DOMAIN=${BENCH_DOMAIN:-61} BENCH_OVERRIDES=$HERE/gz_overrides.yaml
export BENCH_CONTROLLERS="FollowPathRLShim FollowPathRLShimACD2"
echo "acd2 gz start $(date -Is) out $OUT" | tee -a "$OUT/gz.txt"

CHILD=
marked() { for p in /proc/[0-9]*; do { tr '\0' '\n' < "$p/environ"; } 2>/dev/null | grep -qx "BENCH_RUN=$1" && echo "${p#/proc/}"; done; }
stop_child() {
  [ -n "$CHILD" ] || return
  kill -TERM "$CHILD" 2>/dev/null
  rest=$(marked "$CHILD"); [ -n "$rest" ] && kill -TERM $rest 2>/dev/null
  wait "$CHILD"
}
trap 'stop_child; exit 130' INT TERM
bench() { bash "$BENCH/run_bench.sh" "$@" & CHILD=$!; wait "$CHILD"; local r=$?; CHILD=; return $r; }

rc=0
BENCH_CONFIG=$SRC/results/2026-11-22/compare_012/q2e3.yaml BENCH_REPEATS=20 \
  bench "$OUT/q2e3" a_hall || rc=1
sleep 180
BENCH_CONFIG=$BENCH/scenarios.yaml BENCH_REPEATS=20 \
  bench "$OUT/main" c_moving || rc=1
echo "gazebo done $(date -Is) rc $rc" | tee -a "$OUT/gz.txt"

python3 "$HERE/gz_results.py" "$OUT" "$OUT" > "$OUT/gz_results.out" 2>&1 || rc=1
echo "acd2 gz end $(date -Is) rc $rc" | tee -a "$OUT/gz.txt"
exit $rc
