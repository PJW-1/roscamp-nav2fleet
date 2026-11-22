#!/usr/bin/env bash
# item 8 다섯 컨트롤러 비교(compare_012와 같은 조건). WSL에서: bash item8_run.sh [OUT]
# RPP, MPPI, RL+shim(store_arena_4b), RL+shim c(store_arena_c), RL+shim acd(store_arena_acd).
# Q2 -> E-3 칸마다 3판, 그다음 O_IN -> M_E 시나리오 넷(a_hall e_corridor d_robot c_moving) 칸마다 6판.
# 잠금은 run_bench.sh가 시나리오마다 잡고 놓는다(사이 180 s). 끝나면 표와 이른 넘김 수를 만든다.
SRC=${SRC:-$HOME/nf_src/pd-plugin2}
B=${B:-$HOME/gg_build/pd-plugin2}
OUT=${1:-$HOME/rl_gz_check/item8_$(date +%m%d_%H%M%S)}
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_deps_ws/install/setup.bash"
source "$HOME/pinky_deps_ws/install/local_setup.bash"
export CMAKE_PREFIX_PATH=$HOME/ort_probe/x/opt/ros/jazzy:${CMAKE_PREFIX_PATH:-}
export AMENT_PREFIX_PATH=$HOME/ort_probe/x/opt/ros/jazzy:${AMENT_PREFIX_PATH}
source "$B/install/setup.bash"
set -u
for p in store_arena_4b store_arena_c store_arena_acd; do
  f=$B/install/pode_rl_controller/share/pode_rl_controller/policies/$p/policy.onnx
  [ -f "$f" ] || { echo "missing $f"; exit 2; }
done
HERE=$SRC/docs/rl_shim_recipe_2026-11-22
BENCH=$SRC/tools/nav_bench
mkdir -p "$OUT"
export BENCH_DOMAIN=${BENCH_DOMAIN:-61} BENCH_OVERRIDES=$HERE/item8_overrides.yaml
export BENCH_CONTROLLERS="FollowPathGoal FollowPathMPPI FollowPathRLShim FollowPathRLShimC FollowPathRLShimACD"
SHIMS="FollowPathRLShim FollowPathRLShimC FollowPathRLShimACD"
echo "item8 start $(date -Is) out $OUT" | tee -a "$OUT/item8.txt"

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
BENCH_CONFIG=$SRC/results/2026-11-22/compare_012/q2e3.yaml BENCH_REPEATS=3 \
  bench "$OUT/q2e3" a_hall || rc=1
sleep 180
BENCH_CONFIG=$BENCH/scenarios.yaml BENCH_REPEATS=6 \
  bench "$OUT/main" a_hall e_corridor d_robot c_moving || rc=1
echo "gazebo done $(date -Is) rc $rc" | tee -a "$OUT/item8.txt"

python3 "$BENCH/report.py" --config "$SRC/results/2026-11-22/compare_012/q2e3.yaml" \
  --legs "$OUT/q2e3/legs" --out "$OUT/report_q2e3" > "$OUT/report_q2e3.out" 2>&1 || rc=1
python3 "$BENCH/report.py" --config "$BENCH/scenarios.yaml" --legs "$OUT/main/legs" \
  --out "$OUT/report_main" > "$OUT/report_main.out" 2>&1 || rc=1
python3 "$HERE/early_count.py" "$OUT/q2e3/legs" $SHIMS > "$OUT/early_q2e3.md" || rc=1
python3 "$HERE/early_count.py" "$OUT/main/legs" $SHIMS > "$OUT/early_main.md" || rc=1
echo "item8 end $(date -Is) rc $rc" | tee -a "$OUT/item8.txt"
exit $rc
