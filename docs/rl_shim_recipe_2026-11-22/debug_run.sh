#!/usr/bin/env bash
# RotationShim 조기 넘김 원인 확인 실행(1.5절). WSL에서: bash debug_run.sh [OUT]
# 한 Gazebo에서 RL+shim(compare_012의 FollowPathRLShim)으로 Q2 -> E-3 3판, O_IN -> M_E 2판.
# controller_server만 DEBUG, rosout·local costmap·odom·cmd_vel을 sim 시간으로 bag에 남긴다.
# 무거운 작업 잠금을 한 번 잡고, Gazebo가 끝나면 바로 놓는다. 남의 잠금은 지우지 않는다.
SRC=${SRC:-$HOME/nf_src/pd-plugin2}
B=${B:-$HOME/gg_build/pd-plugin2}
OUT=${1:-$HOME/rl_gz_check/shimdbg_$(date +%m%d_%H%M%S)}
LOCKD=/mnt/c/Users/pjw/ros2camp/.heavy_run.lock
HERE=$SRC/docs/rl_shim_recipe_2026-11-22
BENCH=$SRC/tools/nav_bench
Q2E3_REPEATS=${Q2E3_REPEATS:-3}
MAIN_REPEATS=${MAIN_REPEATS:-2}
CTL=FollowPathRLShim

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_deps_ws/install/setup.bash"
source "$HOME/pinky_deps_ws/install/local_setup.bash"
export CMAKE_PREFIX_PATH=$HOME/ort_probe/x/opt/ros/jazzy:${CMAKE_PREFIX_PATH:-}
export AMENT_PREFIX_PATH=$HOME/ort_probe/x/opt/ros/jazzy:${AMENT_PREFIX_PATH}
source "$B/install/setup.bash"
set -u
export ROS_DOMAIN_ID=${DBG_DOMAIN:-61} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp GALLIUM_DRIVER=${GALLIUM_DRIVER:-d3d12}
case $ROS_DOMAIN_ID in 61|62) ;; *) echo "domain $ROS_DOMAIN_ID not allowed"; exit 2 ;; esac
mkdir -p "$OUT"
export ROS_LOG_DIR=$OUT/roslog GZ_PARTITION=shimdbg_$$

HAVE_LOCK=0
mine() { for p in /proc/[0-9]*; do { tr '\0' '\n' < "$p/environ"; } 2>/dev/null | grep -qx "DBG_RUN=$$" && echo "${p#/proc/}"; done; }
OWNER="rl-plugin shim-debug $$"
stop_all() {
  if [ -n "${BAG:-}" ]; then
    kill -INT "$BAG" 2>/dev/null
    for i in $(seq 20); do kill -0 "$BAG" 2>/dev/null || break; sleep 1; done
    BAG=
  fi
  rest=$(mine); [ -n "$rest" ] && { kill -INT $rest 2>/dev/null; sleep 10; }
  rest=$(mine); [ -n "$rest" ] && { kill -9 $rest 2>/dev/null; sleep 1; }
  if [ "$HAVE_LOCK" = 1 ]; then
    if grep -q "^$OWNER " "$LOCKD/owner" 2>/dev/null; then
      rm -rf "$LOCKD"; echo "lock released $(date -Is)" | tee -a "$OUT/runs.txt"
    else
      echo "lock owner changed ($(cat "$LOCKD/owner" 2>/dev/null)), left alone" | tee -a "$OUT/runs.txt"
    fi
    HAVE_LOCK=0
  fi
}
trap stop_all EXIT
trap 'exit 130' INT TERM

for i in $(seq 240); do
  mkdir "$LOCKD" 2>/dev/null && { HAVE_LOCK=1; break; }
  [ "$i" = 1 ] && echo "lock held by $(cat "$LOCKD/owner" 2>/dev/null), waiting $(date +%T)"
  sleep 60
done
[ "$HAVE_LOCK" = 1 ] || { echo "no lock, stop"; exit 3; }
echo "$OWNER $(date -Is)" > "$LOCKD/owner"
avail=$(free -m | awk '/Mem:/{print $7}')
[ "$avail" -lt 3072 ] && { echo "only $avail MB free, stop"; exit 4; }
echo "start $(date -Is) avail ${avail}MB domain $ROS_DOMAIN_ID out $OUT" | tee -a "$OUT/runs.txt"

( cd "$OUT" && DBG_RUN=$$ exec ros2 launch fleet_gz_sim one_pode.launch.py gui:=false rviz:=false \
    fsm:=false start:=O_IN nav_overrides:="$HERE/debug_overrides.yaml" \
    nav_log_level:=pode1.controller_server:=debug > launch.out 2>&1 ) &
up=$!
ok=0
for i in $(seq 90); do
  kill -0 "$up" 2>/dev/null || break
  DBG_RUN=$$ timeout 10 ros2 lifecycle get /pode1/bt_navigator 2>/dev/null | grep -q "^active" && { ok=1; break; }
  sleep 2
done
[ $ok = 1 ] || { echo "Nav2 did not come up, see $OUT/launch.out" | tee -a "$OUT/runs.txt"; exit 5; }
sleep 5

DBG_RUN=$$ ros2 bag record --use-sim-time -o "$OUT/bag" --topics /rosout /pode1/local_costmap/costmap \
  /pode1/odom /pode1/cmd_vel /pode1/plan /pode1/sim/truth > "$OUT/bag.out" 2>&1 &
BAG=$!
sleep 5
kill -0 "$BAG" 2>/dev/null && [ -d "$OUT/bag" ] || { echo "bag recorder did not start, see $OUT/bag.out" | tee -a "$OUT/runs.txt"; exit 6; }

rc=0
DBG_RUN=$$ python3 "$BENCH/bench_node.py" --config "$SRC/results/2026-11-22/compare_012/q2e3.yaml" \
  --scenario a_hall --out "$OUT/legs_q2e3" --repeats "$Q2E3_REPEATS" --controllers $CTL \
  > "$OUT/bench_q2e3.out" 2>&1 || rc=1
echo "q2e3: $(tail -1 "$OUT/bench_q2e3.out")" | tee -a "$OUT/runs.txt"
DBG_RUN=$$ python3 "$BENCH/bench_node.py" --config "$BENCH/scenarios.yaml" \
  --scenario a_hall --out "$OUT/legs_main" --repeats "$MAIN_REPEATS" --controllers $CTL \
  > "$OUT/bench_main.out" 2>&1 || rc=1
echo "main: $(tail -1 "$OUT/bench_main.out")" | tee -a "$OUT/runs.txt"

stop_all
echo "gazebo done $(date -Is) rc $rc out $OUT" | tee -a "$OUT/runs.txt"
exit $rc
