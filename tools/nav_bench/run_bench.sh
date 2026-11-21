#!/usr/bin/env bash
# RPP와 MPPI 비교 묶음: 시나리오 하나마다 Gazebo 한 대를 띄우고 bench_node가 두 컨트롤러를 번갈아 돈다.
# 사용(WSL, ROS와 이 저장소 install을 source한 셸에서):
#   tools/nav_bench/run_bench.sh OUT_DIR a_hall b_static c_moving d_robot
# 환경 변수:
#   BENCH_DOMAIN   ROS_DOMAIN_ID(기본 61). 43~101만 받는다(isolation.py)
#   BENCH_LOCK     같이 쓰는 무거운 작업 잠금 폴더(기본 /mnt/c/Users/pjw/ros2camp/.heavy_run.lock). 비우면 잠그지 않는다
#   BENCH_REPEATS  시나리오마다 컨트롤러마다 판 수(기본은 scenarios.yaml의 repeats)
#   BENCH_OVERRIDES  nav_overrides yaml(비교용 값 덮기). 비면 없음
#   BENCH_LOCK_GAP_S 잠금을 놓은 뒤 다음 시나리오 전에 쉬는 초(180 이상만 받는다. 기다리는 다른 작업이 잡게)
# 시나리오마다 잠금을 잡고 놓는다(다른 작업이 오래 굶지 않게). 판마다의 기록은 OUT_DIR/legs/*.json이다.
# 어느 시나리오든 bench_node가 실패하거나 Nav2가 뜨지 않으면 끝에 1로 끝난다.
set -u
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
OUT=$1; shift
LOCKD=${BENCH_LOCK-/mnt/c/Users/pjw/ros2camp/.heavy_run.lock}
GAP=${BENCH_LOCK_GAP_S:-180}; [ "$GAP" -lt 180 ] && GAP=180
export ROS_DOMAIN_ID=${BENCH_DOMAIN:-61} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export GALLIUM_DRIVER=${GALLIUM_DRIVER:-d3d12}
mkdir -p "$OUT/legs"
HAVE_LOCK=0
FAILED=0
mine() { for p in /proc/[0-9]*; do { tr '\0' '\n' < "$p/environ"; } 2>/dev/null | grep -qx "BENCH_RUN=$$" && echo "${p#/proc/}"; done; }
unlock() { [ "$HAVE_LOCK" = 1 ] && rm -rf "$LOCKD"; HAVE_LOCK=0; }
stop_all() {
  rest=$(mine); [ -n "$rest" ] && { kill -INT $rest 2>/dev/null; sleep 10; }
  rest=$(mine); [ -n "$rest" ] && { kill -9 $rest 2>/dev/null; sleep 1; }
  unlock
}
trap 'stop_all' EXIT
trap 'exit 130' INT TERM
n=$#; k=0
for scenario in "$@"; do
  k=$((k + 1))
  if [ -n "$LOCKD" ]; then
    for i in $(seq 240); do
      mkdir "$LOCKD" 2>/dev/null && { HAVE_LOCK=1; break; }
      [ "$i" = 1 ] && echo "$scenario: lock held by $(cat "$LOCKD/owner" 2>/dev/null), waiting $(date +%T)"
      sleep 120
    done
    [ "$HAVE_LOCK" = 1 ] || { echo "$scenario: no lock, stop"; exit 3; }
    echo "nav_bench $$ $scenario $(date -Is)" > "$LOCKD/owner"
  fi
  avail=$(free -m | awk '/Mem:/{print $7}')
  if [ "$avail" -lt 3072 ]; then echo "$scenario: only $avail MB free, stop"; exit 4; fi
  R=$OUT/$scenario; mkdir -p "$R"
  echo "$scenario start $(date -Is) avail ${avail}MB domain $ROS_DOMAIN_ID" | tee -a "$OUT/runs.txt"
  ovr=""; [ -n "${BENCH_OVERRIDES:-}" ] && ovr="nav_overrides:=$BENCH_OVERRIDES"
  ( cd "$R" && BENCH_RUN=$$ GZ_PARTITION=nav_bench_$$_$scenario ros2 launch fleet_gz_sim one_pode.launch.py \
      gui:=false rviz:=false fsm:=false start:=O_IN $ovr > launch.out 2>&1 ) &
  ok=0
  for i in $(seq 90); do
    BENCH_RUN=$$ ros2 lifecycle get /pode1/bt_navigator 2>/dev/null | grep -q active && { ok=1; break; }; sleep 2
  done
  if [ $ok = 1 ]; then
    sleep 5
    BENCH_RUN=$$ GZ_PARTITION=nav_bench_$$_$scenario python3 "$HERE/bench_node.py" --config "$HERE/scenarios.yaml" \
      --scenario "$scenario" --out "$OUT/legs" ${BENCH_REPEATS:+--repeats $BENCH_REPEATS} > "$R/bench.out" 2>&1
    rc=$?
    [ $rc = 0 ] || FAILED=1
    echo "$scenario: exit $rc, $(tail -1 "$R/bench.out")" | tee -a "$OUT/runs.txt"
  else
    FAILED=1
    echo "$scenario: Nav2 did not come up" | tee -a "$OUT/runs.txt"
  fi
  stop_all
  echo "$scenario end $(date -Is)" | tee -a "$OUT/runs.txt"
  [ -n "$LOCKD" ] && [ "$k" -lt "$n" ] && sleep "$GAP"
done
exit $FAILED
