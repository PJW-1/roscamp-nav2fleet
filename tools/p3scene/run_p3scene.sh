#!/bin/bash
# P-3 끼임 장면 반복 시험: 설정마다 Gazebo 한 판을 띄우고 run_trials.py로 같은 pode3 자세 다섯 판을 돈다.
#   OUT=<결과 폴더> WS=<작업공간(install 있음)> run_p3scene.sh C0 C1 C2 C3
# pode1 자리 둘(SETS): A = P-1, B = 북쪽 길 위(NavFn이 틈으로 가게). 자세 다섯씩.
# 설정: C0 그대로, C1 peer 반지름 0.075, C2 local footprint_padding 0.0, C3 = C1 + C2
# 도메인 61, LOCALHOST, gui:=true(DISPLAY :0). 끝낼 때 이 판의 프로세스 묶음만 죽인다.
set -u
WS=${WS:?WS=작업공간}
OUT=$(realpath -m "${OUT:?OUT=결과 폴더}")
HERE=$(cd "$(dirname "$0")" && pwd)
TIMEOUT_S=${TIMEOUT_S:-240}
SETS=${SETS:-A,B}
PAD0=$WS/sim/fleet_gz_sim/nav_overrides/local_padding_0.yaml
mkdir -p "$OUT"

args_of() {
  case $1 in
    C0) echo "" ;;
    C1) echo "peer_radius_m:=0.075" ;;
    C2) echo "nav_overrides:=$PAD0" ;;
    C3) echo "peer_radius_m:=0.075 nav_overrides:=$PAD0" ;;
    *) echo "BAD" ;;
  esac
}

run() {
  local cfg=$1 extra
  extra=$(args_of "$cfg")
  [ "$extra" = BAD ] && { echo "unknown config $cfg"; return 2; }
  local dir=$OUT/$cfg
  [ -e "$dir/trials.jsonl" ] && { echo "$dir/trials.jsonl exists: use a new OUT"; return 2; }
  mkdir -p "$dir"
  (
  set +u
  source /opt/ros/jazzy/setup.bash; source /home/ane/pinky_deps_ws/install/setup.bash
  source /home/ane/ros2camp/nav2fleet_ws/install/setup.bash
  source "$WS/install/setup.bash"
  export ROS_DOMAIN_ID=61 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0
  cd "$dir"
  echo "== $cfg start $(date +%T) args: $extra"
  # shellcheck disable=SC2086
  setsid timeout -s INT 3000 ros2 launch fleet_gz_sim gz.launch.py layout:=v7 \
    scenario:="$HERE/p3_scene.yaml" monitor:=false fleet_check:=false sim_dock_stub:=true \
    rviz:=fleet gui:=true result_dir:="$dir/result" $extra > "$dir/launch.log" 2>&1 &
  local pid=$!
  trap 'kill -INT -- -$pid 2>/dev/null; sleep 5; pkill -KILL -g $pid 2>/dev/null; exit 130' INT TERM
  python3 "$HERE/run_trials.py" --config "$cfg" --out "$dir/trials.jsonl" \
    --timeout "$TIMEOUT_S" --sets "$SETS" 2>&1 | tee "$dir/trials.log"
  local rc=${PIPESTATUS[0]}
  # 실제로 걸린 값(판 뒤에 남긴다)
  timeout 20 ros2 param get /pode2/local_costmap/local_costmap footprint_padding > "$dir/params.txt" 2>&1
  grep -m3 "bodies of" "$dir/launch.log" >> "$dir/params.txt"
  kill -INT -- -$pid 2>/dev/null
  for _ in $(seq 30); do kill -0 $pid 2>/dev/null || break; sleep 1; done
  pkill -KILL -g $pid 2>/dev/null  # 이 판 묶음만
  echo "== $cfg end $(date +%T) rc=$rc"
  exit "$rc"
  )
  local status=$?
  sleep 10
  return $status
}

fail=0
for cfg in "$@"; do run "$cfg" || fail=1; done
exit $fail
