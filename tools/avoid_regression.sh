#!/bin/bash
# 회피 회귀 시험(추가 지시서 3의 7-3): 주행 완료 기준(2-5)과 매장 상황(5절)을 한 명령으로 돌린다.
# 회피 담당의 변경(params/avoidance.yaml, 비전 노드)은 이 명령이 0으로 끝나야 들어온다.
#
#   source /opt/ros/jazzy/setup.bash && source install/setup.bash    # colcon build 한 뒤
#   export ROS_DOMAIN_ID=45 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # 6은 쓰지 않는다
#   tools/avoid_regression.sh                      # 세 단계 모두. 결과는 avoid_regression_out/
#   tools/avoid_regression.sh --out /tmp/reg --games 1 --laps 1
#   tools/avoid_regression.sh --skip-gazebo        # 1단계만(ROS 없이 돈다)
#
# 단계
#   1. 머리 없는 세계: 매장 상황 표(tools/situations.py --store --avoid all). 못 지킨 기대가 있으면 실패
#   2. Gazebo 한 대(D2): 고정 경로의 충돌 검사를 켠 채 구간 종류마다 LAPS번(drive_check)
#   3. Gazebo 세 대 경기 GAMES판(씨드 1..GAMES): 끝나지 않은 주문에 "주행" 원인이 없고,
#      몸이 keepout 구역에 들어간 깊이가 keepout_intrusion_max_m(0.02 m) 이하
# 판정은 tools/avoid_regression.py가 한다. 한 단계가 실패해도 나머지를 돌리고, 하나라도 실패면 1로 끝난다.
set -u
repo="$(cd "$(dirname "$0")/.." && pwd)"
out="$PWD/avoid_regression_out"
games=3; laps=3; seeds="1 2 3"; plan="stage2"; keepout_max="0.02"
headless=true; gazebo=true
drive_wall_s="${DRIVE_WALL_S:-2400}"; game_wall_s="${GAME_WALL_S:-3000}"

usage() { sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; }
while [ $# -gt 0 ]; do
  case "$1" in
    --out) out="$2"; shift 2 ;;
    --games) games="$2"; shift 2 ;;
    --laps) laps="$2"; shift 2 ;;
    --seeds) seeds="$2"; shift 2 ;;
    --plan) plan="$2"; shift 2 ;;
    --keepout-max) keepout_max="$2"; shift 2 ;;
    --skip-headless) headless=false; shift ;;
    --skip-gazebo) gazebo=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "모르는 인자: $1" >&2; usage >&2; exit 2 ;;
  esac
done
mkdir -p "$out" && out="$(cd "$out" && pwd)" || exit 2
failed=()

note() { echo "[avoid_regression] $*"; }

# launch 하나를 제 프로세스 그룹으로 띄운다. 그룹 번호는 $group.
start_launch() {  # <로그> <launch 파일> [인자...]
  local log="$1"; shift
  setsid ros2 launch fleet_gz_sim "$@" > "$log" 2>&1 &
  group=$!
}

# 이 스크립트가 띄운 그룹만 끝낸다(다른 사람의 Gazebo는 건드리지 않는다).
stop_launch() {
  [ -n "$group" ] || return 0
  kill -INT -- "-$group" 2>/dev/null
  for _ in $(seq 1 30); do pgrep -g "$group" > /dev/null || break; sleep 1; done
  kill -KILL -- "-$group" 2>/dev/null
  group=""
  sleep 2
}
group=""
trap 'note "중단: 띄운 launch를 끝낸다"; stop_launch; exit 130' INT TERM

headless_stage() {
  note "1/3 매장 상황(머리 없는 세계, 흉내 세 모드, 씨드 $seeds)"
  # shellcheck disable=SC2086
  if python3 "$repo/tools/situations.py" --store --avoid all --seeds $seeds \
      > "$out/store_situations.md" 2> "$out/store_situations.log"; then
    note "    통과: $out/store_situations.md"
  else
    failed+=("매장 상황")
    note "    실패: $out/store_situations.md 끝의 '못 지킨 기대'"
  fi
}

# Nav2가 모두 active가 될 때까지 기다린다(180 s, 끝의 sleep만큼 넘을 수 있다). planner가 아직
# 켜지는 중에 보낸 goal은 0.1 s 만에 NAV_ABORTED로 끝난다. ros2 service call은 서비스와 답을
# 한도 없이 기다리므로 한 번을 10 s와 남은 시간 가운데 짧은 쪽으로 끊는다
nav2_active() {
  local start=$SECONDS left
  while left=$(( 180 - (SECONDS - start) )); [ "$left" -gt 0 ]; do
    timeout "$(( left < 10 ? left : 10 ))" ros2 service call \
      /pinky1/lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2> /dev/null \
      | grep -q "success=True" && return 0
    sleep 2
  done
  return 1
}

drive_stage() {
  note "2/3 Gazebo 한 대, 구간 종류마다 ${laps}번($plan)"
  rm -f "$out/drive_check.json" "$out/drive_check.md" "$out/drive_check.log"
  start_launch "$out/drive_launch.log" one_pinky.launch.py layout:=d2 start:=P-1 gui:=false \
    rviz:=false fixed_line_collision_check:=true
  if ! nav2_active; then
    failed+=("한 대 주행 (Nav2가 180 s 안에 active가 되지 않았다)")
    note "    실패: Nav2가 180 s 안에 active가 되지 않았다. $out/drive_launch.log"
    stop_launch
    return
  fi
  timeout "$drive_wall_s" ros2 run fleet_gz_sim drive_check --layout d2 --plan "$plan" \
    --laps "$laps" --json "$out/drive_check.json" > "$out/drive_check.log" 2>&1
  stop_launch
  if [ -f "$out/drive_check.json" ] \
      && python3 "$repo/tools/avoid_regression.py" drive "$out/drive_check.json" \
           > "$out/drive_check.md"; then
    note "    통과: $out/drive_check.md"
  else
    failed+=("한 대 주행")
    note "    실패: $out/drive_check.md, $out/drive_check.log"
  fi
}

game_stage() {  # <씨드>
  local seed="$1" dir="$out/game$1"
  note "3/3 Gazebo 세 대 경기, 씨드 $seed"
  if pgrep -f "gz sim" > /dev/null; then
    failed+=("세 대 경기 씨드 $seed (gz sim이 남아 있다)")
    note "    실패: 앞 단계의 gz sim이 끝나지 않았다"
    return
  fi
  rm -rf "$dir"; mkdir -p "$dir"
  start_launch "$out/game$seed.log" gz.launch.py layout:=d2 gui:=false rviz:=none \
    sim_dock_stub:=true fleet_record:=true scenario_seed:="$seed" result_dir:="$dir"
  # 시나리오 모니터가 끝나면 launch가 스스로 끝난다. 벽시계 한도를 넘으면 멈추고 실패로 친다
  local over=true
  for _ in $(seq 1 $((game_wall_s / 5))); do
    pgrep -g "$group" > /dev/null || { over=false; break; }
    sleep 5
  done
  stop_launch
  if $over; then
    failed+=("세 대 경기 씨드 $seed (시간 한도 ${game_wall_s} s)")
    note "    실패: 시간 한도를 넘었다. $out/game$seed.log"
  elif python3 "$repo/tools/avoid_regression.py" game "$dir" --keepout-max "$keepout_max" \
      > "$out/game$seed.md"; then
    note "    통과: $out/game$seed.md"
  else
    failed+=("세 대 경기 씨드 $seed")
    note "    실패: $out/game$seed.md, $out/game$seed.log"
  fi
}

$headless && headless_stage
if $gazebo; then
  if ! ros2 pkg prefix fleet_gz_sim > /dev/null 2>&1; then
    echo "fleet_gz_sim이 보이지 않는다: colcon build 뒤 install/setup.bash를 source한다" >&2
    exit 2
  fi
  if [ "${ROS_DOMAIN_ID:-}" = "" ] || [ "${ROS_DOMAIN_ID}" = "6" ]; then
    echo "ROS_DOMAIN_ID를 6이 아닌 값으로 정한다(예: 45)" >&2
    exit 2
  fi
  if pgrep -f "gz sim" > /dev/null; then
    echo "gz sim이 이미 돌고 있다. 끝낸 뒤 다시 돌린다" >&2
    exit 2
  fi
  drive_stage
  for seed in $(seq 1 "$games"); do game_stage "$seed"; done
fi

if [ ${#failed[@]} -eq 0 ]; then
  note "모두 통과. 결과: $out"
  exit 0
fi
note "실패 ${#failed[@]}건: $(IFS=,; echo "${failed[*]}"). 결과: $out"
exit 1
