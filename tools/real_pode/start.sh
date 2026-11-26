#!/usr/bin/env bash
# bringup + Nav2 + pode_fsm을 뒤에서 띄운다:  start.sh pode1 0.12 0.10 1.5708 [0.12]
# (namespace, 지도 위 로봇 자리 x y yaw, 속도 m/s 기본 0.12). 기록: logs/launch_*.log
# 두 대 이상이면 PEER_ROBOTS=pode1,pode2 start.sh ...  (peer 층을 켠다. 안 주면 꺼짐)
# 도킹 대역을 PC에서 돌리면 DOCK_STUB=1 start.sh ...  (pode_fsm Dock 서버를 /<ns>/dock_fsm으로 비킨다)
# 고정 경로 충돌 검사는 기본 꺼짐(시뮬과 같다). 켜 두려면 FIXED_LINE_CHECK=1 start.sh ...
# Nav2 goal 길 유지 노드(해결안 A)는 기본 켬. 끄려면 PLAN_KEEP=0 start.sh ...
# 동작 없음(설계 4.5)은 NO_MOTION=1 start.sh pode2 [X Y YAW]: pode_fsm만 띄우고 주행하지 않는다.
#   자세를 안 주면 로봇별 주차 자리(v7 P-1~P-3)를 쓴다. stop.sh로 내린다.
cd "$(dirname "$(readlink -f "$0")")" || exit 1
# NO_MOTION은 1이면 켜고, 비었거나 0이면 끈다. 그 밖의 값은 거절한다
# TODO(no-motion): 실주행으로 돌아가면 이 분기와 아래 기본 자리 표를 지운다
nomo=${NO_MOTION:-0}
case $nomo in
  0) nomo= ;;
  1) ;;
  *) echo "bad NO_MOTION: use 1 (or unset/0)"; exit 2 ;;
esac
if [ -n "$nomo" ]; then
  [ $# -eq 1 ] || [ $# -eq 4 ] || { echo "usage: NO_MOTION=1 start.sh NAMESPACE [X Y YAW]"; exit 2; }
  # 실주행 선택은 함께 받지 않는다(PEER_ROBOTS는 비어야, 나머지는 비었거나 0이어야 한다)
  [ -z "${PEER_ROBOTS:-}" ] || { echo "NO_MOTION=1 does not take PEER_ROBOTS (no Nav2, no wheels)"; exit 2; }
  for v in DOCK_STUB FIXED_LINE_CHECK PLAN_KEEP; do
    case ${!v:-} in ''|0) ;; *) echo "NO_MOTION=1 does not take $v (no Nav2, no wheels)"; exit 2 ;; esac
  done
  if [ $# -eq 1 ]; then
    # 로봇별 기본 시작 자리: fleet_map_v7 주차 P-1~P-3 (x y yaw)
    case $1 in
      pode1) set -- pode1 0.12 0.10 1.5708 ;;
      pode2) set -- pode2 0.38 0.10 1.5708 ;;
      pode3) set -- pode3 0.64 0.10 1.5708 ;;
      *) echo "no default start pose for $1; give X Y YAW"; exit 2 ;;
    esac
  fi
  # 자세는 숫자만 받는다(launch가 뜬 뒤에 실패하지 않게)
  num='^[-+]?([0-9]+\.?[0-9]*|\.[0-9]+)$'
  for v in "$2" "$3" "$4"; do
    [[ $v =~ $num ]] || { echo "bad pose value '$v': use numbers like 0.12"; exit 2; }
  done
else
  [ $# -eq 4 ] || [ $# -eq 5 ] || { echo "usage: start.sh NAMESPACE X Y YAW [SPEED]"; exit 2; }
fi
speed=${5:-0.12}
# 시작 전에 이름 목록을 검사한다
peers=${PEER_ROBOTS:-}
if [ -n "$peers" ] && ! [[ $peers =~ ^[a-z_][a-z0-9_]*(,[a-z_][a-z0-9_]*)*$ ]]; then
  echo "bad PEER_ROBOTS: use names like pode1,pode2"; exit 2
fi
# DOCK_STUB는 1이면 켜고, 비었거나 0이면 끈다. 그 밖의 값은 거절한다
dock=${DOCK_STUB:-0}
case $dock in
  0) dock= ;;
  1) ;;
  *) echo "bad DOCK_STUB: use 1 (or unset/0)"; exit 2 ;;
esac
# FIXED_LINE_CHECK는 1이면 검사를 켜 두고, 비었거나 0이면 끈다. 그 밖의 값은 거절한다
fixed=${FIXED_LINE_CHECK:-0}
case $fixed in
  0) fixed= ;;
  1) fixed=--fixed-line-check ;;
  *) echo "bad FIXED_LINE_CHECK: use 1 (or unset/0)"; exit 2 ;;
esac
# PLAN_KEEP는 0이면 끄고, 비었거나 1이면 켠다. 그 밖의 값은 거절한다
keep=${PLAN_KEEP:-1}
case $keep in
  0) keep=false ;;
  1) keep=true ;;
  *) echo "bad PLAN_KEEP: use 0 (or unset/1)"; exit 2 ;;
esac
mkdir -p logs || exit 1
# 동시에 두 번 시작하지 않게 잠근다(pid 파일이 써질 때까지 쥔다)
exec 9> logs/start.lock || exit 1
flock -n 9 || { echo "another start.sh is running; try again"; exit 1; }
if [ -s logs/launch.pid ] && pgrep -g "$(cat logs/launch.pid)" > /dev/null; then
  echo "already running (group $(cat logs/launch.pid)); run stop.sh first"; exit 1
fi
# 속도 사본을 먼저 만든다(틀린 속도면 아무것도 띄우지 않는다). 동작 없음은 Nav2가 없어 건너뛴다
[ -n "$nomo" ] || python3 make_params.py --speed "$speed" --out params $fixed || exit 1
source env.sh || exit 1
log=logs/launch_$(date +%H%M%S).log
rm -f logs/launch.pid
# launch에는 잠금 fd를 넘기지 않는다
setsid bash -c 'echo $$ > logs/launch.pid
  if [ -n "$7" ]; then
    exec ros2 launch pode_fsm pode_fsm.launch.py namespace:="$1" no_motion:=true \
      initial_x:="$2" initial_y:="$3" initial_yaw:="$4"
  fi
  extra=(); [ -z "$5" ] || extra=(peer_obstacles:=true peer_robots:="$5")
  [ -z "$6" ] || extra+=(dock_aside:=true)
  [ -z "$8" ] || extra+=(plan_keep:="$8")
  exec ros2 launch ./real_pode.launch.py \
  namespace:="$1" initial_x:="$2" initial_y:="$3" initial_yaw:="$4" "${extra[@]}"' \
  _ "$1" "$2" "$3" "$4" "$peers" "$dock" "$nomo" "$keep" \
  > "$log" 2>&1 < /dev/null 9>&- &
child=$!
for _ in $(seq 50); do [ -s logs/launch.pid ] && break; sleep 0.1; done
if [ ! -s logs/launch.pid ]; then
  # 늦게 뜨는 launch가 다음 start와 겹치지 않게 잠금을 풀기 전에 내린다
  kill -TERM -- "-$child" 2> /dev/null || kill -TERM "$child" 2> /dev/null
  # 약 2 s 동안 사라지기를 기다린다(KILL로 올리지 않는다). 좀비는 죽은 것으로 본다
  alive() { ps -A -o pgid=,pid=,stat= | awk -v g="$child" \
    '($1 == g || $2 == g) && $3 !~ /^Z/ { f = 1 } END { exit !f }'; }
  for _ in $(seq 20); do alive || break; sleep 0.1; done
  if alive; then
    echo "launch did not start and is still running after TERM (group/pid $child);" \
      "stop it by hand; see $PWD/$log"; exit 1
  fi
  echo "launch did not start, stopped it; see $PWD/$log"; exit 1
fi
if [ -n "$nomo" ]; then
  echo "started NO MOTION at ($2, $3, $4) (group $(cat logs/launch.pid)), log $PWD/$log"
else
  echo "started (speed $speed m/s, group $(cat logs/launch.pid)), log $PWD/$log"
fi
