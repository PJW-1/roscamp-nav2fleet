#!/usr/bin/env bash
# bringup + Nav2 + pode_fsm을 뒤에서 띄운다:  start.sh pode1 0.12 0.10 1.5708 [0.12]
# (namespace, 지도 위 로봇 자리 x y yaw, 속도 m/s 기본 0.12). 기록: logs/launch_*.log
# 두 대 이상이면 PEER_ROBOTS=pode1,pode2 start.sh ...  (peer 층을 켠다. 안 주면 꺼짐)
# 도킹 대역을 PC에서 돌리면 DOCK_STUB=1 start.sh ...  (pode_fsm Dock 서버를 /<ns>/dock_fsm으로 비킨다)
# 고정 경로 충돌 검사는 기본 꺼짐(시뮬과 같다). 켜 두려면 FIXED_LINE_CHECK=1 start.sh ...
cd "$(dirname "$(readlink -f "$0")")" || exit 1
[ $# -eq 4 ] || [ $# -eq 5 ] || { echo "usage: start.sh NAMESPACE X Y YAW [SPEED]"; exit 2; }
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
mkdir -p logs || exit 1
# 동시에 두 번 시작하지 않게 잠근다(pid 파일이 써질 때까지 쥔다)
exec 9> logs/start.lock || exit 1
flock -n 9 || { echo "another start.sh is running; try again"; exit 1; }
if [ -s logs/launch.pid ] && pgrep -g "$(cat logs/launch.pid)" > /dev/null; then
  echo "already running (group $(cat logs/launch.pid)); run stop.sh first"; exit 1
fi
# 속도 사본을 먼저 만든다(틀린 속도면 아무것도 띄우지 않는다)
python3 make_params.py --speed "$speed" --out params $fixed || exit 1
source env.sh || exit 1
log=logs/launch_$(date +%H%M%S).log
rm -f logs/launch.pid
# launch에는 잠금 fd를 넘기지 않는다
setsid bash -c 'echo $$ > logs/launch.pid
  extra=(); [ -z "$5" ] || extra=(peer_obstacles:=true peer_robots:="$5")
  [ -z "$6" ] || extra+=(dock_aside:=true)
  exec ros2 launch ./real_pode.launch.py \
  namespace:="$1" initial_x:="$2" initial_y:="$3" initial_yaw:="$4" "${extra[@]}"' \
  _ "$1" "$2" "$3" "$4" "$peers" "$dock" \
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
echo "started (speed $speed m/s, group $(cat logs/launch.pid)), log $PWD/$log"
