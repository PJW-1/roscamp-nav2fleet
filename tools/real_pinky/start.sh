#!/usr/bin/env bash
# bringup + Nav2 + pinky_fsm을 뒤에서 띄운다:  start.sh pinky1 0.12 0.10 1.5708 [0.12]
# (namespace, 지도 위 로봇 자리 x y yaw, 속도 m/s 기본 0.12). 기록: logs/launch_*.log
cd "$(dirname "$(readlink -f "$0")")" || exit 1
[ $# -eq 4 ] || [ $# -eq 5 ] || { echo "usage: start.sh NAMESPACE X Y YAW [SPEED]"; exit 2; }
speed=${5:-0.12}
if [ -s logs/launch.pid ] && pgrep -g "$(cat logs/launch.pid)" > /dev/null; then
  echo "already running (group $(cat logs/launch.pid)); run stop.sh first"; exit 1
fi
# 속도 사본을 먼저 만든다(틀린 속도면 아무것도 띄우지 않는다)
python3 make_params.py --speed "$speed" --out params || exit 1
source env.sh || exit 1
mkdir -p logs || exit 1
log=logs/launch_$(date +%H%M%S).log
setsid bash -c 'echo $$ > logs/launch.pid; exec ros2 launch ./real_pinky.launch.py \
  namespace:="$1" initial_x:="$2" initial_y:="$3" initial_yaw:="$4"' _ "$1" "$2" "$3" "$4" \
  > "$log" 2>&1 < /dev/null &
echo "started (speed $speed m/s), log $PWD/$log"
