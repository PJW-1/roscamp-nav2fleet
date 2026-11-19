#!/usr/bin/env bash
# 시뮬 묶음(판마다 실제 위치 기록기 같이): 한 판씩 차례로, 도메인 61
# 사용: sim_batch.sh BASE_DIR OVERLAY_SETUP|- name:scenario ...
# PC마다 다른 것은 환경 변수로 준다(안 주면 2026-11-19 기준 묶음을 돌린 PC의 값):
#   SIM_BATCH_INSTALL  공유 install의 setup.bash
#   SIM_BATCH_ENV      깨끗한 env에 더 넣을 변수들(예: "WAYLAND_DISPLAY=wayland-0 GALLIUM_DRIVER=d3d12")
#                      빈칸으로 나눈 NAME=value 목록이다. 값에 빈칸이나 *를 쓰지 못한다
#   SIM_BATCH_PAUSE_S  판마다 시작하기 전에 쉬는 시간(초). 첫 판 앞에서도 쉰다
# 기록기(truth_logger.py)는 이 스크립트가 있는 폴더의 것을 쓴다.
S=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
INSTALL=${SIM_BATCH_INSTALL:-$HOME/ros2camp/nav2fleet_ws/install/setup.bash}
EXTRA_ENV=${SIM_BATCH_ENV-XAUTHORITY=/run/user/1000/gdm/Xauthority}
B=$1; OVL=$2; shift 2
# 묶음 로그 시각은 KST로 적는다(express_metrics.py, q2_dwell.py가 KST로 읽음)
export TZ=Asia/Seoul
left61() { for p in /proc/[0-9]*; do { tr '\0' '\n' < $p/environ; } 2>/dev/null | grep -qx ROS_DOMAIN_ID=61 && echo ${p#/proc/}; done; }
for run in "$@"; do
  name=${run%%:*}; scen=${run#*:}; R=$B/$name
  for i in $(seq 24); do [ -z "$(left61 | grep -v "^$$\$")" ] && break; sleep 5; done
  rest=$(left61 | grep -v "^$$\$"); if [ -n "$rest" ]; then echo "$name: domain 61 processes left: $rest, stop"; exit 1; fi
  sleep "${SIM_BATCH_PAUSE_S:-0}"
  mkdir -p $R
  ovl_line=""; [ "$OVL" != - ] && ovl_line="source $OVL"
  echo "scenario $scen, layout v7, use_order_model true, sim_dock_stub true, fleet_record true, domain 61, overlay $OVL, start $(date +%T)" > $R/run_info.txt
  echo "== $name $scen start $(date +%T)"
  ( cd $R && env -i HOME=$HOME PATH=/usr/local/bin:/usr/bin:/bin DISPLAY=:0 XDG_RUNTIME_DIR=/run/user/1000 LANG=${LANG:-C.UTF-8} $EXTRA_ENV bash --norc --noprofile -c "
source /opt/ros/jazzy/setup.bash
source ~/pinky_deps_ws/install/local_setup.bash
source '$INSTALL'
$ovl_line
export ROS_DOMAIN_ID=61 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 -c 'import fleet_core; print(\"fleet_core from\", fleet_core.__file__)'
python3 $S/truth_logger.py $R/truth.csv 61 3 > truth_logger.out 2>&1 &
LOG=\$!
( iso=0; while kill -0 \$\$ 2>/dev/null; do
    if [ \$iso = 0 ] && grep -q ROBOT_ISOLATED launch.out; then iso=\$(date +%s); fi
    if [ \$iso != 0 ] && [ \$iso != stop ] && [ \$(( \$(date +%s) - iso )) -ge 45 ]; then echo \"watchdog: ROBOT_ISOLATED, Ctrl+C at \$(date +%T)\" > watchdog.txt; kill -INT \$\$; iso=stop; fi
    sleep 3
  done; kill -INT \$LOG ) &
exec timeout -s INT -k 60 1500 ros2 launch fleet_gz_sim gz.launch.py layout:=v7 scenario:=$scen use_order_model:=true sim_dock_stub:=true fleet_record:=true rviz:=fleet gui:=true result_dir:=$R/result
" > launch.out 2>&1 )
  rc=$?
  echo "== $name exit $rc end $(date +%T) summary: $(python3 -c "import json;d=json.load(open('$R/result/summary.json'));print(d['pass'],d['orders_shipped'],'/',d['orders_total'],round(d['duration_s'],1),'s')" 2>&1 | tail -1)"
done
echo "batch done"
