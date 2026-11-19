#!/usr/bin/env bash
# 시뮬 묶음(판마다 실제 위치 기록기 같이): 한 판씩 차례로, 도메인 61
# 사용: sim_batch.sh BASE_DIR OVERLAY_SETUP|- name:scenario ...
S=/tmp/claude-1000/-home-ane-ros2camp/255eff68-4c77-42b3-b9fc-325ae2037b61/scratchpad
B=$1; OVL=$2; shift 2
left61() { for p in /proc/[0-9]*; do { tr '\0' '\n' < $p/environ; } 2>/dev/null | grep -qx ROS_DOMAIN_ID=61 && echo ${p#/proc/}; done; }
for run in "$@"; do
  name=${run%%:*}; scen=${run#*:}; R=$B/$name
  for i in $(seq 24); do [ -z "$(left61 | grep -v "^$$\$")" ] && break; sleep 5; done
  rest=$(left61 | grep -v "^$$\$"); if [ -n "$rest" ]; then echo "$name: domain 61 processes left: $rest, stop"; exit 1; fi
  mkdir -p $R
  ovl_line=""; [ "$OVL" != - ] && ovl_line="source $OVL"
  echo "scenario $scen, layout v7, use_order_model true, sim_dock_stub true, fleet_record true, domain 61, overlay $OVL, start $(date +%T)" > $R/run_info.txt
  echo "== $name $scen start $(date +%T)"
  ( cd $R && env -i HOME=$HOME PATH=/usr/local/bin:/usr/bin:/bin DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority XDG_RUNTIME_DIR=/run/user/1000 LANG=${LANG:-C.UTF-8} bash --norc --noprofile -c "
source /opt/ros/jazzy/setup.bash
source ~/pinky_deps_ws/install/local_setup.bash
source ~/ros2camp/nav2fleet_ws/install/setup.bash
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
