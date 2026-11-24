#!/usr/bin/env bash
# 경로 뒤집힘 비교 묶음: 한 판씩 차례로, 도메인을 인자로 받는다(GUI 없음)
# 사용: run_batch.sh OUT_DIR INSTALL_SETUP DOMAIN 이름:시나리오[:launch인자,launch인자...] ...
#   launch 인자 자리의 install:=경로(setup.bash)는 그 판만 다른 빌드로, controller:=이름은 그 판의
#   Nav2 goal controller를 고른다(select_controller.py, 예 FollowPathRL). 둘은 launch에 넘기지 않는다
# 경로와 인자 값에 빈칸, 따옴표, 셸 특수문자를 쓰지 못한다. 묶음은 한 번에 하나만 돌린다(잠금 없음)
#   예: run_batch.sh ~/gz_runs/pf 'path/install/setup.bash' 93 b1v1:gz_v7_varied:nav_map_variant:=a1
# 판마다 남기는 것: launch.out, result/(summary.json, events.csv, fleet_record.jsonl), truth.csv,
#   bag/(계획 경로, 속도 명령, action feedback), cpu.csv와 cpu_proc.csv(cpu_sampler.py), run_info.txt
# 시나리오가 gz_v7_varied면 장애물 v7_yard_mix와 battery_low_pct:=20을 붙인다(11-17~18 판과 같은 조건)
S=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
TRUTH=$S/../baseline_sim_2026-11-19/truth_logger.py
B=$(realpath -m "$1"); INSTALL=$(realpath -m "$2"); DOM=$3; shift 3
export TZ=Asia/Seoul
mine() { for p in /proc/[0-9]*; do { tr '\0' '\n' < $p/environ; } 2>/dev/null | grep -qx ROS_DOMAIN_ID=$DOM && echo ${p#/proc/}; done; }
for run in "$@"; do
  name=${run%%:*}; rest=${run#*:}; scen=${rest%%:*}; extra=""
  [ "$rest" != "$scen" ] && extra=$(echo "${rest#*:}" | tr ',' ' ')
  inst=$INSTALL; ctrl=""; keep_args=""
  for a in $extra; do case $a in install:=*) inst=$(realpath -m "${a#install:=}");; controller:=*) ctrl=${a#controller:=};; *) keep_args="$keep_args $a";; esac; done
  extra=$keep_args
  [ "$scen" = gz_v7_varied ] && extra="obstacles:=v7_yard_mix battery_low_pct:=20 $extra"
  R=$B/$name
  if [ -e $R/result/summary.json ]; then echo "== $name already done, skip"; continue; fi
  # 끝나지 않은 판 폴더는 옆으로 치운다(기록이 섞이지 않게)
  if [ -e $R ]; then mv -n $R $R.unfinished-$(date +%H%M%S%N) && [ ! -e $R ] || { echo "$name: cannot move the unfinished folder aside, stop"; exit 1; }; echo "== $name: moved an unfinished folder aside"; fi
  for i in $(seq 24); do [ -z "$(mine | grep -v "^$$\$")" ] && break; sleep 5; done
  left=$(mine | grep -v "^$$\$"); if [ -n "$left" ]; then echo "$name: domain $DOM processes left: $left, stop"; exit 1; fi
  mkdir -p $R/result
  args="layout:=v7 scenario:=$scen use_order_model:=true sim_dock_stub:=true fleet_record:=true rviz:=none gui:=false $extra"
  echo "args $args, controller ${ctrl:-default}, domain $DOM, install $inst, start $(date +%F' '%T)" > $R/run_info.txt
  echo "== $name $scen ($extra) start $(date +%T)"
  ( cd $R && env -i HOME=$HOME PATH=/usr/local/bin:/usr/bin:/bin XDG_RUNTIME_DIR=/run/user/1000 LANG=${LANG:-C.UTF-8} bash --norc --noprofile -c "
source /opt/ros/jazzy/setup.bash
source ~/pinky_deps_ws/install/local_setup.bash
source '$inst' || { echo 'cannot source $inst'; exit 3; }
export ROS_DOMAIN_ID=$DOM ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 $TRUTH $R/truth.csv $DOM 3 > truth_logger.out 2>&1 &
TL=\$!
python3 $S/cpu_sampler.py $DOM $R 5 > cpu_sampler.out 2>&1 &
CS=\$!
( exec ros2 bag record -o bag --include-hidden-topics -e '/pode[123]/(plan|plan_kept|cmd_vel|cmd_vel_nav)\$|/pode[123]/.*/_action/feedback' ) > bag.out 2>&1 &
BG=\$!
SEL=
if [ -n '$ctrl' ]; then python3 $S/select_controller.py $DOM $ctrl 3 > select.out 2>&1 & SEL=\$!; fi
( iso=0; while kill -0 \$\$ 2>/dev/null; do
    if [ \$iso = 0 ] && grep -q ROBOT_ISOLATED launch.out; then iso=\$(date +%s); fi
    if [ \$iso != 0 ] && [ \$iso != stop ] && [ \$(( \$(date +%s) - iso )) -ge 45 ]; then echo \"watchdog: ROBOT_ISOLATED, Ctrl+C at \$(date +%T)\" > watchdog.txt; kill -INT \$\$; iso=stop; fi
    sleep 3
  done; kill -INT \$TL \$CS \$SEL; pkill -INT -P \$BG; kill -INT \$BG ) &
exec timeout -s INT -k 60 1800 ros2 launch fleet_gz_sim gz.launch.py $args result_dir:=$R/result
" > launch.out 2>&1 )
  rc=$?
  sleep 5
  echo "== $name exit $rc end $(date +%T) summary: $(python3 -c "import json;d=json.load(open('$R/result/summary.json'));print(d['pass'],d['orders_shipped'],'/',d['orders_total'],round(d['duration_s'],1),'s')" 2>&1 | tail -1)"
done
echo "batch done"
