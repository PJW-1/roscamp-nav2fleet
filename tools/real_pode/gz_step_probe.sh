#!/usr/bin/env bash
# Gazebo Pode 한 대(Nav2 없음)에 cmd_step_probe 모든 묶음을 돌리고 step_report로 보고서를 쓴다.
# 먼저 ROS, ~/pinky_deps_ws, fleet_gz_sim을 빌드한 작업공간을 source하고 ROS_DOMAIN_ID(43~101)를 정한다.
#   GUI=true SETS="linear arc" PROBE_ARGS="--repeats 1" OUT=dir tools/real_pode/gz_step_probe.sh
here=$(cd "$(dirname "$(readlink -f "$0")")" && pwd) || exit 1
repo=$(cd "$here/../.." && pwd)
ns=${NS:-pode1}
sets=${SETS:-linear angular small_rot arc}
out=${OUT:-$repo/results/$(date +%F)/step_probe_gz}
# ---- 격리 확인 ----
case "${ROS_DOMAIN_ID:-}" in
  ''|6|12|48|5[0-9]|61) echo "set ROS_DOMAIN_ID (not 6, 12, 48, 50-59, 61), e.g. 63"; exit 2 ;;
esac
[ "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" = LOCALHOST ] || {
  echo "need ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"; exit 2; }
ros2 pkg prefix fleet_gz_sim > /dev/null 2>&1 || { echo "fleet_gz_sim not found: source the workspace"; exit 2; }
export GALLIUM_DRIVER=${GALLIUM_DRIVER:-d3d12}
mkdir -p "$out" || exit 1
started=$(date +%s)
# ---- Gazebo 띄우기 ----
setsid ros2 launch "$here/gz_probe.launch.py" namespace:="$ns" gui:="${GUI:-false}" \
  > "$out/launch.log" 2>&1 < /dev/null &
group=$!
stop_sim() {
  kill -INT -- "-$group" 2> /dev/null
  for _ in $(seq 60); do kill -0 -- "-$group" 2> /dev/null || return 0; sleep 0.5; done
  kill -TERM -- "-$group" 2> /dev/null
  for _ in $(seq 20); do kill -0 -- "-$group" 2> /dev/null || return 0; sleep 0.5; done
  echo "Gazebo group $group still running after INT and TERM; stop it by hand"
}
trap stop_sim EXIT
trap 'exit 130' INT TERM
echo "waiting for /$ns/odom (log $out/launch.log)"
ok=
for _ in $(seq 24); do
  kill -0 "$group" 2> /dev/null || { echo "launch ended; see $out/launch.log"; exit 1; }
  if ros2 topic echo --once --timeout 5 "/$ns/odom" nav_msgs/msg/Odometry > /dev/null 2>&1; then
    ok=1; break
  fi
done
[ -n "$ok" ] || { echo "no /$ns/odom within 2 minutes; see $out/launch.log"; exit 1; }
sleep 2
# ---- 묶음마다 한 번 ----
failed=
for s in $sets; do
  echo "== probe set $s"
  # shellcheck disable=SC2086
  python3 "$here/cmd_step_probe.py" --ns "$ns" --set "$s" --use-sim-time \
    --out "$out/step_$s.csv" $PROBE_ARGS || { echo "probe set $s failed ($?)"; failed="$failed $s"; }
done
python3 "$here/scan_info.py" --ns "$ns" --seconds 5 --out "$out/scan_gz.json" > /dev/null \
  || echo "scan_info failed"
python3 "$here/step_report.py" "$out"/step_*.csv --out "$out" \
  --title "Gazebo Pode cmd_vel 계단 응답 ($(date +%F))" > /dev/null || failed="$failed report"
echo "wrote $out/report.md in $(( $(date +%s) - started )) s"
[ -z "$failed" ] || { echo "failed:$failed"; exit 1; }
