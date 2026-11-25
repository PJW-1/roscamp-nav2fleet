#!/usr/bin/env bash
# 재생 한 판: 자기 도메인에서 AMCL을 띄우고 bag 한 구간을 틀어 정한 시각의 자세를 남긴다.
# 쓰기: run_one.sh DOMAIN BAG MAP_YAML LABEL START_OFFSET_S DURATION_S X Y YAW_RAD TIMES OUT_JSON
set -euo pipefail
[ $# -eq 11 ] || { sed -n 2,3p "$0"; exit 2; }
DOMAIN=$1 BAG=$2 MAP=$3 LABEL=$4 START=$5 DUR=$6 X=$7 Y=$8 YAW=$9 TIMES=${10} OUT=${11}
HERE=$(cd "$(dirname "$0")" && pwd)

# 다른 세션과 로봇망에 섞이지 않게 이 PC 안, 자기 도메인만 쓴다
export ROS_DOMAIN_ID=$DOMAIN ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_DISCOVERY_SERVER ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
set +u; source /opt/ros/jazzy/setup.bash; set -u

LOG=${OUT%.json}.log
setsid ros2 launch "$HERE/replay.launch.py" map_fine:="$MAP" x:="$X" y:="$Y" yaw:="$YAW" \
  > "$LOG" 2>&1 &
LPID=$!
PPID_=
cleanup() {
  [ -n "$PPID_" ] && kill "$PPID_" 2>/dev/null || true  # 중간에 끝나도 probe를 남기지 않는다
  kill -INT -- "-$LPID" 2>/dev/null || true; sleep 2; kill -KILL -- "-$LPID" 2>/dev/null || true
}
trap cleanup EXIT

# AMCL이 active가 될 때까지(지도 받고 초기 자세 둔 뒤)
for _ in $(seq 60); do
  grep -q "Managed nodes are active" "$LOG" && break; sleep 0.5
done
grep -q "Managed nodes are active" "$LOG" || { echo "[run] $LABEL: AMCL이 active가 안 됨"; exit 1; }

python3 "$HERE/amcl_replay.py" probe --bag "$BAG" --times "$TIMES" --map-label "$LABEL" --out "$OUT" \
  >> "$LOG" 2>&1 &
PPID_=$!
sleep 1
ros2 bag play "$BAG" --clock 100 --start-offset "$START" --playback-duration "$DUR" \
  --disable-keyboard-controls >> "$LOG" 2>&1 || true
wait "$PPID_"
PPID_=
