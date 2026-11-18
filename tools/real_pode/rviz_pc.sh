#!/usr/bin/env bash
# PC에서 실물 시험 RViz를 띄운다. 로봇마다 TF가 /<ns>/tf라서 전체 창 + 로봇별 창을 연다.
#   rviz_pc.sh pode1 pode2                            # 전체 창 1개 + 로봇별 창 2개
#   RVIZ_MODE=overview rviz_pc.sh pode1 pode2 pode3   # 전체 창만
#   RVIZ_MODE=robots rviz_pc.sh pode1 pode2           # 로봇별 창만
# 환경은 PC 관제와 같다: 도메인 12, SUBNET (env.sh와 같은 값. PC에는 ~/nav2fleet_ws가 없어 직접 둔다).
set -eu
[ $# -ge 1 ] || { echo "사용: $0 pode1 [pode2 ...]" >&2; exit 2; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
# PC 관제와 같은 작업공간(있으면)
WS_SETUP="${NAV2FLEET_WS_SETUP:-$HOME/ros2camp/nav2fleet_ws/install/setup.bash}"
[ ! -f "$WS_SETUP" ] || source "$WS_SETUP"
set -u
export ROS_DOMAIN_ID=12
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
MODE="${RVIZ_MODE:-all}"
TMP="$(mktemp -d)"
trap 'kill $(jobs -p) 2>/dev/null || true; rm -rf "$TMP"' EXIT INT TERM
if [ "$MODE" != robots ]; then
  python3 "$HERE/rviz_config.py" overview "$@" > "$TMP/overview.rviz"
  # map 틀이 있어야 해서 첫 로봇의 TF를 읽는다
  rviz2 -d "$TMP/overview.rviz" --ros-args -r "/tf:=/$1/tf" -r "/tf_static:=/$1/tf_static" &
fi
if [ "$MODE" != overview ]; then
  for ns in "$@"; do
    python3 "$HERE/rviz_config.py" per "$ns" > "$TMP/$ns.rviz"
    rviz2 -d "$TMP/$ns.rviz" --ros-args -r "/tf:=/$ns/tf" -r "/tf_static:=/$ns/tf_static" &
  done
fi
wait
