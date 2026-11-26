#!/usr/bin/env bash
# pode2 Nav2에 목표 하나를 보내고 결과를 기다린다:  goal.sh X Y YAW_DEG
set -eo pipefail
x=$1; y=$2; deg=$3
read qz qw < <(python3 -c "import math; a=math.radians($deg)/2; print(round(math.sin(a),6), round(math.cos(a),6))")
cd /home/ane/ros2camp/nav2fleet_ws/src/roscamp-nav2fleet
set +u; SUPER=1 source tools/real_pode/pc_env.sh
echo "$(date +%T) goal ($x, $y, $deg deg)"
timeout 120 ros2 action send_goal /pode2/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $x, y: $y}, orientation: {z: $qz, w: $qw}}}}" 2>&1 \
  | grep -E "accepted|rejected|Result|status|error_code|SUCCEEDED|ABORTED|CANCELED" | head -8
echo "$(date +%T) done"
