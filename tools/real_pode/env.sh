# 실물 Pode 시험 환경(2026-11-10 두 대 시험과 같다). 로봇과 PC 모두 도메인 12.
for f in /opt/ros/jazzy/setup.bash "$HOME/pinky_pro/install/local_setup.bash" \
    "$HOME/nav2fleet_ws/install/local_setup.bash"; do
  [ -f "$f" ] || { echo "env.sh: 없음 $f (nav2fleet_ws는 colcon build 뒤에 생긴다)"; return 1; }
  source "$f"
done
export ROS_DOMAIN_ID=12
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
