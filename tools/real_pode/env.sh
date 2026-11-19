# 실물 Pode 시험 환경. start.sh가 읽는다. 로봇에서 ros2 명령을 칠 때도 먼저 source한다
for f in "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}" "$HOME/pinky_pro/install/local_setup.bash" \
    "$HOME/nav2fleet_ws/install/local_setup.bash"; do
  [ -f "$f" ] || { echo "env.sh: 없음 $f (nav2fleet_ws는 colcon build 뒤에 생긴다)"; return 1; }
  source "$f"
done
# 도메인, RMW, 디스커버리 서버는 net.sh 한곳에 둔다
source "$(dirname "${BASH_SOURCE[0]}")/net.sh"
