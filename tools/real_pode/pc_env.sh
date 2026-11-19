# PC 터미널 환경: ROS, PC 작업공간, 통신 설정(net.sh). 새 bash 터미널에서 source한다
#   source tools/real_pode/pc_env.sh                         # 관제(pc.launch.py)
#   SUPER=1 source tools/real_pode/pc_env.sh                 # ros2 명령 터미널
#   DISCOVERY_SERVER=off source tools/real_pode/pc_env.sh    # 서버 없이 예전처럼
# 경로는 ROS_SETUP, NAV2FLEET_WS_SETUP으로 바꿀 수 있다
# 실패(ROS setup 없음, setup 실패, net.sh 못 읽음)면 1. 작업공간 파일이 없을 때만 알리고 계속한다
# ROS setup은 정의되지 않은 변수를 읽으므로 nounset을 잠시 끄고, 원래 켜져 있었을 때만 다시 켠다
_pc_u=
case $- in *u*) _pc_u=1; set +u ;; esac
_pc_rc=0
_pc_ros=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}
_pc_ws=${NAV2FLEET_WS_SETUP:-$HOME/ros2camp/nav2fleet_ws/install/setup.bash}
_pc_net="$(dirname "${BASH_SOURCE[0]}")/net.sh"
if [ ! -f "$_pc_ros" ]; then
  echo "pc_env.sh: 없음 $_pc_ros" >&2; _pc_rc=1
elif ! source "$_pc_ros"; then
  echo "pc_env.sh: 실패 $_pc_ros" >&2; _pc_rc=1
elif [ ! -f "$_pc_ws" ]; then
  echo "pc_env.sh: 작업공간 없음 $_pc_ws (ROS만 읽었다)" >&2
elif ! source "$_pc_ws"; then
  echo "pc_env.sh: 실패 $_pc_ws" >&2; _pc_rc=1
fi
if [ "$_pc_rc" = 0 ] && ! { [ -f "$_pc_net" ] && source "$_pc_net"; }; then
  echo "pc_env.sh: net.sh를 읽지 못했다 $_pc_net" >&2; _pc_rc=1
fi
[ -z "$_pc_u" ] || set -u
unset _pc_u _pc_ros _pc_ws _pc_net
# 돌려줄 값을 먼저 펼친 뒤 마지막 보조 변수까지 지운다
eval "unset _pc_rc; return $_pc_rc"
