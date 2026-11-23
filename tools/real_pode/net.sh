# 실물 시험 통신 설정. 로봇(env.sh)과 PC(pc_env.sh)가 같이 읽는다
#   기본: 관제 PC의 디스커버리 서버(192.168.0.252:11811)로 서로 찾는다
#   DISCOVERY_SERVER=off       서버 없이 예전처럼 멀티캐스트로 찾는다
#   DISCOVERY_SERVER=IP:PORT   다른 서버
#   SUPER=1                    전체 그래프를 보는 터미널(ros2 명령, RViz)
# 다시 source해도 이전 값이 남지 않게 먼저 지운다
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
# 로봇 네트워크 도메인은 6(팀 통신 규칙 IR-81, 2026-11-20). 시험은 6이 아닌 도메인 + LOCALHOST
export ROS_DOMAIN_ID=6
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
_net_ds=${DISCOVERY_SERVER:-192.168.0.252:11811}
if [ "$_net_ds" != off ]; then
  export ROS_DISCOVERY_SERVER=$_net_ds
  if [ "${SUPER:-0}" = 1 ]; then export ROS_SUPER_CLIENT=TRUE; fi
fi
unset _net_ds
