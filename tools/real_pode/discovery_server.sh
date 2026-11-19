#!/usr/bin/env bash
# 관제 PC에서 Fast DDS 디스커버리 서버를 앞에서 돌린다. 시험 전에 먼저 켜 두고, 끌 때는 Ctrl+C
# (fastdds는 sh, python, 서버 프로그램이 겹쳐 있어 PID 하나에 TERM을 보내면 서버가 남을 수 있다)
# 포트는 net.sh 기본 주소(192.168.0.252:11811)와 같다. 주소를 주지 않아 모든 망(유선, Wi-Fi)에서 듣는다
set +u
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}" || { echo "discovery_server.sh: ROS setup 없음" >&2; exit 1; }
# 서버 자신은 다른 서버의 클라이언트가 아니다
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
exec fastdds discovery -i 0 -p 11811
