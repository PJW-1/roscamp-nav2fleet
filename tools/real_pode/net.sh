# 실물 시험 통신 설정. 로봇(env.sh)과 PC(pc_env.sh)가 같이 읽는다
#   기본: 관제 PC의 디스커버리 서버(192.168.0.252:11811)로 서로 찾는다
#   DISCOVERY_SERVER=off       서버 없이 예전처럼 멀티캐스트로 찾는다
#   DISCOVERY_SERVER=IP:PORT   다른 서버
#   SUPER=1                    전체 그래프를 보는 터미널(ros2 명령, RViz)
#   FLEET_DOMAIN_ID=N          도메인(기본 6, 팀 규칙 IR-81). 0~232 정수
# 도메인을 먼저 검사한다. 틀리면 아무것도 바꾸지 않고 1
_net_dom=${FLEET_DOMAIN_ID:-6}
case $_net_dom in
  ''|*[!0-9]*) _net_dom= ;;
  *)
    # 앞의 0을 떼고 자릿수를 본 뒤 비교한다(긴 수는 셸 산술이 넘친다)
    _net_dom=${_net_dom#"${_net_dom%%[!0]*}"}
    _net_dom=${_net_dom:-0}
    case $_net_dom in
      ????*) _net_dom= ;;
      *) [ "$_net_dom" -le 232 ] || _net_dom= ;;
    esac ;;
esac
if [ -z "$_net_dom" ]; then
  echo "net.sh: FLEET_DOMAIN_ID는 0~232 정수여야 한다: '${FLEET_DOMAIN_ID:-}'" >&2
  unset _net_dom
  return 1
fi
# 다시 source해도 이전 값이 남지 않게 먼저 지운다
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export ROS_DOMAIN_ID=$_net_dom
unset _net_dom
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
_net_ds=${DISCOVERY_SERVER:-192.168.0.252:11811}
if [ "$_net_ds" != off ]; then
  export ROS_DISCOVERY_SERVER=$_net_ds
  if [ "${SUPER:-0}" = 1 ]; then export ROS_SUPER_CLIENT=TRUE; fi
fi
unset _net_ds
