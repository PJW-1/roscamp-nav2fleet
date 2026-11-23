#!/usr/bin/env bash
# 로봇 안에서 원인 분석용 토픽을 rosbag으로 남긴다(SD 카드에 쓰므로 Wi-Fi가 끊겨도 남는다)
#   bag_record.sh start pode1 1124_run1   # bags/<판 이름>에 쓴다
#   bag_record.sh stop
# 담는 토픽(모두 /<ns>/ 아래): scan, odom, cmd_vel, amcl_pose, particle_cloud, plan, tf, tf_static, state
# start.sh보다 먼저 켜도 된다(새 토픽을 계속 찾는다). 디스커버리 서버를 쓰면 SUPER=1로 전체 그래프를 본다
cd "$(dirname "$(readlink -f "$0")")" || exit 1
mkdir -p logs bags || exit 1
case $1 in
  start)
    [ $# -eq 3 ] || { echo "usage: bag_record.sh start NAMESPACE NAME"; exit 2; }
    ns=$2 name=$3
    [[ $ns =~ ^[a-z_][a-z0-9_]*$ ]] || { echo "bad NAMESPACE: $ns"; exit 2; }
    [[ $name =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "bad NAME: $name"; exit 2; }
    if [ -s logs/bag.pid ] && pgrep -g "$(cat logs/bag.pid)" -f "bag record" > /dev/null; then
      echo "already recording (group $(cat logs/bag.pid)); run bag_record.sh stop first"; exit 1
    fi
    [ -e "bags/$name" ] && { echo "bags/$name exists; use another NAME"; exit 1; }
    SUPER=1 source env.sh || exit 1
    rm -f logs/bag.pid
    # 작업 제어를 켜야 뒤에서 도는 녹화기가 SIGINT를 무시하지 않는다(stop이 SIGINT로 마무리한다)
    set -m
    setsid bash -c 'echo $$ > logs/bag.pid
      exec ros2 bag record -o "bags/$2" -e "^/$1/(scan|odom|cmd_vel|amcl_pose|particle_cloud|plan|tf|tf_static|state)$"' \
      _ "$ns" "$name" > "logs/bag_$name.log" 2>&1 < /dev/null &
    for _ in $(seq 30); do [ -s logs/bag.pid ] && break; sleep 0.1; done
    [ -s logs/bag.pid ] || { echo "recorder did not start; see $PWD/logs/bag_$name.log"; exit 1; }
    echo "recording /$ns to $PWD/bags/$name (group $(cat logs/bag.pid)), log logs/bag_$name.log"
    ;;
  stop)
    pid=$(cat logs/bag.pid 2>/dev/null)
    [ -n "$pid" ] && pgrep -g "$pid" -f "bag record" > /dev/null || { echo "not recording"; exit 0; }
    # SIGINT로 멈춰야 bag을 마무리한다(metadata.yaml이 써진다)
    kill -INT -- "-$pid"
    for _ in $(seq 40); do pgrep -g "$pid" > /dev/null || break; sleep 0.5; done
    if pgrep -g "$pid" > /dev/null; then echo "still alive after 20 s (group $pid)"; exit 1; fi
    rm -f logs/bag.pid
    echo "stopped"
    ;;
  *) echo "usage: bag_record.sh start NAMESPACE NAME | stop"; exit 2 ;;
esac
