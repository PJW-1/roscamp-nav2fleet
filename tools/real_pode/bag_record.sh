#!/usr/bin/env bash
# 로봇 안에서 원인 분석용 토픽을 rosbag으로 남긴다(SD 카드에 쓰므로 Wi-Fi가 끊겨도 남는다)
#   bag_record.sh start pode1 1124_run1   # bags/<판 이름>에 쓴다
#   bag_record.sh stop
# 담는 토픽(모두 /<ns>/ 아래): scan, odom, cmd_vel, amcl_pose, particle_cloud, plan, plan_kept, tf, tf_static, state
# 새 토픽을 잡는 데 몇 초 걸리므로 start.sh보다 먼저 켠다. 디스커버리 서버를 쓰면 SUPER=1로 전체 그래프를 본다
cd "$(dirname "$(readlink -f "$0")")" || exit 1
mkdir -p logs bags || exit 1
here=$(pwd -P)

usage() { echo "usage: bag_record.sh start NAMESPACE NAME | stop"; exit 2; }

# 다른 start/stop과 겹치지 않게 잠근다
lock() {
  exec 9> logs/bag.lock
  flock -n 9 || { echo "another bag_record.sh is running; try again"; exit 1; }
}

# pid 파일의 녹화기가 살아 있는 우리 것인지 본다(번호 재사용 대비: 작업 폴더와 bag 인자가 같아야 한다)
ours() {
  local pid name
  read -r pid name 2> /dev/null < logs/bag.pid || return 1
  [[ $pid =~ ^[0-9]+$ && -n $name ]] || return 1
  [ "$(readlink "/proc/$pid/cwd" 2> /dev/null)" = "$here" ] || return 1
  tr '\0' '\n' 2> /dev/null < "/proc/$pid/cmdline" | grep -qxF "bags/$name" || return 1
  echo "$pid $name"
}

# 살아 있는 프로세스인지 본다(좀비 제외)
alive() { [[ $(ps -o stat= -p "$1" 2> /dev/null | tr -d ' ') == [^Z]* ]]; }

# 녹화기 그룹에 신호를 보내고 끝나기를 기다린다. 끝나면 0
signal_and_wait() {  # PID SIGNAL TRIES(0.5 s)
  kill "-$2" -- "-$1" 2> /dev/null || kill "-$2" "$1" 2> /dev/null
  for _ in $(seq "$3"); do alive "$1" || return 0; sleep 0.5; done
  ! alive "$1"
}

case $1 in
  start)
    [ $# -eq 3 ] || usage
    ns=$2 name=$3
    [[ $ns =~ ^[a-z_][a-z0-9_]*$ ]] || { echo "bad NAMESPACE: $ns"; exit 2; }
    [[ $name =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "bad NAME: $name"; exit 2; }
    timeout_s=${BAG_START_TIMEOUT_S:-20}
    [[ $timeout_s =~ ^[1-9][0-9]*$ ]] || { echo "bad BAG_START_TIMEOUT_S: $timeout_s"; exit 2; }
    lock
    if cur=$(ours); then
      echo "already recording (pid ${cur% *}, bags/${cur#* }); run bag_record.sh stop first"; exit 1
    fi
    [ -e "bags/$name" ] && { echo "bags/$name exists; use another NAME"; exit 1; }
    SUPER=1 source env.sh || exit 1
    # 새 세션으로 띄운다. 작업 제어 없이 뒤에서 도는 명령은 SIGINT를 무시하므로 env로 되돌린다
    # (stop이 SIGINT로 bag을 마무리한다). 잠금 fd는 넘기지 않는다
    setsid env --default-signal=INT bash -c \
      'exec ros2 bag record -o "$2" -s mcap -e "^/$1/(scan|odom|cmd_vel|amcl_pose|particle_cloud|plan|plan_kept|tf|tf_static|state)$"' \
      _ "$ns" "bags/$name" > "logs/bag_$name.log" 2>&1 < /dev/null 9>&- &
    pid=$!
    if ! echo "$pid $name" > logs/bag.pid; then
      echo "cannot write logs/bag.pid; stopping recorder $pid"
      signal_and_wait "$pid" INT 10 || signal_and_wait "$pid" TERM 10 || {
        echo "recorder $pid is still alive; stop it by hand"; exit 1; }
      exit 1
    fi
    # bag 파일이 생기면 녹화기가 준비된 것이다(rosbag2는 첫 메시지 전에 파일을 연다)
    # 방금 띄운 자식이라 exec 전이어도 놓치지 않게 ours 대신 alive로 본다
    for _ in $(seq $((timeout_s * 10))); do
      alive "$pid" || break
      if compgen -G "bags/$name/*.mcap" > /dev/null; then
        echo "recording /$ns to $here/bags/$name (pid $pid), log logs/bag_$name.log"; exit 0
      fi
      sleep 0.1
    done
    if alive "$pid"; then
      echo "recorder did not open bags/$name in $timeout_s s; stopping it"
      signal_and_wait "$pid" INT 10 || signal_and_wait "$pid" TERM 10 || {
        echo "recorder $pid is still alive; stop it by hand"; exit 1; }
    fi
    rm -f logs/bag.pid
    echo "recorder did not start; see $here/logs/bag_$name.log"
    tail -5 "logs/bag_$name.log"
    exit 1
    ;;
  stop)
    [ $# -eq 1 ] || usage
    lock
    if ! cur=$(ours); then
      # pid 파일만 남았다: 녹화 중에 죽었으면 bag이 마무리되지 않았을 수 있다
      name=
      read -r _ name 2> /dev/null < logs/bag.pid
      rm -f logs/bag.pid
      if [[ $name =~ ^[A-Za-z0-9_.-]+$ && -d bags/$name && ! -f bags/$name/metadata.yaml ]]; then
        echo "recorder for bags/$name is gone and the bag was not finalized; run: ros2 bag reindex bags/$name"
        exit 1
      fi
      echo "not recording"; exit 0
    fi
    pid=${cur% *} name=${cur#* }
    # SIGINT로 멈춰야 bag을 마무리한다(metadata.yaml이 써진다)
    signal_and_wait "$pid" INT 40 || { echo "still alive after 20 s (pid $pid)"; exit 1; }
    rm -f logs/bag.pid
    [ -f "bags/$name/metadata.yaml" ] || {
      echo "stopped, but bags/$name/metadata.yaml is missing; run: ros2 bag reindex bags/$name"; exit 1; }
    echo "stopped; bag at $here/bags/$name"
    ;;
  *) usage ;;
esac
