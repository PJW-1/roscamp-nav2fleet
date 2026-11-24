#!/usr/bin/env bash
# start.sh가 띄운 것을 내린다(프로세스 그룹에 SIGTERM. 뒤에서 도는 launch는 SIGINT를 무시한다).
cd "$(dirname "$(readlink -f "$0")")" || exit 1
pid=$(cat logs/launch.pid 2>/dev/null)
[ -n "$pid" ] && pgrep -g "$pid" > /dev/null || { echo "not running"; exit 0; }
# pid 파일이 오래되어 번호가 다시 쓰였을 수 있다: 우리 launch를 돌리는 그룹만 내린다
# NO_MOTION=1 start.sh는 pode_fsm.launch.py를 start.sh의 인자 순서 그대로 띄운다
pgrep -g "$pid" -f 'real_pode\.launch\.py|pode_fsm\.launch\.py namespace:=[^ ]+ no_motion:=true initial_x:=' \
  > /dev/null || {
  echo "group $pid is not real_pode.launch.py or pode_fsm.launch.py (old pid file?); nothing stopped"
  exit 1; }
kill -TERM -- "-$pid"
for _ in $(seq 40); do pgrep -g "$pid" > /dev/null || break; sleep 0.5; done
if pgrep -g "$pid" > /dev/null; then
  echo "still alive after 20 s: $(pgrep -g "$pid" -l | tr '\n' ' ')"; exit 1
fi
echo "stopped"
