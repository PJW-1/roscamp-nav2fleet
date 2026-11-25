#!/usr/bin/env bash
# pode_fsm 한 판: 입력 재현을 띄우고 pode_fsm을 띄워 워밍업 뒤 CPU를 잰다.
# 사용: BAG=<bag 디렉터리> OUT_DIR=<결과 폴더> measure.sh <install_dir> <label> [측정초] [py-spy 출력] [taskset 코어]
# py-spy는 PYSPY(기본 py-spy)로 찾는다. ROS_DOMAIN_ID는 겹치지 않게 주고 부른다.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
INSTALL=$1; LABEL=$2; SECS=${3:-60}; SPY=${4:-}; CORES=${5:-}
BAG=${BAG:?bag 디렉터리를 BAG로 준다}
PYSPY=${PYSPY:-py-spy}
OUT_DIR=${OUT_DIR:-$PWD/runs}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-44}
[ "$ROS_DOMAIN_ID" = 6 ] && { echo "domain 6 is the real network" >&2; exit 1; }
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
set +u; source /opt/ros/jazzy/setup.bash; source "$INSTALL/setup.bash"; set -u
mkdir -p "$OUT_DIR"
OUT="$OUT_DIR/$LABEL"
python3 "$HERE/replay_inputs.py" "$BAG" --ns pode2 > "$OUT.replay.log" 2>&1 &
REPLAY=$!
sleep 6
EXE="$INSTALL/pode_fsm/lib/pode_fsm/pode_fsm_node"
ARGS=(--ros-args -r __ns:=/pode2 -r /tf:=tf -r /tf_static:=tf_static
      -p robot_id:=pode2 -p use_sim_time:=false -p through_controller_id:=FollowPathThrough
      -p heartbeat_timeout_s:=1.5)
PRE=()
[ -n "$CORES" ] && PRE=(taskset -c "$CORES")
if [ -n "$SPY" ]; then
  # ptrace는 자식에게만 되므로 py-spy가 pode_fsm을 띄운다(기록은 시작부터 측정초+15초, import 표본은 spy_sum.py가 뺀다)
  "${PRE[@]}" "$PYSPY" record -r 200 -f raw -o "$SPY" --nonblocking \
    --duration $((SECS + 15)) -- python3 "$EXE" "${ARGS[@]}" > "$OUT.fsm.log" 2>&1 &
  SPYPID=$!
  sleep 2
  FSM=$(pgrep -P $SPYPID -f pode_fsm_node | head -1)
else
  "${PRE[@]}" python3 "$EXE" "${ARGS[@]}" > "$OUT.fsm.log" 2>&1 &
  FSM=$!
fi
sleep 13
ticks() { awk '{print $14+$15}' "/proc/$1/stat"; }
HZ=$(getconf CLK_TCK)
a=$(ticks $FSM); t0=$(date +%s.%N)
sleep "$SECS"
b=$(ticks $FSM); t1=$(date +%s.%N)
python3 - "$a" "$b" "$t0" "$t1" "$HZ" "$LABEL" <<'EOF'
import sys
a, b, t0, t1, hz, label = sys.argv[1:]
cpu = (int(b) - int(a)) / int(hz) / (float(t1) - float(t0)) * 100
print(f"{label}: pode_fsm CPU {cpu:.1f} % over {float(t1) - float(t0):.0f} s")
EOF
# 상태가 실물처럼 자세를 갖는지 한 번 본다
timeout 5 ros2 topic echo --once /pode2/state fleet_interfaces/msg/RobotState --field state 2>/dev/null | head -1 | sed "s/^/$LABEL state: /"
kill -INT $FSM; sleep 2; kill -0 $FSM 2>/dev/null && kill -TERM $FSM
kill -INT $REPLAY; sleep 1; kill -0 $REPLAY 2>/dev/null && kill -TERM $REPLAY
[ -n "$SPY" ] && wait $SPYPID 2>/dev/null
wait $REPLAY 2>/dev/null
true
