# 실물 판 기록(jsonl)을 따라 읽으며 볼 일만 한 줄씩 낸다(읽기만 한다)
import json, os, sys, time

path = sys.argv[1]
WATCH = {"BLOCKED", "ROBOT_ISOLATED", "ISOLATED", "COMM_LOST", "COMM_RESTORED", "NAV_ABORTED",
         "GOAL_ABORTED_LOCALIZING", "LOCAL_YIELD", "HALL_YIELD", "BASE_END_WAIT", "ROBOT_NOT_REPORTING",
         "LOCALIZATION_SUSPECT", "ORDER_BOUND", "DOCKED", "DOCK_FAILED", "HEARTBEAT_LOST", "LANE_ERROR"}
STATES = {"LOCALIZING", "RECOVERING", "ERROR", "BLOCKED", "FAULT", "ESTOP"}
t0 = None
last = {}      # 로봇별 마지막 상태 시각
state = {}     # 로봇별 마지막 상태
shipped = []
quiet_said = False
last_line = time.time()


def say(t, text):
    print(f"+{t - t0:6.1f}s {text}" if t0 else text, flush=True)


while not os.path.exists(path):
    time.sleep(0.5)
f = open(path)
while True:
    line = f.readline()
    if not line:
        if time.time() - last_line > 15 and not quiet_said:
            print("기록이 15 s 넘게 멈춤 (관제가 꺼졌나?)", flush=True)
            quiet_said = True
        time.sleep(0.2)
        continue
    last_line = time.time()
    quiet_said = False
    try:
        r = json.loads(line)
    except ValueError:
        continue
    t = r.get("t")
    if t is None:
        continue
    if t0 is None:
        t0 = t
        say(t, "기록 시작")
    if r.get("in") == "on_robot_state":
        a = r["args"]
        name, s = a[0], a[4]
        if name in last and t - last[name] > 0.6:
            say(t, f"{name} 상태 끊김 {t - last[name]:.1f}s")
        last[name] = t
        if name not in state:
            say(t, f"{name} 첫 보고 {s} ({a[1]:.2f}, {a[2]:.2f})")
        elif s != state[name] and (s in STATES or state[name] in STATES):
            say(t, f"{name} {state[name]} -> {s} ({a[1]:.2f}, {a[2]:.2f})")
        state[name] = s
    elif r.get("out") == "EventOut":
        code = r.get("code", "")
        if r.get("level") in ("ERROR", "WARN") or code in WATCH:
            say(t, f"{r.get('level')} {code} {r.get('robot', '')} {r.get('detail', '')}"[:200])
    elif r.get("out") == "ReportOrderCmd":
        shipped.append(r.get("order_id"))
        say(t, f"주문 {r.get('order_id')} {r.get('status')} by {r.get('robot')} ({len(shipped)}/4)")
        if len(shipped) >= 4:
            say(t, "주문 4건 보고 끝")
