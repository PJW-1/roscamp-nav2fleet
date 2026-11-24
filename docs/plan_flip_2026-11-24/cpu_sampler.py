"""한 판 동안 PC CPU 사용률과 그 도메인 프로세스의 CPU 시간을 남긴다.

사용: python3 cpu_sampler.py DOMAIN OUT_DIR [주기 s]
cpu.csv: 시각, PC 전체 CPU %(모든 코어 평균), 이 도메인 프로세스 CPU %(코어 하나 = 100).
cpu_proc.csv: 끝날 때 프로세스 이름(실행 파일과 첫 인자)마다 쓴 CPU 시간(s).
"""
import signal
import sys
import time
from pathlib import Path

import psutil

DOM, OUT = sys.argv[1], Path(sys.argv[2])
PERIOD = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
stop = False


def _stop(*_):
    global stop
    stop = True


signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)


def label(p):
    # 노드 이름(__node:=)이 있으면 그것, 없으면 실행 파일과 첫 인자
    cmd = p.cmdline()
    for a in cmd:
        if a.startswith("__node:="):
            return a.split("=", 1)[1]
    exe = Path(cmd[0]).name if cmd else p.name()
    if exe.startswith("python") and len(cmd) > 1:
        return Path(cmd[1]).name
    return exe


used = {}  # pid -> (이름, 마지막 CPU 시간)
with open(OUT / "cpu.csv", "w") as f:
    f.write("wall,pc_cpu_pct,domain_cpu_pct\n")
    psutil.cpu_percent(None)
    last_total = None
    while not stop:
        time.sleep(PERIOD)
        total = 0.0
        for p in psutil.process_iter():
            try:
                if p.pid in used:
                    name = used[p.pid][0]
                elif p.environ().get("ROS_DOMAIN_ID") == DOM:
                    name = label(p)
                else:
                    continue
                t = sum(p.cpu_times()[:2])
                used[p.pid] = (name, t)
                total += t
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
        live = sum(t for _, t in used.values())
        dom = 0.0 if last_total is None else (live - last_total) / PERIOD * 100.0
        last_total = live
        f.write(f"{time.time():.1f},{psutil.cpu_percent(None):.1f},{dom:.1f}\n")
        f.flush()
by_name = {}
for name, t in used.values():
    by_name[name] = by_name.get(name, 0.0) + t
with open(OUT / "cpu_proc.csv", "w") as f:
    f.write("process,cpu_s\n")
    for name, t in sorted(by_name.items(), key=lambda kv: -kv[1]):
        f.write(f"{name},{t:.1f}\n")
