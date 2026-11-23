# 실물 판마다 로봇 안에서 rosbag 남기기 (2026-11-23)

## 왜

11-23 실물 끔 판(`results/2026-11-23`, 브랜치 `data/real-2026-11-23`)에서 pode1이 급행 북쪽 벽에 박혔다.
그 뒤 64 s 동안 보고 위치(AMCL)가 혼자 움직였고, PC 도킹 대역이 그 위치를 믿어 가짜 출고가 기록됐다.
원인 후보(바퀴 헛돎, 복도에서 스캔이 비슷함, AMCL 입자를 다시 뿌리지 않음, 라이더 최소 거리)를 가르려면
odom, scan, cmd_vel, AMCL 입자가 필요했다. 그런데 관제 기록(jsonl)과 로봇 콘솔 로그에는 없었다.
체크리스트의 경로 기록(`path_overlay.py record`, odom/amcl_pose/tf)도 그날 돌리지 않았다.

## 바꾼 것

- `tools/real_pode/bag_record.sh` 추가: `start NS NAME`, `stop`.
  - 로봇 안에서 `ros2 bag record -e "^/NS/(scan|odom|cmd_vel|amcl_pose|particle_cloud|plan|tf|tf_static|state)$"`를 `bags/NAME`에 쓴다. Wi-Fi가 끊겨도 남는다.
  - `SUPER=1 source env.sh`: 디스커버리 서버를 쓸 때도 전체 그래프를 보고 토픽을 찾는다.
  - `set -m`과 `setsid`로 띄우고, stop은 그룹에 SIGINT를 보내 bag을 마무리한다. 비동기 명령은 작업 제어가 꺼져 있으면 SIGINT를 무시하기 때문이다.
  - pid 파일 `logs/bag.pid`, 기록 `logs/bag_NAME.log`. 같은 이름의 bag이 있거나 이미 녹화 중이면 시작하지 않는다.
- `tools/real_pode/README.md`에 한 줄 추가.

## 시험

pode1(pinky_27a6, 서버 켬, 도메인 12)에서 2026-11-23에 돌렸다.
- start 뒤 `/pode1/state`(담을 토픽)와 `/pode1/foo`(담지 않을 토픽)를 10 Hz로 냈다.
- stop 뒤 `bags/bagtest_1123`에 `metadata.yaml`과 mcap이 있었다. `ros2 bag info`에는 `/pode1/state` 8개만 있고 `/pode1/foo`는 없었다.
- `/pode1/state`는 4 s 동안 약 40개를 냈는데 8개만 담겼다. 녹화기가 새 토픽을 잡는 데 약 3 s 걸렸다. 그래서 판마다 `start.sh`보다 먼저 켠다.
- 시험 bag은 지웠다.

## 쓰는 법 (판마다)

```bash
~/nav2fleet_real/bag_record.sh start pode1 <판 이름>   # start.sh보다 먼저
~/nav2fleet_real/start.sh pode1 0.12 0.10 1.5708
...
~/nav2fleet_real/stop.sh
~/nav2fleet_real/bag_record.sh stop
scp -r pinky@<pode1 주소>:nav2fleet_real/bags/<판 이름> <PC 결과 폴더>/
```

## 남은 것

- 실제 판 길이(약 10분)에서 bag 크기와 로봇 CPU 부담을 아직 재지 않았다. 다음 판에서 vmstat와 함께 본다.
- 기준판 배포(nav2fleet `deploy.sh`)에는 이 파일이 없다. 측정 도구처럼 `scp`로 로봇 `~/nav2fleet_real/`에 따로 넣는다.
