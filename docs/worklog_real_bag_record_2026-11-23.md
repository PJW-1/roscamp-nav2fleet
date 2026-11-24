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
  - `setsid env --default-signal=INT`로 새 세션에 띄운다. 작업 제어 없이 뒤에서 도는 명령은 SIGINT를 무시하는데, stop이 SIGINT로 bag을 마무리하므로 env로 기본 동작을 되돌린다. 저장 형식은 `-s mcap`으로 정한다.
  - start는 `bags/NAME/*.mcap`이 생길 때까지 기다린다(기본 20 s, `BAG_START_TIMEOUT_S`). rosbag2는 첫 메시지 전에 파일을 연다(pode1에서 확인). 녹화기가 죽거나 시간이 지나면 SIGINT, 그다음 SIGTERM으로 내리고 실패로 끝낸다.
  - `logs/bag.lock`(flock)으로 start와 stop이 겹치지 않는다. 녹화기에는 잠금 fd를 넘기지 않는다.
  - pid 파일 `logs/bag.pid`에는 "PID NAME"을 적는다. 그 PID의 작업 폴더가 이 폴더이고 명령 인자에 `bags/NAME`이 있을 때만 우리 녹화기로 본다(번호가 다시 쓰였을 때 남의 프로세스를 내리지 않는다).
  - stop은 SIGINT 뒤 20 s 기다리고, `metadata.yaml`이 없으면 `ros2 bag reindex`를 안내하며 실패로 끝낸다.
  - 기록 `logs/bag_NAME.log`. 같은 이름의 bag이 있거나 이미 녹화 중이면 시작하지 않는다.
- `tools/test/test_bag_record.py`: 가짜 ros2로 정상 녹화와 마무리, 녹화기가 바로 죽는 경우, bag을 열지 못하는 경우, 잠금, 중복 시작, 번호 재사용, 잘못된 인자, exec 전 확인, 녹화 중 죽은 녹화기를 본다(9개 통과).
- `tools/real_pode/README.md`에 한 줄 추가.

## 시험

pode1(pinky_27a6, 서버 켬, 도메인 12)에서 2026-11-23에 돌렸다.
- start 뒤 `/pode1/state`(담을 토픽)와 `/pode1/foo`(담지 않을 토픽)를 10 Hz로 냈다.
- stop 뒤 `bags/bagtest_1123`에 `metadata.yaml`과 mcap이 있었다. `ros2 bag info`에는 `/pode1/state` 8개만 있고 `/pode1/foo`는 없었다.
- `/pode1/state`는 4 s 동안 약 40개를 냈는데 8개만 담겼다. 녹화기가 새 토픽을 잡는 데 약 3 s 걸렸다. 그래서 판마다 `start.sh`보다 먼저 켠다.
- 시험 bag은 지웠다.
- Codex 검토(2026-11-23)에서 찾은 네 가지를 고친 뒤 pode1에서 다시 돌렸다. 메시지 없이도 3.5 s 만에 시작했고, 두 번째 start는 거절됐다. `/pode1/state` 14개가 담겼고, stop 뒤 녹화 프로세스가 남지 않았다.
  - 네 가지: 녹화기가 실패해도 start가 성공을 알림, 잠금 없음, 시간이 지나도 녹화기가 남음, 다시 쓰인 PID를 우리 것으로 봄.
- 재검토에서 두 가지를 더 고쳤다.
  - start가 녹화기를 exec 전에 확인하면 살아 있는 녹화기를 실패로 보고 버릴 수 있었다. start는 방금 띄운 자식을 프로세스 상태(`ps`, 좀비 제외)로 본다.
  - 녹화 중에 녹화기가 죽으면 stop이 "not recording"으로 성공을 알렸다. 이제 bag에 `metadata.yaml`이 없으면 `ros2 bag reindex`를 안내하고 실패로 끝낸다.
  - 고치지 않은 것: 같은 폴더, 같은 `bags/NAME` 인자를 가진 다른 프로세스가 같은 PID를 받는 경우(현실에서 생기지 않는다). 녹화 그룹에 자식이 남는 경우(`ros2 bag record`는 자식을 만들지 않고, pode1에서 stop 뒤 남은 프로세스가 없었다).
  - 시험 9개 통과, pode1에서 다시 확인했다.
- 2026-11-24 main 병합(`fix/bag-record-followups-2026-11-24`, 사용자 지시): `logs/bag.pid`를 못 쓰면 방금 띄운 녹화기를 INT, 그다음 TERM으로 멈추고 start를 1로 끝낸다. README 실행 순서에 bag start(start.sh 전)와 stop(stop.sh 뒤)을 넣었고, `bags/`는 git에서 뺐다. 합친 main에서 `tools/test` 286개 통과, 3개 건너뜀.
  - Codex 검토(11-24)가 두 가지를 짚었다. 고치지 않았다.
  - 새 경로도 `signal_and_wait`로 그룹 리더만 확인한다. 위 "녹화 그룹에 자식이 남는 경우"와 같은 한계이고, 판단도 같다.
  - 새 시험은 리더 PID만 보고, pid 파일 쓰기가 가짜 `ros2`가 뜨기 전에 실패할 수도 있다. 그래서 이 시험이 보여 주는 것은 "start가 1로 끝나고, 띄운 프로세스가 남지 않는다"까지다. 실제로 녹화 중이던 녹화기를 INT로 마무리하는지는 보여 주지 않는다.

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
