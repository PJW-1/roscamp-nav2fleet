# 11-23(월) 학원 실물 시험과 측정 체크리스트

판: 하루 종일 **nav2fleet 기준판**(태그 `rpp-baseline-2026-11-21`, b9e86c5) 하나로 한다. 측정 도구 세 파일만 pode-drive에서 가져온다.

| 순서 | 무엇 | 시간 | 이걸로 정하는 것 |
|---|---|---|---|
| 0 | 배포와 빌드 | 30분 | |
| 1 | 반응 측정과 scan | 45분 | Isaac 무작위화 범위, 재학습 |
| 2 | 두 대 시험(discovery 끔/켬) | 1~1.5시간 | 팀 PR 병합, discovery 기본값, LOCALIZING 수정을 꺼낼지, 실물 ETA 오차 |
| 3 | AMCL 오차 | 30분~1시간 | 학습 위치 잡음, relocalize를 꺼낼지 |
| 4 | 지도 줄자(시간 남으면) | 30분 | 2 cm 넘게 다른 곳만 고침 |
| 5 | 결과 올리기 | 15분 | CLI 분석 |

꼭 할 것은 0~2. 3은 줄여도 되고 4는 미뤄도 된다.

## 진행 기록 (11-23 20:20 CLI가 채움, 오늘은 여기까지)

자세한 결과와 근거는 GitHub pode-drive 브랜치 `data/real-2026-11-23`의 `results/2026-11-23/measure_2026-11-23.md`와, 같은 내용의 Drive `실물_측정_결과_2026-11-23.md`에 있다. 아래 각 단계의 기록표도 그 값으로 채웠다.

| 순서 | 상태 |
|---|---|
| 0 | 끝. PC와 두 로봇 모두 b9e86c5로 배포했다. |
| 1 | 안 함. 사용자 결정 B로 다음에 한다(W 입구 밀림의 원인을 가르려고 실물 회전 반응을 잰다). |
| 2 | 끝. 판 4개: 켬 `1123_ds_on`, 끔 `1123_ds_off`, 끔 다시 `1123_ds_off2`(16:59, 로봇 rosbag 첫 판), 예비 `1123_pode2_ds_on`(pode2 혼자). **통과 못 함.** 사용자 결정: 디스커버리 기본값은 서버 켬, 디스커버리 비교는 여기서 멈춘다. |
| 3 | 11-24로 미룸(사용자 결정, 11-23 20시). 3-추가도 같이 한다. 할 때는 pode1 혼자, 서버 켬, rosbag을 켜고 navigate_to_pose로 node마다 보낸다. |
| 4 | 끝(16:57). 사용자 줄자 값과 표시 화면(`tape_marks_2026-11-23.png`). 11-23 밤 사용자 결정으로 관제 지도, Gazebo 세계, 시뮬 지도에 반영했다(진열대 A 서쪽 끝 0.70, 남쪽 서쪽 진열대 서쪽 끝 0.65, HALL 띠 끝 0.645; pode-drive main 333a037, Gazebo 전후 두 판 모두 8/8). 로봇의 AMCL·costmap 지도(`tools/real_pode/maps`)는 11-24 스캔 확인 뒤에 정한다. |
| 5 | 진행 중. 위 브랜치에 push했다. 반응 측정과 path_overlay 경로 기록은 하지 않아 없다. pode2의 끔 다시 판 파일은 pode2가 응답하지 않아 아직 못 가져왔다. |

계획과 달라진 점:

- 역할: 사용자 결정으로 CLI가 로봇 접속, 배포, 주행, 감시를 맡았다. 사용자는 로봇 배치와 비상 정지를 맡았다.
- 순서: 켬 판을 먼저 돌렸다. 로봇은 거의 동시에 띄우지 않고 pode1이 IDLE이 된 뒤 pode2를 띄웠다(약 30 s 간격).
- 끔 판에서 원인을 가를 기록(odom, scan, cmd_vel)이 없어, 끔 다시 판부터 로봇 안 rosbag(`bag_record.sh`)을 켰다. 앞으로 모든 실물 판에서 켠다.

## 준비물

- 줄자, 바닥 테이프, 두 로봇 배터리 충전
- 반응 측정용 빈 바닥: 경기장 밖, 사방 1 m 이상
- 로봇: pode1(27a6), pode2(07d7). 아래 `<pode1 주소>`, `<pode2 주소>`를 실제 주소로 바꿔 쓴다
- 로봇 옆에 들어 올릴 사람(비상 정지)

## 역할 (CLI가 학원 PC에서 감시할 때)

- **본인:** 로봇 배치, start/stop, 측정 실행, 줄자, 눈으로 볼 것, 비상 정지
- **CLI:** 읽기 전용 감시와 분석만. deploy.sh, 로봇 명령, 로봇 접속, cmd_vel 발행은 하지 않는다.

CLI에 줄 첫 말:

```text
오늘 학원에서 실물 시험과 측정을 한다. 너는 이 PC에서 읽기 전용으로 감시하고 분석만 해. deploy.sh, 로봇 명령, 로봇 접속, cmd_vel 발행은 하지 마.
- 「실물_측정_체크리스트_2026-11-23.md」 순서대로 간다. 단계마다 내가 끝났다고 하면 결과 파일이 제대로 생겼는지 확인해.
- 반응 측정 CSV가 오면 pode-drive의 tools/real_pode/step_report.py로 바로 보고서를 만들고, Gazebo 기준값(results/2026-11-21/step_probe_gz/report.md)과 크게 다른 값을 알려 줘.
- 두 대 시험 중에는 관제 로그와 record 파일을 보며 ERROR, BLOCKED, LOCALIZING, dock FAILED가 나오면 바로 알려 줘. 판이 끝나면 통과 기준을 판정해.
- 결과는 pode-drive의 results/2026-11-23/에 모으고, 끝나면 data/real-2026-11-23 브랜치로 push해.
```

## 0. 배포와 빌드 (30분)

PC, nav2fleet 저장소 루트에서:

```bash
cd ~/ros2camp/nav2fleet_ws/src/roscamp-nav2fleet
git status                               # 고치던 파일이 없어야 한다
git fetch --tags origin
git checkout rpp-baseline-2026-11-21     # 태그로 옮긴다(분리된 HEAD)
git log -1 --oneline                     # b9e86c5인지 확인
tools/real_pode/deploy.sh pinky@<pode1 주소>
tools/real_pode/deploy.sh pinky@<pode2 주소>
cd ~/ros2camp/nav2fleet_ws && colcon build && source install/setup.bash
```

로봇 두 대 각각: `cd ~/nav2fleet_ws && colcon build`

측정 도구 세 파일(기준판에는 없다):

```bash
git clone https://github.com/PJW-1/roscamp-pode-drive ~/pode-drive   # 이미 있으면: cd ~/pode-drive && git pull
cd ~/pode-drive/tools/real_pode
scp cmd_step_probe.py scan_info.py bringup_only.launch.py pinky@<pode1 주소>:~/nav2fleet_real/
scp cmd_step_probe.py scan_info.py bringup_only.launch.py pinky@<pode2 주소>:~/nav2fleet_real/
mkdir -p ~/pode-drive/results/2026-11-23
```

막히면: 15분 안에 안 풀리면 원인을 적어 두고 다음 단계로 간다.

## 1. 반응 측정과 scan (45분)

경기장 밖 빈 바닥. Nav2 없이 한다. **두 터미널 모두 discovery server 없이** 띄운다(서버가 없으면 로봇 안의 노드끼리도 못 찾는다).

로봇 터미널 1:

```bash
DISCOVERY_SERVER=off source ~/nav2fleet_real/env.sh
ros2 launch ~/nav2fleet_real/bringup_only.launch.py namespace:=pode1
```

로봇 터미널 2:

```bash
DISCOVERY_SERVER=off source ~/nav2fleet_real/env.sh
ros2 topic list | grep pode1             # /pode1/cmd_vel, /pode1/odom, /pode1/scan이 있는지 먼저 본다
cd ~/nav2fleet_real && mkdir -p probe
python3 cmd_step_probe.py --ns pode1 --set linear --dry-run
python3 cmd_step_probe.py --ns pode1 --set linear    --out probe/step_linear.csv
python3 cmd_step_probe.py --ns pode1 --set angular   --out probe/step_angular.csv
python3 cmd_step_probe.py --ns pode1 --set small_rot --out probe/step_small_rot.csv
python3 cmd_step_probe.py --ns pode1 --set arc       --out probe/step_arc.csv
python3 scan_info.py --ns pode1 --seconds 10 --out probe/scan_real.json
```

- 묶음 사이에 로봇을 빈 곳 가운데로 다시 옮겨도 된다.
- 0.6 m 넘게 움직이면 스스로 멈춘다. Ctrl+C로 언제든 멈춘다.
- 배터리 잔량을 적어 둔다.

pode2는 namespace와 `--ns`를 pode2로 바꿔 **linear와 arc만** 한다.

PC로 가져오기:

```bash
scp -r pinky@<pode1 주소>:~/nav2fleet_real/probe ~/pode-drive/results/2026-11-23/step_probe_real_pode1
scp -r pinky@<pode2 주소>:~/nav2fleet_real/probe ~/pode-drive/results/2026-11-23/step_probe_real_pode2
```

## 2. 두 대 시험: discovery 끔(A판), 켬(B판) (1~1.5시간)

11-18 run8과 같은 조건이다. 주문 4건(R-001~R-004), 도킹은 PC 대역이 3 s 기다린다. 두 판은 discovery만 다르다.

| | A판 | B판 |
|---|---|---|
| 판 이름 | `1123_ds_off` | `1123_ds_on` |
| PC 서버 | 안 켠다 | 먼저 `tools/real_pode/discovery_server.sh`를 켜 둔다 |
| PC 터미널 | `DISCOVERY_SERVER=off source tools/real_pode/pc_env.sh` | `source tools/real_pode/pc_env.sh` |
| 로봇 명령 앞에 | `DISCOVERY_SERVER=off` | 없음 |

판마다 순서:

1. 로봇을 P-1(pode1, 0.12, 0.10), P-2(pode2, 0.38, 0.10)에 북쪽을 보게 놓는다.
2. 로봇 두 대를 거의 동시에 띄우고 시간을 잰다(A판은 앞에 `DISCOVERY_SERVER=off`를 붙인다):

   ```bash
   PEER_ROBOTS=pode1,pode2 DOCK_STUB=1 ~/nav2fleet_real/start.sh pode1 0.12 0.10 1.5708
   PEER_ROBOTS=pode1,pode2 DOCK_STUB=1 ~/nav2fleet_real/start.sh pode2 0.38 0.10 1.5708
   ```

3. **모두 보일 때까지 걸린 시간:** PC에서 두 로봇이 다 답할 때까지(B판은 이 터미널을 `SUPER=1 source tools/real_pode/pc_env.sh`로 연다):

   ```bash
   ros2 topic echo /pode1/state --qos-reliability best_effort --once --timeout 10
   ros2 topic echo /pode2/state --qos-reliability best_effort --once --timeout 10
   ```

4. 로봇마다 CPU 기록을 켠다(판 끝에 Ctrl+C): `vmstat 1 | tee ~/nav2fleet_real/logs/cpu_<판 이름>.txt`
5. 관제를 띄운다:

   ```bash
   mkdir -p ~/real_runs
   ros2 launch iponic_bringup pc.launch.py use_fake:=true use_sim_clock:=false \
       fake_robots:=none robots:="pode1 pode2" map_file:=v7 scenario:=real_two_orders \
       monitor:=false dock_wait_robots:="pode1 pode2" record:=$HOME/real_runs/<판 이름>.jsonl
   ```

   주문은 관제가 뜬 뒤 약 20, 40, 60, 80 s에 들어온다.
6. **discovery 패킷 수:** 관제를 띄운 직후 60 s 동안(Wi-Fi 장치 이름은 `ip a`로 확인):

   ```bash
   sudo timeout 60 tcpdump -i <Wi-Fi 이름> -n udp port 10400 2>/dev/null | wc -l
   ```

7. **경로 기록(pode1):** 다른 PC 터미널에서. A판은 `DISCOVERY_SERVER=off source ~/ros2camp/nav2fleet_ws/src/roscamp-nav2fleet/tools/real_pode/pc_env.sh`, B판은 같은 파일을 `SUPER=1 source`로 먼저 연다:

   ```bash
   cd ~/pode-drive
   python3 tools/real_pode/path_overlay.py record --ns pode1 --out results/2026-11-23/path_pode1_<판 이름>.csv
   ```

8. 주문 4개가 끝나면 로봇마다 `~/nav2fleet_real/stop.sh`, 관제는 Ctrl+C. CPU와 경로 기록도 Ctrl+C.

눈으로 볼 것:

- 급행 북쪽 벽 여유(R-001 때 가까이서), `O_X`에서 기다리는 장면
- 띄운 직후 위치가 놓은 자리와 맞는지
- 두 대가 마주치면 낮은 우선순위가 물러났다 다시 가는지
- PC 로그의 `dock WORK ok` / `dock OUTBOUND ok`, `another node also serves` 오류가 없는지
- Q2로 가는 구간의 도리도리

**통과 기준(팀 PR 병합 조건):**

- 두 판 모두 주문 4개 출고 완료
- ERROR 격리 0, 손으로 옮긴 적 0
- BLOCKED가 나와도 스스로 풀림
- 급행에서 벽에 닿지 않음

통과하고 PR(`feat/fleet-eta-2026-11-23`)의 시험이 모두 통과로 적혀 있으면 병합한다. 실패하면 병합하지 않고 기록을 올린다.

기록표:

| 판 | 모두 보일 때까지 s | 패킷 수(60 s) | 주문 4개 | 격리 | BLOCKED | 눈으로 본 LOCALIZING | 이상한 점 |
|---|---|---|---|---|---|---|---|
| 1123_ds_off | 62 (pode1 launch 12:54:23 → 관제가 pode2 첫 상태 받음 12:55:25) | 안 잼 | 기록 4/4, 실제 3 (pode1 혼자. R-004는 벽에 박힌 뒤 위치만 가서 가짜로 출고) | 2 (pode2 NAV_ABORTED, pode1 FAILED) | BLOCKED_WAIT 2 (첫째는 5 s 뒤 풀림, 둘째는 우회 뒤 격리로 끝남) | pode1이 pode2 합류 중 38.6 s 다시 LOCALIZING | pode2 시작 직후 격리. pode1이 4번째 급행 서쪽 통과 중 O_X 동쪽 북쪽 벽에 박힘(사용자 목격), 그 뒤 64 s 위치를 잃음 |
| 1123_ds_on | 46 (pode1 launch 12:04:13 → 12:04:59) | 안 잼 | 4/4 | 0 | 0 (LOCAL_YIELD 2, 스스로 풀림) | 시작 때만 | R-004에서 pode2가 W 입구 동쪽 벽에 닿음 |
| 1123_ds_off2 (추가, 16:59, 끔) | 63 (pode1 launch 16:59:48 → 17:00:51) | 안 잼 | 4/4 (pode1 혼자) | 1 (pode2 NAV_ABORTED 4번. 첫 번은 LOCALIZING 중이라 시도로 세지 않음) | 0 (LOCAL_YIELD 4, 스스로 풀림) | pode1이 pode2 합류 중 55.5 s 다시 LOCALIZING | pode2 시작 직후 격리. 합류 중 로봇 안 녹화기가 받은 pode1의 odom과 scan 간격이 최대 0.6 s 벌어짐(보내는 쪽이 멈췄는지 받는 쪽이 늦었는지는 모름). 벽 충돌 보고 없음 |
| 1123_pode2_ds_on (추가, 11:10, 켬, pode2 혼자 예비) | 혼자라 해당 없음 | 안 잼 | 4/4 | 0 | 0 (LOCAL_YIELD 2, 스스로 풀림) | 시작 때만 | 없음 |

- "눈으로 본 LOCALIZING" 칸은 관제 기록에서 센 값이다.
- 통과 기준: **통과 못 함.** 끔 판은 격리 2와 급행 벽 충돌, 끔 다시 판은 격리 1, 켬 판은 W 입구 벽 접촉이 있었다. 팀 PR `feat/fleet-eta-2026-11-23`은 이날 팀 GitHub에 없었고 병합하지 않는다.
- 손으로 옮긴 적: 기록에는 없다. 사용자 확인이 필요하다.
- pode1 CPU(vmstat, launch부터 판 끝까지 평균 / 합류 구간 평균): 켬 74 / 81 %, 끔 76 / 88 %, 끔 다시 81 / 92 %. 끔 다시 판은 rosbag 녹화도 같이 돌았다.
- 켬 판 ETA 오차와 구간 끝 정지는 결과 문서 부록에 있다(전체 평균 -0.79 s, 출발 -2.29 s, 정지 2/28).

LOCALIZING 횟수, CPU 최고값, ETA 오차는 기록 파일로 CLI가 센다.

## 3. AMCL 오차 (30분~1시간)

**로봇을 손으로 들어 옮긴 직후 값은 쓰지 않는다.** 들어 옮기면 odom이 모르니 AMCL도 따라오지 못한다. 로봇이 스스로 달려와 선 자리에서 잰다.

방법:

1. node마다 바닥에 테이프로 십자를 붙인다(좌표는 아래 표).
2. 로봇을 그 node로 보낸다. 두 대 시험 중 대기 자리에 섰을 때 재도 되고, 시험 뒤 RViz의 2D Goal Pose로 보내도 된다.
3. 서면 줄자로 **로봇 중심(두 바퀴 사이 가운데)이 십자에서 동쪽(+x), 북쪽(+y)으로 몇 cm인지** 잰다. 방향은 테이프 선과 비교해 눈으로 대략 적는다.
4. 같은 순간의 AMCL 값을 적는다: `ros2 topic echo /pode1/amcl_pose --once` (x, y와 orientation의 z, w를 적는다. yaw는 CLI가 바꾼다)
5. 10 s 뒤 한 번 더 읽는다(정지 상태 흔들림).

| node | x | y | 방향 |
|---|---|---|---|
| P-1 | 0.12 | 0.10 | 북(90°) |
| Q2 | 1.66 | 0.15 | 동(0°) |
| O_IN | 0.49 | 0.625 | 110° |
| O_X | 0.71 | 0.868 | 서(180°) |
| W(도크) | 1.86 | 0.495 | 북(90°) |

시간이 없으면 Q2, O_X, O_IN 세 곳만 두 번씩 잰다(HALL 주행 뒤 한 번, 급행 지난 뒤 한 번).

기록표:

| 로봇 | node | 언제(시작/HALL 뒤/급행 뒤) | 실제 dx cm | 실제 dy cm | 방향 눈대중 | AMCL x | AMCL y | AMCL z, w | 10 s 뒤 x, y |
|---|---|---|---|---|---|---|---|---|---|
| pode1 | P-1 | 시작 | | | | | | | |
| pode1 | Q2 | HALL 뒤 | | | | | | | |
| pode1 | W | | | | | | | | |
| pode1 | O_X | 급행 뒤 | | | | | | | |
| pode1 | O_IN | | | | | | | | |

(이 순서로 잰다. 테이프 십자는 벽 안쪽 면에서 잰다: P-1 서쪽 12, 남쪽 10 / Q2 서쪽 166, 남쪽 15 / W 동쪽 12, 남쪽 49.5 / O_X 서쪽 71, 북쪽 11.2 / O_IN 서쪽 49, 남쪽 62.5 cm)

### 3-추가. 벽에 붙어 가는 원인 가르기 (11-23 추가, 15분)

고정 경로에서 로봇이 어떨 땐 왼쪽, 어떨 땐 오른쪽 벽에 붙어 간다. 위치 추정 문제인지 컨트롤러/회전 문제인지 가른다. 설정은 바꾸지 않는다(오늘은 frozen 기준선).

1. **RViz footprint와 경로 비교**: 로봇이 실제로 벽에 붙어 갈 때 RViz에서 footprint가 경로 위에 있는지 본다.
   - RViz에서는 경로 위, 실제는 벽 쪽 → AMCL 또는 지도 오차
   - RViz에서도 경로를 벗어남 → 컨트롤러 또는 회전 지연
2. **lidar TF**: 경기장 안 빈 자리에서 RViz로 제자리 한 바퀴 돌린다(2D Goal Pose로 방향만 바꾸기). scan 점이 지도 벽과 계속 겹치면 정상, 돌 때 벽이 밀리거나 겹쳐 보이면 lidar 장착 각도/위치(TF)를 의심한다.
3. **급행 구간 안 scan**: 로봇이 급행 구간(폭 0.18 m) 안에 섰을 때 RViz에서 양옆 벽이 scan에 찍히는지 본다. 안 찍히면 lidar 최소 거리 한계다(1번 scan_info.py의 range_min과 같이 본다).
4. AMCL 보정 간격 참고: 지금 `update_min_d` 0.15 m라 0.12 m/s에서 약 1.25 s에 한 번 보정한다. 바꾸는 것은 오늘 결과를 본 뒤 정한다.

| 확인 | 로봇 | 어디서 | 결과(겹침/벗어남/찍힘 여부) | 메모 |
|---|---|---|---|---|
| footprint vs 경로 | | | | |
| 제자리 한 바퀴 scan | | | | |
| 급행 안 양옆 벽 scan | | | | |

## 4. 지도 줄자 (시간 남으면, 30분)

벽 안쪽 기준이다. 지도 값과 2 cm 넘게 다른 곳만 적는다.

| 잴 것 | 지도 값(m) | 실측(m) | 메모 |
|---|---|---|---|
| 경기장 가로(서쪽 벽 → 동쪽 벽) | 1.98 | 1.98 | |
| 경기장 세로(남쪽 벽 → 북쪽 벽) | 0.98 | 0.98 | |
| 남쪽 통로 폭(남쪽 벽 → 가운데 진열대 남쪽 면) | 0.365 | 0.36 | 검정(아래) |
| 가운데 통로 폭(가운데 진열대 북쪽 면 → 진열대 A 남쪽 면) | 0.365 | 0.36 | 검정(위) |
| 가운데 서쪽 진열대 x 범위 | 0.68 ~ 0.99 | 0.65 ~ (동쪽 끝 안 잼) | 서쪽 끝 -3 cm, 2 cm 넘게 다름 |
| 가운데 동쪽 진열대 x 범위 | 1.25 ~ 1.56 | (서쪽 끝 안 잼) ~ 1.56 | 동쪽 끝은 같음 |
| 진열대 A(북쪽) x 범위 | 0.63 ~ 1.56 | 0.70 ~ 1.56 | 서쪽 끝 +7 cm, 2 cm 넘게 다름 |
| (추가) 급행 폭(북쪽 벽 → 진열대 A 북쪽 면) | 0.197 | 0.198 | 빨강 |
| (추가) 진열대 A 동쪽 끝 → 동쪽 벽 | 0.42 | 0.42 | 초록 |
| (추가) 가운데 동쪽 진열대 동쪽 끝 → 동쪽 벽 | 0.42 | 0.42 | 초록 |
| (추가) 가운데 두 진열대 사이 틈 | 0.26 | 0.25 | 주황 |
| (추가) 진열대 A 서쪽 끝 → 서쪽 벽 | 0.63 | 0.70 | 파랑 |
| (추가) 가운데 서쪽 진열대 서쪽 끝 → 서쪽 벽 | 0.68 | 0.65 | 보라 |

- 11-23 16:57 사용자 측정. 색은 사용자가 Gazebo 화면에 표시한 것이다(`tape_marks_2026-11-23.png`). 진열대 끝 x는 서쪽 벽(x 0)과 동쪽 벽(x 1.98)에서 잰 거리로 계산했다.
- 색과 벽의 짝은 사용자가 표시한 화면으로 확인했다(11-23 20시): 파랑은 진열대 A 서쪽 끝, 보라는 가운데 서쪽 진열대 서쪽 끝에서 서쪽 벽까지다.
- 이 표의 지도 값은 도면(Gazebo) 값이다. 실제로 쓰는 두 지도는 더 작다: AMCL 지도는 경기장 1.955 x 0.955, 급행 0.135, 진열대 A 서쪽 끝까지 0.615이고, costmap 지도는 1.95 x 0.95, 급행 0.150, 0.60이다. 자세한 것은 결과 문서의 "줄자 측정"에 있다.
- 11-23 밤 사용자 결정으로 관제 지도, Gazebo 세계, 시뮬 지도에 반영했다(진열대 A 서쪽 끝 0.70, 남쪽 서쪽 진열대 서쪽 끝 0.65, HALL 띠 끝 0.645; pode-drive main 333a037, Gazebo 전후 두 판 모두 8/8). 로봇의 AMCL·costmap 지도(`tools/real_pode/maps`)는 11-24 스캔 확인 뒤에 정한다.

## 5. 결과 올리기 (15분)

모을 것(`~/pode-drive/results/2026-11-23/` 아래):

- `step_probe_real_pode1/`, `step_probe_real_pode2/`
- `path_pode1_1123_ds_off.csv`, `path_pode1_1123_ds_on.csv`
- `~/real_runs/1123_ds_*.jsonl` 복사본
- 로봇의 `~/nav2fleet_real/logs/cpu_*.txt`(scp로 가져온다)
- 이 문서의 표를 채운 `measure_2026-11-23.md`

```bash
cd ~/pode-drive
git checkout -b data/real-2026-11-23
git add results/2026-11-23
git commit -m "results: real measurements 2026-11-23"
git push -u origin data/real-2026-11-23
```

push가 안 되면 폴더를 압축해 Drive에 올린다.

11-23 결과(20:10): `data/real-2026-11-23`에 push했다. 들어간 것은 관제 기록 4판(`runs/`), 로봇 launch 로그와 vmstat(켬·끔 두 대, 끔 다시 pode1), 끔 다시 판 pode1 rosbag(zstd 압축 8.1 MB), 줄자 표시 화면, 분석 스크립트, 켬 판 ETA와 정지 분석(`eta/`), `measure_2026-11-23.md`다. 없는 것은 `step_probe_real_*`(1단계 안 함), `path_pode1_*.csv`(경로 기록 안 함), pode2의 끔 다시 판 bag·로그·vmstat(pode2 응답 없음)다. CPU는 `cpu_*.txt` 대신 `vmstat_*.txt`로 남겼다.

그다음 집 CLI에:

```text
data/real-2026-11-23 브랜치의 results/2026-11-23을 분석해: 반응 측정 보고서(Gazebo 기준값과 비교, Isaac 무작위화 범위 밖인 것 표시), scan 비교, 경로 겹쳐 그리기, 두 판의 LOCALIZING 횟수와 시간·CPU 최고값·패킷 수, 실물 ETA 오차(eta_cause 도구), AMCL 오차 표, 지도 차이. 보고서는 docs/report_2026-11-23_real.md로 써서 같은 브랜치에 push해.
```
