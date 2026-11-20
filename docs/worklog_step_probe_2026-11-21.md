# 작업 기록: 실물 응답 측정 도구 (2026-11-21)

브랜치 `feat/step-probe-2026-11-21`(roscamp-nav2fleet main a6265b0에서). 계획 문서 5-3 가의 측정 도구를 만들었다.
실물 로봇에는 붙지 않았다. 다음 주 실물 측정은 `tools/real_pode/README.md`의 "응답 측정 도구" 절을 따른다.

## 만든 것

- `tools/real_pode/cmd_step_probe.py`: 계단 입력과 CSV 기록. 묶음 넷(`linear`, `angular`, `small_rot`, `arc`)을 따로 돌린다.
  계단마다 정지 2 s → 명령 → 정지 2 s → odom 닫힌 고리로 출발 자리 복귀. 거리 한도(출발 자리에서 직선 0.6 m), 다른 cmd_vel
  publisher가 있으면 거절(`cmd_watchdog`은 허용), odom 끊김 정지, 어떻게 끝나든 속도 0 다섯 번, `--dry-run`.
- `tools/real_pode/step_report.py`: 지연(10%), 상승(10→90%), 정상 비, 최대 가속·감속, 데드밴드, 원호 반지름과 좌우 차이. Markdown + PNG.
- `tools/real_pode/path_overlay.py`: `record`(odom, amcl_pose, map→odom tf), `overlay`(구역 지도의 W→W_X→O_X→O 위에 그림, 모서리별 바깥쪽 벗어남 cm).
- `tools/real_pode/scan_info.py`: 실물 스캔 빔 수, 각도, 거리, 주기.
- `tools/real_pode/bringup_only.launch.py`: Nav2 없이 제조사 bringup + `cmd_watchdog`(실물 probe용).
- `tools/real_pode/gz_probe.launch.py`, `gz_step_probe.sh`: Gazebo 한 대(월드·브리지·모델만, Nav2 없음, 경기장 밖 (5, 5)).
- `tools/test/test_step_probe.py`: 합성 자료로 분석 값 복원, 안전 장치(순수 함수 + 도메인 63의 rclpy 가짜 로봇).

## 정한 것과 까닭

- 거리 한도는 누적 주행 거리가 아니라 출발 자리에서의 직선 거리다. 누적으로 보면 `linear` 한 번(4.5 m)이 0.6 m 한도로는 돌 수 없다.
- 복귀는 열린 고리(같은 시간 반대 명령) 대신 odom 닫힌 고리다. 좌우·전후 비대칭이 쌓이지 않는다.
- probe는 로봇에서 돌린다. 운용 때도 Nav2가 로봇 안에서 명령을 내므로 Wi-Fi 지연은 응답에 넣지 않는다.
- Gazebo에서는 `one_pode.launch.py`가 Nav2를 늘 띄우므로(sim 코드는 건드리지 않음) `launch_parts`의 월드·브리지·모델만 쓰는 launch를 따로 두었다.

## 확인한 것

- `ruff check tools/real_pode tools/test/test_step_probe.py`: All checks passed.
- WSL `python3 -m pytest tools/test -q`(도메인 63): `222 passed in 159.32s`. 새 시험 28개 중 셋은 rclpy 가짜 로봇으로
  거절(다른 publisher), 거리 한도 정지, Ctrl+C에서 마지막 다섯 명령이 0인지 본다.
- Gazebo 연기 실행(다른 무거운 작업이 도는 중이라 값은 기준이 아니다):
  `SETS="angular arc" PROBE_ARGS="--repeats 1" tools/real_pode/gz_step_probe.sh`, 처음부터 끝까지 82 s, 두 묶음 모두 종료 0,
  CSV 둘(2050줄, 2141줄)과 `report.md`, PNG 둘, `scan_gz.json`(640빔, 360°, 0.05~12 m, 10.0 Hz)을 썼고 Gazebo는 내려갔다.
  - 원호 R 명령 0.200, 맞춤 0.200(좌우 같음). angular 0.5, 1.0은 정상 비 1.000, 최대 가속 2.00 rad/s².
  - angular 1.5는 정상 비 0.667: Gazebo DiffDrive의 `max_velocity 1.0`, `max_acceleration 2.0`이 각속도에도 걸린다
    (`pode_sim.urdf.xacro`). 시뮬은 1.0 rad/s 넘게 돌지 못한다. 실물 바퀴 상한(100 rpm)으로는 약 5.9 rad/s다.

## Gazebo 측정(기준 판)

2026-11-21 04:22~04:32, 이 PC의 WSL, 도메인 63에서 돌렸다. 다른 무거운 작업은 없었다(Gazebo 비교 묶음이 끝난 뒤에 돌렸다).

- 명령: `OUT=results/2026-11-21/step_probe_gz tools/real_pode/gz_step_probe.sh`(기본 네 묶음, 반복 3번). 처음부터 끝까지 566 s, 네 묶음 모두 종료 0.
- 실시간 비율(CSV의 `t_ros` 경과 / `t_mono` 경과): linear 0.999, angular 0.999, small_rot 1.000, arc 0.999.
- 결과는 `results/2026-11-21/step_probe_gz/`에 있다(`report.md`, PNG 넷, CSV 넷과 메타 JSON, `scan_gz.json`, `run.log`, `launch.log`).
- 끝낼 때 `gz sim` 서버가 SIGINT를 받은 뒤 종료 코드 −11로 죽었다(`launch.log` 끝). 측정이 모두 끝난 다음이라 결과에는 영향이 없다.
- `step_report.py`가 `Mean of empty slice` 경고를 한 번 냈다. w1.50의 상승 시간이 세 번 모두 미도달(NaN)이라 평균이 빈 값이 된 것이고, 표에는 미도달로 적혔다.

| 묶음 | 결과(반복 3번의 평균. v0.05 말고는 세 번이 같은 값) |
|---|---|
| linear v 0.05~0.25 m/s | 정상 비 1.000. 지연 0.013~0.032 s, 상승 0.043~0.104 s. 최대 가속은 0.90(v0.05)에서 2.00 m/s²(v0.25)까지 명령이 클수록 크다 |
| angular ω 0.5, 1.0 rad/s | 정상 비 1.000. 지연 0.047, 0.072 s, 상승 0.200, 0.400 s. 가속 2.00 rad/s² |
| angular ω 1.5 rad/s | 정상 비 0.667, 상승 미도달. 연기 실행 때와 같이 Gazebo DiffDrive의 1.0 rad/s 상한에 걸린다 |
| small_rot ω 0.05~0.30 (v 0, 0.12) | 모두 비 1.000. 가장 작은 0.05 rad/s에도 반응한다(시뮬에는 데드밴드가 없다) |
| arc v 0.10, ω ±0.50 | R 명령 0.200, 궤적 맞춤 0.200, 좌우 차이 0.0 % |
| scan | 640빔, 360°, 0.05~12 m, 10.0 Hz. Gazebo 설정값과 같다 |

- 이 값은 실물과 비교할 기준이다. 시뮬은 상한과 가속 제한 안에서 명령을 그대로 따르므로, 실물 측정에서 차이가 나는 만큼이 실물의 지연, 데드밴드, 좌우 차이다.
- 같은 명령의 세 번 반복은 linear v0.05의 상승(0.044, 0.043, 0.043 s)과 최대 가속(1.00, 0.85, 0.85 m/s²)을 빼면 표에 적힌 자리까지 같았다. Gazebo 판 하나 안에서는 흔들림이 거의 없다는 뜻이고, 실물의 반복 편차를 이것과 견주면 된다.

## 리뷰 반영

코드 리뷰 지적 여섯 건을 시험을 먼저 쓰고 고쳤다(새 시험은 고치기 전 5cb0b47에서 모두 실패).

- 두 번째 probe: 시작 검사를 자기 publisher를 만들기 전에 하고 자기 이름도 빼지 않는다. 도는 중에는 자기 publisher만
  endpoint gid로 뺀다. 같은 이름의 probe가 이미 돌면 거절(종료 3). `--allow`(기본 `cmd_watchdog`)는 그대로 이름으로 본다.
- NaN·inf odom: 하나라도 유한하지 않으면 멈춘다(새 종료 5). 거리 한도도 NaN이면 넘은 것으로 본다. 유한하지 않은 명령은 내지 않는다.
- 속도 0 다섯 번: 먼저 다 보내고 CSV는 뒤에 적는다. 적기 실패는 삼키고, 보내는 동안 SIGINT·SIGTERM을 무시한다.
- `step_report.py`: 10%·90% 교차와 최대 가속을 계단 구간 [t0, t1] 안에서만 찾는다. 못 닿으면 표에 미도달.
- `path_overlay.py`: 경로에서 25 cm 넘게 떨어진 점도 버리지 않는다(몇 개인지 따로 적는다). 표본이 없는 모서리는 `-inf` 대신 `no samples`.
- 돌아오기: 출발 자리까지의 거리가 그 돌아오기에서 가장 가까웠던 거리보다 5 cm 넘게 늘면 멈춘다(새 종료 6). README의
  "한 계단 거리 이상 멀어지지 않는다"는 이 규칙대로 고쳤다.

2차 리뷰(67f838a)의 네 건도 시험을 먼저 쓰고 고쳤다.

- 두 번째 신호: SIGINT·SIGTERM 처리기는 처음 한 번만 KeyboardInterrupt를 내고, 그 뒤와 정리 중(안전 정지로 끝날 때 포함)에는
  무시한다. 속도 0 다섯 번 사이의 무시도 그대로다.
- 쿼터니언: 성분 하나라도 유한하지 않으면 yaw를 NaN으로 두어 종료 5로 멈춘다(예: (0, 0, inf, 1)은 전에는 유한한 yaw가 나왔다).
- 늦게 생긴 publisher: 노드 이름을 `cmd_step_probe_<pid>`로 바꿔 남의 endpoint를 자기 것으로 잡지 않게 했고, 자기 publisher를
  찾은 뒤 첫 명령 전에 한 번 더 세어 있으면 거절(종료 3), 반복의 첫 바퀴에서도 바로 센다.
- `path_overlay.py`: 쓴 표본이 없으면 가장 큰 옆 거리도 `0.0 cm` 대신 `no samples`.
- 고치지 않음: 돌아오기 5 cm 규칙이 odom 잡음에 걸릴 수 있다는 지적(바퀴 odom은 적분 값이라 매끄럽다고 보고 그대로 둔다).

## 확인하지 못한 것

- 실물 토픽 이름은 제조사 `pinky_bringup/bringup.py`(cmd_vel, odom 30 Hz, Twist)와 `real_pode.launch.py`(namespace)에서 읽었다. 실물에서 보지 않았다.
- AMCL의 map→odom이 `/<ns>/tf`로 나온다고 가정했다(`real_pode.launch.py`가 /tf를 namespace 안으로 돌린다). 없으면 overlay는 amcl_pose(드문드문)로 그린다.
