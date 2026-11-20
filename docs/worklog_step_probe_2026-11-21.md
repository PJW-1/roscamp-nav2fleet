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

## 확인하지 못한 것

- 실물 토픽 이름은 제조사 `pinky_bringup/bringup.py`(cmd_vel, odom 30 Hz, Twist)와 `real_pode.launch.py`(namespace)에서 읽었다. 실물에서 보지 않았다.
- AMCL의 map→odom이 `/<ns>/tf`로 나온다고 가정했다(`real_pode.launch.py`가 /tf를 namespace 안으로 돌린다). 없으면 overlay는 amcl_pose(드문드문)로 그린다.
