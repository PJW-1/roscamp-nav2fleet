# AMCL 재생 비교

실물에서 녹화한 bag(`bag_record.sh`)으로 AMCL을 PC에서 다시 돌려, 지도마다 위치 추정 오차를 줄자 정답과 비교한다. 로봇은 쓰지 않는다.

## 순서

```bash
source /opt/ros/jazzy/setup.bash
cd tools/real_pode/amcl_replay

# 1. live AMCL의 map → odom만 뺀 bag을 만든다(scan, odom, tf, tf_static만 남김)
python3 amcl_replay.py prep ~/ros2camp/real_data/2026-11-24/pode2/1124_amcl_base_pode2 /tmp/replay/filtered

# 2. 지도 2개 × 자리 2곳을 동시에 돌린다(도메인 101부터 4개를 씀)
bash run_2026-11-24.sh /tmp/replay/filtered /tmp/replay/runs 1 101

# 3. 원래 AMCL(live)과 재생 결과를 줄자 정답과 비교한 표
python3 amcl_replay.py score --bag ~/ros2camp/real_data/2026-11-24/pode2/1124_amcl_base_pode2 \
  --truth truth_2026-11-24.json /tmp/replay/runs/*.json
```

한 판만 돌릴 때는 `run_one.sh DOMAIN BAG MAP_YAML LABEL START_OFFSET_S DURATION_S X Y YAW_RAD TIMES OUT_JSON`이다.

## 파일

| 파일 | 하는 일 |
|---|---|
| `amcl_replay.py` | `prep`(bag 거르기), `probe`(재생 중 정한 시각의 `map → base_footprint`와 `amcl_pose` 궤적을 JSON으로), `score`(표) |
| `replay.launch.py` | `map_server_fine`, `amcl`, lifecycle 관리자만 sim time으로 띄운다. AMCL 값은 `pode_nav_config/params/nav2_params.yaml`의 `amcl` 절 그대로 |
| `run_one.sh` | 한 판: 자기 도메인(`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`)에서 launch, probe, `ros2 bag play --clock`을 돌리고 끝나면 자기 프로세스 그룹만 내린다 |
| `run_2026-11-24.sh` | 11-24 bag의 두 구간(Q2, O_IN)을 지도별로 N번씩 |
| `truth_2026-11-24.json` | 11-24 줄자 정답과 잰 시각 |
| `run_2026-11-26.sh` | 11-26 bag(1126_real) 19:01~19:14 구간(Q2 두 번, O_IN)을 한 도메인에서 N번 차례로. 결과 `results/2026-11-26/amcl_replay/` |
| `truth_2026-11-26.json` | 11-26 줄자 정답(라이다 가운데)과 잰 시각 |

## 알아 둘 것

- **초기 자세:** 로봇의 `start.sh`처럼 AMCL 파라미터 `initial_pose`로 주고 공분산은 0이다. 모든 지도가 같은 출발 자세를 받는다. O_IN 구간은 사용자가 11-24 21:43에 P-1에서 준 initialpose 뒤 live AMCL의 첫 값(0.116, 0.142, 89.7°)을 쓴다.
- **도메인:** 판마다 다른 `ROS_DOMAIN_ID`를 쓰지만 이미 쓰이는지 보지 않는다. 같은 PC에서 여러 묶음을 동시에 돌릴 때는 도메인 범위가 겹치지 않게 첫 도메인을 따로 준다(Gazebo·관제 시험이 쓰는 도메인도 피한다).
- **tf_static:** bag 맨 앞에 한 번만 있어서 구간 중간부터 틀면 빠진다. `probe`가 bag에서 읽어 직접 낸다.
- **정답 좌표계:** 줄자는 벽에서 라이다 가운데까지 쟀고, 정답은 벽이 도면 자리(x 0, 1.98, y 0, 0.98)에 있다고 보고 바꾼 값이다(`results/2026-11-24/measure_2026-11-24.md` 4단계). 관제 배치와 Nav2 goal이 모두 이 좌표계를 쓴다.
- **같은 입력이면 같은 값:** 11-24 bag에서 같은 지도로 5번 돌린 결과가 모두 같았다(AMCL 난수가 판마다 같다). 지도 하나에 한 번이면 된다.
- **lidar 거리 보정:** `prep --range-scale A --range-offset B`는 "읽은 값 = A × 실제 + B"를 거꾸로 풀어 scan을 고친 bag을 만든다(pode2 11-24: 0.958, 0.005).
- **라이다 자리로 채점:** 정답이 라이다 가운데면 `score --sensor-x -0.017`(base_footprint → rplidar 앞뒤 거리)로 재생 자세를 라이다로 옮겨 잰다. 기본 0은 base_footprint 그대로다(11-25 표는 이 값 없이 냈다).
