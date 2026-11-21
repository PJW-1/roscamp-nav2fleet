# 작업 기록: 0.12 m/s에서 네 컨트롤러 비교 (2026-11-22)

RPP, MPPI, RL 혼자, RL+shim을 같은 Gazebo 경기장에서 0.12 m/s 상한으로 비교했다. RL 정책은 0.12로 다시 학습한
`store_arena_4b`(v_max 0.12, 앞보기 0.40 m, 창 20, 출처 `ppo_k/c2/model_998.pt` @ 95e2054)이다.
원본 표, 판별 값, 그림은 `results/2026-11-22/compare_012/`에 있다.

## 기준과 만든 것

- 기준: `feat/retrain-012-2026-11-22` 8064b64(`integrate/week2` 1ca7ae0 + 경기장 브랜치 + 새 정책). 작업 브랜치
  `feat/compare-012-2026-11-22`.
- 826f256: parity 시험의 정책 사양 검사가 v_max 0.25를 박아 두고 있던 것을 속도 원천(`nav_speed.yaml`)에서 읽게 했다.
- b4d17b9: MPPI `time_steps` 40 → 83(아래 결정 3), 그 거리를 지키는 시험, 표의 `FollowPathRLShim` 이름 "RL+shim".
- 1250e45: `report.py`의 궤적 그림이 시나리오가 하나뿐인 설정(Q2 → E-3)에서 죽던 것을 고쳤다.
- ac37a2c, 4f1e5f9, 487ba85: sim에서 `nav_max_speed`를 0.12보다 낮추면 MPPI의 `vx_max`만 내려가 내다보는 거리가
  줄던 것(Codex 지적)을 고쳤다. 이제 `time_steps`가 같은 비율로 늘어난다(0.12에서는 그대로 83). 같은 김에
  덮어쓰기(`nav_overrides`) dict를 복사해서 합쳐, 호출한 쪽의 dict가 바뀌지 않게 했다.

## 시험

- `pode_rl_controller` colcon 시험 20개 실패 0. parity: C++와 torch 차이 2.62e-06(허용 1.01e-05), ORT python과 0,
  관측 차이 4.47e-08, 행동 차이 3.58e-08, 추론 한 번 6.7 µs(PC).
- pytest `sim/fleet_gz_sim/test`, `tools/test/test_nav_bench.py`, `pode/pode_nav_config/test`: 748 통과.
- 정책의 v_max(0.12)와 속도 원천이 다르면 FollowPathRL이 남기는 ERROR: 다섯 launch 로그 모두 0줄. 로그의 ERROR 줄은
  goal 실패(Controller patience exceeded, Goal failed)와 끝낼 때의 "process has died"뿐이다.
- ruff: 고친 파일은 깨끗하다(sim의 다른 파일 다섯 곳에 원래 있던 줄 길이 경고는 그대로).

## 실행 조건

- 묶음은 둘이다. Q2 → E-3(장애물 없음, 칸마다 3판, 02:54~03:02)과 본 비교 O_IN → M_E(시나리오 넷, 칸마다 6판,
  03:05~04:28). 잠금은 시나리오마다 한 번 잡고 끝나면 바로 풀었다. 잡은 시간은 8, 14.5, 14.5, 20.5, 24.3분(가장 긴 것이 30분 안),
  사이는 180 s, 합 약 82분.
- 순서는 `run_bench.sh`의 Williams 설계다. 네 컨트롤러가 완전히 균형을 이루려면 8판이 필요하지만 시간 예산(약 1.5 시간,
  04:45 마감)에 맞춰 6판으로 했다. 그래서 자리(몇 번째로 도는지)의 균형은 완전하지 않다.
- c_moving의 사람(지름 8 cm, 0.10 m/s)과 카트(0.06 m/s)는 비키지 않는다. d_robot의 마주 오는 로봇은 우리 로봇 중심과
  0.25 m 안이면 멈춘다(비키는 쪽).
- 성공 = Nav2 SUCCEEDED이면서 실제 위치가 goal에서 0.10 m 안. 충돌 = 몸 발자국과 장애물 면의 실제 거리 0.005 m 이하
  (스치기도 충돌로 센다). 움직이는 장애물이 서 있는 우리를 친 것도 충돌로 센다(누가 쳤는지는 나누지 않는다).

## 결과

### 본 비교 (O_IN → M_E, 칸마다 6판)

| 시나리오 | 컨트롤러 | 성공 | 시간 s (성공 판) | 충돌한 판 | 충돌 합 | BLOCKED | 도리도리 /분 | 주행 vx m/s | CPU % |
|---|---|---|---|---|---|---|---|---|---|
| a_hall | RPP | 6/6 | 13.3 ± 0.4 | 0 | 0 | 0 | 58 | 0.109 | 10 |
| a_hall | MPPI | 6/6 | 15.5 ± 1.2 | 0 | 0 | 0 | 19 | 0.107 | 14 |
| a_hall | RL+shim | 6/6 | 12.6 ± 0.2 | 0 | 0 | 0 | 73 | 0.119 | 10 |
| a_hall | RL | 6/6 | 12.1 ± 0.8 | 0 | 0 | 0 | 90 | 0.102 | 10 |
| e_corridor | RPP | 6/6 | 13.7 ± 0.9 | 0 | 0 | 0 | 53 | 0.109 | 9 |
| e_corridor | MPPI | 6/6 | 13.3 ± 0.6 | 0 | 0 | 0 | 14 | 0.111 | 15 |
| e_corridor | RL+shim | 6/6 | 13.5 ± 1.2 | 0 | 0 | 0 | 76 | 0.119 | 9 |
| e_corridor | RL | 6/6 | 13.6 ± 0.7 | 0 | 0 | 0 | 78 | 0.078 | 9 |
| d_robot | RPP | 5/6 | 19.4 ± 5.5 | 0 | 0 | 4 | 32 | 0.110 | 10 |
| d_robot | MPPI | 6/6 | 16.0 ± 1.7 | 0 | 0 | 0 | 15 | 0.099 | 15 |
| d_robot | RL+shim | 6/6 | 12.6 ± 0.1 | 0 | 0 | 0 | 70 | 0.118 | 10 |
| d_robot | RL | 1/6 | 11.9 | 5 | 6 | 11 | 35 | 0.047 | 10 |
| c_moving | RPP | 5/6 | 35.1 ± 6.2 | 4 | 9 | 1 | 40 | 0.111 | 10 |
| c_moving | MPPI | 5/6 | 35.8 ± 6.7 | 2 | 2 | 4 | 24 | 0.078 | 14 |
| c_moving | RL+shim | 5/6 | 25.1 ± 3.3 | 4 | 14 | 2 | 43 | 0.092 | 10 |
| c_moving | RL | 4/6 | 30.1 ± 7.7 | 4 | 7 | 0 | 54 | 0.050 | 10 |

몸 최소 거리(m, 평균): a_hall RPP 0.102, MPPI 0.105, RL+shim 0.100, RL 0.065. e_corridor 0.082, 0.076, 0.075, 0.053.
d_robot 0.045, 0.045, 0.033, 0.001. 달린 길(m): 막히지 않은 판은 넷 모두 1.12~1.18 m.
도리도리는 판마다 각속도 명령의 부호가 바뀐 횟수를 판의 시간으로 나눈 것이다. 주행 vx는 앞뒤로 움직인 명령만의 평균이라
뒤로 간 시간이 길면 작아진다.

### Q2 → E-3 (장애물 없음, 칸마다 3판)

| 컨트롤러 | 성공 | 시간 s | 충돌한 판 | 몸 최소 m | 도리도리 /분 | 주행 vx m/s |
|---|---|---|---|---|---|---|
| RPP | 3/3 | 14.8 ± 1.3 | 0 | 0.065 | 44 | 0.108 |
| MPPI | 3/3 | 13.2 ± 0.1 | 0 | 0.068 | 14 | 0.110 |
| RL+shim | 3/3 | 17.8 ± 3.4 | 1 | 0.033 | 69 | 0.067 |
| RL | 3/3 | 28.9 ± 7.7 | 0 | 0.052 | 94 | -0.042 |

RL+shim의 첫 판은 출발 3.2 s에 (1.616, 0.083, 1.09 rad)에서 남쪽 벽에 닿았다(몸 거리 0.000). 0.25 정책에서 보던 Q2 스침과
같은 자리다. RL 혼자의 가장 가까운 거리는 0.019 m(칸막이)로 닿지 않았다. RL 혼자는 주행 vx 평균이 음수다: 이 길의
대부분을 뒤로 달렸다.

### Welch 차이 (평균 차이 ± 표준오차, t)

시간은 성공한 판만, 몸 최소 거리는 모든 판으로 셌다. `report.py`의 차이 표는 통로를 벗어난 판을 빼므로 c_moving(모든 판이
통로 밖)과 Q2 → E-3은 거기에 줄이 없다. 아래는 모든 판으로 따로 셈한 것이다(`results/.../main/table.md`의 차이 표는 통로 판만).

| 시나리오 | 차이 | 시간 s | 몸 최소 m |
|---|---|---|---|
| a_hall | RL+shim − RPP | −0.76 ± 0.19, t −4.0 | −0.002 ± 0.001, t −1.4 |
| a_hall | RL+shim − MPPI | −2.92 ± 0.49, t −5.9 | −0.005 ± 0.003, t −1.6 |
| a_hall | RL − RL+shim | −0.44 ± 0.33, t −1.3 | −0.036 ± 0.001, t −26.8 |
| e_corridor | RL+shim − RPP | −0.26 ± 0.63, t −0.4 | −0.006 ± 0.002, t −3.6 |
| e_corridor | RL+shim − MPPI | +0.19 ± 0.56, t 0.3 | −0.001 ± 0.004, t −0.2 |
| e_corridor | RL − RL+shim | +0.10 ± 0.58, t 0.2 | −0.022 ± 0.001, t −14.8 |
| d_robot | RL+shim − RPP | −6.74 ± 2.46, t −2.7 (6/5판) | −0.013 ± 0.001, t −14.9 |
| d_robot | RL+shim − MPPI | −3.34 ± 0.71, t −4.7 | −0.012 ± 0.002, t −5.3 |
| d_robot | RL − RL+shim | (RL 성공 1판) | −0.030 ± 0.003, t −10.3 |
| c_moving | RL+shim − RPP | −10.0 ± 3.1, t −3.2 (5/5판) | −0.004 ± 0.020, t −0.2 |
| c_moving | RL+shim − MPPI | −10.7 ± 3.3, t −3.2 (5/5판) | −0.023 ± 0.019, t −1.2 |
| c_moving | RL − RL+shim | +5.0 ± 4.1, t 1.2 (4/5판) | −0.008 ± 0.016, t −0.5 |
| Q2 → E-3 | RL − RL+shim | +11.1 ± 4.9, t 2.3 (3/3판) | +0.019 ± 0.026, t 0.7 |
| Q2 → E-3 | RL+shim − MPPI | +4.6 ± 1.9, t 2.4 (3/3판) | −0.036 ± 0.020, t −1.8 |

판이 6개(Q2 → E-3은 3개)라 |t|가 2 안쪽인 차이는 있다고 말할 수 없다.

### 그림

`results/2026-11-22/compare_012/main/compare.png`(시간, 도리도리, 몸 최소 거리)와 `tracks.png`(실제 위치 궤적).
궤적에서 보이는 것:
- RL 혼자(빨강)는 O_IN에서 북쪽으로 휘어 y 0.62~0.65를 달린다. 다른 셋은 y 0.57 부근이다. 출발 방향이 110°(거의 북쪽)라
  RL 혼자는 제자리에서 돌지 않고 앞뒤로 움직이며 돈다(제자리 회전 0.3~0.5 s, shim은 2.5~2.7 s).
- d_robot에서 RL 혼자는 마주 오는 로봇이 지나가는 북쪽 줄(y 0.70)에 붙어 달리다 부딪쳤다. 다섯 판의 정적 장애물과의
  최소 거리는 0.026 m 이상이므로 닿은 것은 로봇 대역이다. 한 판은 x 0.4, y 0.8 쪽까지 갔다.
- c_moving에서는 네 컨트롤러 모두 여러 판이 남쪽 통로(y 0.2)로 돌았다(계획 경로가 통로 밖, 통로 판 0/6).

### 성공이 아닌 판

| 판 | Nav2 결과 | 시간 s | 몸 최소 m | 충돌 | 끝 위치 오차 m |
|---|---|---|---|---|---|
| c_moving RPP 1 | ABORTED | 22.0 | 0.000 | 1 | 0.66 |
| c_moving MPPI 5 | TIMEOUT | 91.0 | 0.000 | 1 | 0.82 |
| c_moving RL+shim 1 | ABORTED | 46.7 | 0.000 | 9 | 0.61 |
| c_moving RL 1, 2 | SUCCEEDED | 21.2, 16.4 | 0.000 | 3, 1 | 0.73, 0.36 |
| d_robot RPP 4 | ABORTED | 38.0 | 0.047 | 0 | 0.81 |
| d_robot RL 0 | SUCCEEDED | 73.6 | 0.000 | 2 | 1.23 |
| d_robot RL 1, 2, 4, 5 | ABORTED | 32.7~39.5 | 0.000~0.002 | 1씩 | 0.59~0.72 |

SUCCEEDED인데 실패로 센 세 판(c_moving RL 둘, d_robot RL 하나)은 부딪힌 뒤 AMCL 위치가 틀어져 Nav2는 도착했다고 했지만
실제 위치는 goal에서 0.36~1.23 m 떨어져 있었다.

### 계산 부담

- CPU %(controller_server, bench가 잼): RPP와 RL과 RL+shim 9~10 %, MPPI 14~15 %.
- 추론 시간: bench는 Nav2를 warn 로그 수준으로 띄워 FollowPathRL의 1 s INFO 줄(추론 시간)이 남지 않는다. 그래서 parity
  시험의 6.7 µs(PC, 한 번)만 적는다. Pi에서는 재지 않았다.
- 명령 간격 p95: RPP와 RL 52 ms, MPPI와 RL+shim 60 ms.
- 컨트롤러 주기(20 Hz)를 놓쳤다는 경고: MPPI 48판 중 두 번(c_moving MPPI 3과 5, 한 번씩, 가장 느린 것 15.6 Hz),
  RPP 15번(d_robot 3~5, c_moving 5, 막혀 오래 걸리거나 실패한 판), RL 한 번(d_robot RL 0, 16.7 Hz). a_hall, e_corridor, Q2 → E-3에서는 0.
  나머지 경고는 planner가 길을 못 찾는 동안 느려진 것이다(planner_server, 막힌 판들).

## 결정 대기 목록

사용자가 정할 것:

1. **RL 기본을 RL 혼자로 할지 RL+shim으로 할지.**
   - 이번 비교에서 shim 쪽이 나은 점: d_robot 6/6 대 1/6, 몸 최소 거리가 세 시나리오에서 0.02~0.04 m 더 넓다(t −10 이하),
     시간은 같거나 짧다.
   - shim 쪽 비용: Q2 → E-3에서 한 판 남쪽 벽에 닿았다(RL 혼자는 0/3). 제자리 회전이 길어 큰 방향 오차로 시작하는 판이
     조금 느리다(a_hall에서 RL 혼자보다 0.44 s, t −1.3로 차이 없음).
   - shim은 지금 sim 덮어쓰기(`sim/fleet_gz_sim/nav_overrides/follow_path_rl_shim.yaml`)에만 있다. 기본으로 하면 실물
     `nav2_params.yaml`에도 넣어야 한다. `angular_dist_threshold` 0.785는 제안 값이다.
2. **가드(`rear_stop_range`)를 켤지.** 오늘 밤 묶음은 가드를 끄고(0.0) 돌렸다. 가드는 뒤로 가는 행동을 잘라 제자리 회전으로
   바꾸는데, 정책은 뒤로 가기를 쓰도록 학습되어 있어 닫힌 고리가 학습 때와 달라진다(4b 0.25에서 15~24 % 호출이 막혔다,
   `docs/worklog_follow_path_rl_2026-11-21.md` 11절). 켤지는 사용자가 정한다.

내가 기본으로 정한 것(바꿔도 된다):

3. **MPPI `time_steps` 83.** 0.25 m/s에서 고른 40 × 0.05 s × 0.25 m/s = 0.50 m를 0.12 m/s에서도 내다보게 늘렸다
   (83 × 0.05 × 0.12 = 0.498 m). `model_dt`는 컨트롤러 주기(20 Hz)와 같아야 하므로 그대로 두었다. Gazebo에서 주기를
   지켰다(위 "계산 부담": 48판 중 경고 두 번, CPU 14~15 %). sim에서 상한을 더 낮추면 `time_steps`도 같은 비율로 늘어난다.
4. **판 수 6, Williams 순서.** 완전 균형인 8판은 04:45 마감에 들어가지 않았다. 시나리오마다 잠금 한 번, 사이 180 s.
5. **스치기(몸 거리 0.005 m 이하)도 충돌로 센다.** bench의 `collision_m`과 같고, Isaac의 재학습 기준(스치기 = 충돌)과도 같다.

## 주의

- **학습의 움직이는 장애물은 비키는 것과 비키지 않는 것이 섞여 있다.** c_moving은 비키지 않는 것만, d_robot은 비키는 것만이다.
  c_moving의 충돌은 네 컨트롤러 모두에서 나왔고 MPPI가 가장 적었다(2판). 서 있는 우리를 장애물이 친 것도 섞여 있다.
- **가드는 오늘 밤 돌리지 않았다.** 가드는 뒤로 가는 정책의 행동을 잘라낸다. 고침이 아니라 빠른 확인용이다.
- **Isaac 스윕과 Gazebo의 차이.** Isaac의 528 출발 스윕은 스침 0이었지만 Gazebo의 Q2 → E-3에서 RL+shim 한 판이 벽에 닿았다.
  [추측] shim이 돌고 넘겨준 직후의 상태(방향 오차 45° 안, 남쪽 벽 가까이)는 스윕의 출발점(정해진 자리 ×
  30° 간격 방향)에 없을 수 있다. 확인하려면 그 상태를 스윕 출발점에 넣어 본다.
- 칸마다 6판이고 Williams 균형이 완전하지 않다. d_robot의 RL 혼자는 성공이 1판이라 시간 비교를 하지 않았다.
- `compare.png`과 `tracks.png`의 b_static 칸은 돌리지 않아 비어 있다.
- 이번 묶음 동안 viewer가 붙었는지는 확인하지 않았다. 끝난 뒤 내 표시(BENCH_RUN)가 붙은 프로세스는 없었다.

## 알아야 할 포인트

- **RotationShim은 "먼저 돌고, 그다음 맡긴다".** 경로 방향과 로봇 방향의 차이가 `angular_dist_threshold`(0.785 rad = 45°)보다
  크면 shim이 제자리에서 돌고, 그 안으로 들어오면 감싼 컨트롤러(여기서는 FollowPathRL)에 넘긴다. 그래서 RL은 큰 방향 오차를
  거의 보지 않고 출발한다. O_IN(110°)에서 RL 혼자가 북쪽으로 휘던 것이 shim으로 사라진 까닭이다.
- **성공은 실제 위치로 판정한다.** Nav2가 SUCCEEDED라고 해도 Gazebo의 실제 위치가 goal에서 0.10 m 밖이면 실패다. 부딪혀
  AMCL이 틀어지면 Nav2는 엉뚱한 곳에서 도착했다고 한다(이번에 세 판).
- **Welch t.** 두 묶음의 분산이 달라도 쓰는 평균 차이 검정이다. 차이 ÷ 표준오차가 t이고, 판이 6개씩일 때 |t|가 대략 2.2~2.6(자유도에 따라)을
  넘어야 5 % 수준에서 차이가 있다고 말한다. |t| 2 안은 "모른다"로 읽는다.
- **도리도리 /분.** 각속도 명령의 부호가 바뀐 횟수(±0.02 rad/s 안은 건너뜀)를 분으로 나눈 것이다. RL은 20 Hz마다 새로 고르는
  정책이라 RPP보다 크고, MPPI는 매끈함 critic이 있어 가장 작다.
- **MPPI의 내다보는 거리 = `time_steps` × `model_dt` × `vx_max`.** 속도를 낮추고 `time_steps`를 그대로 두면 같은 시간만
  내다보므로 거리가 줄어든다. 그래서 속도를 바꿀 때 `time_steps`를 함께 바꾼다.

## 직접 해 볼 것

1. 표를 다시 만든다(WSL, `source /opt/ros/jazzy/setup.bash` 뒤 저장소에서):

   ```bash
   python3 tools/nav_bench/report.py --config tools/nav_bench/scenarios.yaml \
     --legs ~/rl_gz_check/cmp012_025415/main/legs --out /tmp/cmp012
   ```

   `/tmp/cmp012/table.md`의 첫 표가 위 본 비교 표와 같은지(성공, 시간) 본다.
2. d_robot에서 RL 혼자와 RL+shim을 두 판씩만 돌려 본다(잠금을 잡은 뒤, 도메인 62):

   ```bash
   export BENCH_DOMAIN=62 BENCH_REPEATS=2 \
     BENCH_OVERRIDES=results/2026-11-22/compare_012/rl_and_shim.yaml \
     BENCH_CONTROLLERS="FollowPathRL FollowPathRLShim"
   bash tools/nav_bench/run_bench.sh /tmp/try_d_robot d_robot
   python3 tools/nav_bench/report.py --config tools/nav_bench/scenarios.yaml \
     --legs /tmp/try_d_robot/legs --out /tmp/try_d_robot/report
   ```

   `/tmp/try_d_robot/report/tracks.png`에서 빨강(RL)이 y 0.65 부근으로, 초록(RL+shim)이 y 0.57 부근으로 가는지 본다.
3. `rl_and_shim.yaml`의 `angular_dist_threshold`를 0.785에서 0.4로 바꿔 2번을 다시 돌린다. 제자리 회전 시간(표 둘째 칸
   "제자리 회전 s")이 늘고 RL이 넘겨받을 때의 방향 오차가 줄어드는지 본다.
4. Q2 → E-3을 RL+shim으로 세 판 돌려(`BENCH_CONFIG=results/2026-11-22/compare_012/q2e3.yaml`, 시나리오 `a_hall`)
   출발 3~4 s 사이에 남쪽 벽(y 0)에 붙는 판이 있는지 `tracks.png`에서 본다.

## 남은 것

- 가드 켠 판, RL+shim의 실물 `nav2_params.yaml` 반영, Pi에서의 추론 시간은 하지 않았다.
- 새 정책의 528 출발 스윕은 Isaac 쪽이 이미 돌렸으므로(528/528, 스침 0) 다시 돌리지 않았다.
