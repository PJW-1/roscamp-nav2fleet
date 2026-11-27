# 11-23 서버 켬 판: 구간 끝 정지와 관제 ETA 오차 (Gazebo 대비)

사용자 질문(11-23): 켬 판 기록에서 (1) 다음 구간 예약을 기다리며 구간 끝에서 멈춘 횟수(구간별),
(2) ETA 추정과 실제 도착 시간 차이(평균, 출발 직후 따로)를 Gazebo 값(정지 2/80, ETA 평균 -0.41 s, 출발 -1.47 s)과 비교.

## 쓴 기록과 도구

- 주 기록: `runs/1123_ds_on.jsonl`(두 대, 주문 4건, 모두 출고). 보탬: `runs/1123_pode2_ds_on.jsonl`(11:10 pode2 혼자, 주문 4건).
  두 판 모두 nav2fleet 태그 `rpp-baseline-2026-11-21`(b9e86c5), 디스커버리 서버 켬.
- 정지: `analysis/base_end_stops.py`. 관제 이벤트 `BASE_END_WAIT`("stands at X, the next stretch is not taken yet")를
  끝 노드별로 세고, 분모는 그 노드에서 끝난 Segment 수(`SEGMENT_DONE`)다. 선 시간은 진행 피드백의
  `waiting_at_base_end`가 참인 동안이다. 출력 `base_end_stops.txt`.
- ETA: pode-drive `feat/eta-cause-2026-11-22`(b905ac4)의 `tools/eta_cause/eta_split.py`를 b9e86c5 코드(git archive)에 넣어
  돌렸다. 기록을 그 코드로 재생하며, nav_goal Segment 동안 위치 보고마다(0.5 s) 관제 ETA(`_left_s`)와 실제 남은 시간
  (그 Segment의 `SEGMENT_DONE`까지)을 잰다. Gazebo 숫자를 낸 도구와 같다. 재생 경고는 두 판 모두 0이다.
  출력 `eta_split_*.txt`, 표본 `rows_*.json`.
- 오차 = ETA - 실제. -면 관제가 일찍 닿는다고 본 것이다.
- Gazebo 값의 출처: 정지 2/80은 `docs/worklog_left_s_projection_2026-11-20.md` 비교 표(Q2에서 선 Route, 8판: rush 2/48,
  gz_v7 0/32. 선 두 번은 5.47 s씩), ETA는 `docs/worklog_eta_cause_2026-11-22.md` 3절(nav2fleet 9612003, 6판, 2136표본, Segment 98개).

## 1. 구간 끝 정지 (BASE_END_WAIT)

| 끝 노드 | 켬 판 (선 / 끝난 Segment) | pode2 혼자 판 |
|---|---|---|
| Q2 | 0 / 4 | 0 / 4 |
| O_IN | 1 / 1 | - |
| O_X | 1 / 3 | 0 / 4 |
| E-1, E-2, M_E, O, O_OUT, P-1, Q1, W | 0 / 18 | 0 / 19 |
| 합계 | **2 / 28** | 0 / 27 |

- 정지 두 번은 모두 pode2가 출고 도크 O로 가는 길이었다. 둘 다 바로 뒤에 `ZONE_GRANTED ... OUT_DOCK to O`가 와서 다시 움직였다.
  출고 도크 구역(OUT_DOCK)이 풀리기를 기다린 것이다. 그때 누가 쥐고 있었는지는 이 표에서 직접 보지 않았다.
  - +134.7 s O_IN에서 2.2 s(다음 허가 `HALL+OUT_DOCK to O`).
  - +198.6 s O_X에서 3.2 s(다음 허가 `EXPRESS+OUT_DOCK to O`).
- Gazebo 2/80은 Q2 Route만 센 값이다. 실물 Q2는 0/4(혼자 판 0/4)라 정지가 없었다. 다만 표본이 4개라 2.5 %와 견줄 만큼 많지 않다.
- 혼자 판에는 기다릴 상대가 없어 정지가 0인 것이 당연하다.

## 2. 관제 ETA 오차

| | Gazebo | 켬 판 | pode2 혼자 판 |
|---|---|---|---|
| 전체 평균 | -0.41 (2136표본) | **-0.79** (149표본, Segment 7개) | -1.21 (133표본, 5개) |
| pass 평균 | -0.41 | -0.74 | -1.17 |
| turn 평균 | -0.42 | -1.20 (1개) | -1.41 (1개) |
| 끝 무렵, 실제 남은 0~1 s, pass | (남은 0~1 s pass -0.06) | -0.04 | -0.01 |
| 출발(Segment 첫 표본) | -1.47 | **-2.29** (7개) | -3.30 (5개) |
| 출발 pass | -1.53 | -2.24 (6개) | -3.14 (4개) |
| 출발 turn | -1.17 | -2.59 (1개) | -3.94 (1개) |
| 움직일 때 속도(전체 표본) | 0.108 m/s(출발 pass) | 0.102 m/s | 0.096 m/s |

### 출발 오차의 원인 나누기(Segment 첫 표본, 초)

오차 = 끝 보정(adj) + 길이 + 속도 - 제자리 회전 - 서 있음. 도구의 정의 그대로다.

| | 오차 | adj | 길이 | 속도 | 회전 | 서 있음 |
|---|---|---|---|---|---|---|
| Gazebo, 출발 pass | -1.53 | -1.25 | +1.17 | -0.72 | -0.67 | -0.07 |
| 켬 판, 출발 pass(6개) | -2.24 | -1.25 | +1.11 | -0.90 | -0.89 | -0.31 |
| 혼자 판, 출발 pass(4개) | -3.14 | -1.25 | +1.04 | -1.69 | -1.00 | -0.25 |

### Segment마다 (첫 표본)

| 판 | Segment | 출발 → 끝 | 다음 | ETA | 실제 | 오차 | 속도 몫 | 회전 몫 | 서 있음 |
|---|---|---|---|---|---|---|---|---|---|
| 켬 | pode1-1#2 | E-1 → Q2 | follow_path | 11.6 | 14.8 | -3.25 | -0.89 | -1.00 | -0.50 |
| 켬 | pode2-4#2 | E-2 → Q2 | follow_path | 9.4 | 12.0 | -2.53 | -0.97 | -1.00 | 0 |
| 켬 | pode2-13#1 | W → M_E | nav_goal | 0.8 | 3.1 | -2.32 | -0.47 | -1.00 | -0.71 |
| 켬 | pode2-13#2 | M_E → O_IN | follow_path | 9.9 | 13.5 | -3.63 | -1.62 | -1.00 | -0.63 |
| 켬 | pode1-16#2 | O_OUT → Q2 | follow_path | 13.4 | 14.3 | -0.91 | -0.67 | -0.50 | 0 |
| 켬 | pode2-22#2 | O_OUT → Q2 (주차 가던 중 O_OUT에서 새 주문으로 바뀜) | follow_path | 13.7 | 14.5 | -0.78 | -0.78 | -0.82 | 0 |
| 켬 | pode1-35#2 | O_OUT → E-1 | 후진(turn) | 6.8 | 9.4 | -2.59 | -1.49 | -3.83 | -1.00 |
| 혼자 | pode2-2#2 | E-2 → Q2 | follow_path | 9.4 | 14.9 | -5.48 | -2.94 | -1.50 | -0.50 |
| 혼자 | pode2-11#2 | O_OUT → Q2 | follow_path | 13.5 | 19.8 | -6.31 | -2.89 | -2.00 | -0.50 |
| 혼자 | pode2-20#2 | O_OUT → Q2 | follow_path | 13.2 | 13.8 | -0.60 | -0.36 | -0.49 | 0 |
| 혼자 | pode2-29#2 | O_OUT → Q2 | follow_path | 13.3 | 13.5 | -0.19 | -0.56 | 0 | 0 |
| 혼자 | pode2-38#2 | O_OUT → E-1 | 후진(turn) | 6.5 | 10.4 | -3.94 | -1.95 | -4.00 | -1.39 |

(ETA와 실제는 초. pode2-13#1은 W 도크 자리(1.87, 0.45)에서 받았다. 끝 노드는 표본의 끝 좌표로 확인했다.)

## 3. 읽는 법

- **실물도 관제가 일찍 본다. Gazebo보다 크다:** 전체 평균 -0.79 s(Gazebo -0.41), 출발 -2.29 s(Gazebo -1.47).
- **끝 무렵은 Gazebo와 비슷하다:** 실제 남은 0~1 s pass -0.04 s(11표본). 넘어가기 위치(SEGMENT_DONE 때 끝 노드 앞 거리 중앙값)도
  -0.146 m(5개 Segment)로 Gazebo -0.132 m와 비슷하다. 다음이 follow_path인 구간에서는 끝 보정(-1.25 s)이 맞는 쪽이다.
  다음이 nav_goal인 구간은 1개뿐이라 그 경우는 확인하지 못했다.
- **출발 오차가 커진 몫은 속도, 회전, 서 있음이다**(켬 판 출발 pass, Gazebo 대비): 속도 -0.90(-0.72), 회전 -0.89(-0.67),
  서 있음 -0.31(-0.07). 관제 모델에 없는 출발 제자리 회전과 0.12보다 느린 순항이 Gazebo보다 더 크게 나왔다.
- **같은 O_OUT → Q2라도 판마다 다르다:** 켬 판 -0.91, -0.78, 혼자 판 -0.60, -0.19는 Gazebo 출발 값보다 작다.
  혼자 판 pode2-11#2는 -6.31로 크다. 속도 몫 -2.89(움직일 때 0.100 m/s)와 회전 몫 -2.00이 크다. 왜 그랬는지는 이 기록으로 가를 수 없다.
- **표본이 적다:** 켬 판 nav_goal Segment 7개, 혼자 판 5개다. Gazebo 2136표본, Segment 98개와 같은 무게로 볼 수 없다.
  turn과 next=nav_goal은 각 1개뿐이라 그 줄의 숫자는 참고만 한다.
- 이 숫자는 `docs/worklog_eta_cause_2026-11-22.md` 5절 결정 E1(순항 속도 비율), E2(출발 회전 항), E3(끝 회전 속도)의 실물 자료다.
  E4(다음 조각이 잡혀 있을 때만 넘어가기 보정)는 next=nav_goal 표본이 1개라 이 판으로 정할 수 없다.

## 다시 내기

```bash
cd results/2026-11-23/analysis
python3 base_end_stops.py ../runs/1123_ds_on.jsonl
REC=$(readlink -f ../runs/1123_ds_on.jsonl)
# ETA: b9e86c5 코드에 eta_split.py를 넣어 재생한다(ROS 불필요)
D=$(mktemp -d); git -C <nav2fleet> archive b9e86c5 | tar -x -C $D; mkdir -p $D/tools/eta_cause
git -C <pode-drive> show b905ac4:tools/eta_cause/eta_split.py > $D/tools/eta_cause/eta_split.py
(cd $D && python3 tools/eta_cause/eta_split.py ds_on=$REC --rows=rows.json)
```
