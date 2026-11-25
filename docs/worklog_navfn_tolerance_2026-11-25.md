# 작업 기록: NavFn 대체 허용치 0.5 → 0.05 (2026-11-25)

사용자 결정(11-25, `전체_할일_요약_2026-11-25.md`의 "정할 것" 첫 줄): 선택지 1, NavFn `tolerance`를 줄인다.

## 문제

- 11-24 실물(pode2)에서 O_X로 보낸 `navigate_to_pose`가 도착하지 못했는데 Nav2가 SUCCEEDED를 냈다.
- 2.5 cm costmap에서 급행 줄 전체가 내접 반경 안이라 O_X 칸이 막혀 있었다. NavFn `tolerance: 0.5`가 반경 0.5 m 안의
  다른 점(진열대 A 남쪽)으로 경로를 냈고, 그 점이 1~4초마다 바뀌어 로봇이 제자리에서 돌다가 섰다.
- 관제는 이미 도착을 위치로 다시 본다(`fleet_core/traffic/zone/results.py` `_succeeded`: 성공 뒤 목표에서
  `arrival_tolerance_m` 0.15 m 밖이면 `arrival_settle_s` 뒤 `NOT_AT_TARGET`). 11-24 측정은 관제 없이 goal을 직접 보내 이 확인을 거치지 않았다.
  관제를 거쳐도 로봇은 엉뚱한 점까지 가서 돈 뒤에야 재시도한다.

## 바꾼 것

- `pode/pode_nav_config/params/nav2_params.yaml` `planner_server.GridBased.tolerance`: 0.5 → 0.05.
  NavFn은 goal 칸이 막히면 goal을 가운데 둔 x·y 각 ±tolerance 사각형에서 goal에 가장 가까운 빈 점을 고른다.
  그래서 대체점은 goal에서 최대 0.071 m(대각선)이고, 그 안에 빈 점이 없는 goal은 경로 계획에서 실패해
  NavigateToPose가 중단되고 관제의 재시도로 넘어간다.
- 대체점을 골랐을 때 Nav2 도착 판정(`xy_goal_tolerance` 0.05)은 원래 goal이 아니라 경로 끝(대체점)에 걸린다.
  그래서 원래 goal에서 최대 약 0.12 m에서 SUCCEEDED가 날 수 있고, pode_fsm(`nav_moves.py`)과 관제(`results.py`)의
  0.15 m 위치 확인도 통과한다. 이것은 관제 도착 허용치 안이라 받아들인다. 고친 것은 0.5 m 밖 엉뚱한 점에 서는 일이다.
- 시험 `test_nav_config.py::test_the_planner_does_not_swap_a_blocked_goal_for_a_far_point`: 대체 허용치가 도착 판정 허용치를 넘지 않는지 본다.
  고치기 전 값(0.5)에서 실패하는 것을 먼저 확인했다.
- 0.15(관제 도착 허용치)나 0.1로 넓히지 않은 까닭: 사각형 대각선이 0.21·0.14 m가 되어, O_X에서 직선 0.133 m인 진열대 A 남쪽 점(진열대 건너편)을 대체점으로 고를 수 있다.
  0.05면 대각선 0.071 m라 건너편 점은 고를 수 없다.

## 영향

- 이 값을 쓰는 곳은 Nav2 goal(HALL) 경로뿐이다. 관제 코드는 `compute_path_to_pose`를 부르지 않는다. `pode_plan_keep`은 planner에 그대로 넘긴다.
- 새 1 cm 지도에서 goal 여유: O_X 0.088 m, Q2 0.150 m, O_IN 0.193 m(내접 0.07 m보다 크다). 2.5 cm costmap의 O_X는 전과 같이 못 가지만, 이제는 가짜 성공 대신 실패가 난다.
- 받아들인 대가: goal 칸이 잠깐 막히고(사람, 다른 로봇) 가장 가까운 빈 점이 0.071~0.15 m에 있으면, 예전에는 그 점에 서서 도착으로
  받았지만 이제는 계획 실패(`NAV_ABORTED`) 뒤 관제가 `move_retry_wait_s`마다 재시도하고, `move_retry_limit`을 넘기면 격리한다.
  0.15 m 밖 대체점은 예전에도 `NOT_AT_TARGET` 실패였다.
- 실물에 들어가려면 nav2fleet 동기화와 로봇 배포가 필요하다(사용자가 말할 때).

## 시험 (ROS_DOMAIN_ID 94·95, LOCALHOST)

| 시험 | 결과 |
|---|---|
| `pode_nav_config` 전체 | 132 passed |
| `fleet_gz_sim` `test_sim_nav_params.py`, `test_launch_wiring.py` | 119 passed, 11 failed |
| `tools` `test_real_pode.py` | 156 passed, 1 failed |

실패 12건은 이 변경 전(origin/main 58b0725)에도 똑같이 실패한다. 이 PC의 설치본(`podedrive_ws/install`)이
진열대 4 cm 수정 전 `fleet_map_v7.yaml`을 담고 있어서 생기는 환경 문제다.
