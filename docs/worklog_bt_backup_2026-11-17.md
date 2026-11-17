# nav_goal BT에 BackUp 넣기 시험 (2026-11-17)

사용자 요청(11-17): "goal부분은 그냥 backup이랑 spin은 넣어도될꺼같은데", 이어서 "그럼 백업만 테스트해봐".
브랜치 `feat/nav-goal-bt-backup`(main e4aea0f에서). main에는 넣지 않았다.

## 1. 바꾼 것

- `pode/pode_nav_config/bt/navigate_to_pose_w_replanning_and_recovery_goal.xml` 복구 RoundRobin: 지도 지우기 → **BackUp 0.15 m, 0.05 m/s** → Wait 5 s. Spin은 그대로 뺐다.
- 시험(`test_nav_config.py`): BackUp이 지도 지우기 다음, Wait 앞에 있고 값이 0.15/0.05이며 Spin이 없는지 본다. Nav2 기본 트리와의 비교는 BackUp 값과 순서만 기본으로 되돌려 나머지가 같은지 본다. 시험을 먼저 바꿔 2개가 실패하는 것을 본 뒤 고쳤다. pode_nav_config 105 passed.
- 왜: 병합본 판들에서 pode2가 남쪽 통로 서쪽에 선 pode3 옆 틈에 끼어 40~90 s 갇혔다. 돌면 뒤 모서리가 닿고, RPP는 후진하지 않아, 관제가 길을 바꿔도 첫 움직임이 막혔다(관제 길 바꿈 4번이 모두 "5초에 2 cm 미만").

## 2. 판 조건

- 세 대 판, `layout:=v7 scenario:=gz_v7_varied obstacles:=v7_yard_mix battery_low_pct:=20 use_order_model:=true sim_dock_stub:=true rviz:=fleet gui:=true fleet_record:=true`, 도메인 61, 한 판씩.
- BackUp 없음: main 병합본 두 판(`final_main` 12:51 빌드, `watch_main` e4aea0f). BackUp 있음: 10bb80e 두 판(`bk1`, `bk2`). 판마다 한 번씩이라 숫자는 반복 측정이 아니다.
- 장면 지표: 420 s에 pode2가 Q2로 가라는 주문을 받은 뒤 Q2에 닿을 때까지 걸린 시간, 그동안 틈 자리(x 0.55~0.80, y 0.22~0.34)에 있던 시간(로봇이 보고한 위치, 표본 약 0.5 s 간격으로 셈).

## 3. 결과 [확정, summary.json, safety.json, robots.csv, events.csv, 로그]

| 판 | BackUp | 결과 | 판 시간 | 주문→Q2 | 틈 자리에 있던 시간 | 관제 막힘 표시 | collision 줄 |
|---|---|---|---|---|---|---|---|
| final_main | 없음 | PASS 12/12 | 572.5 s | 59.7 s | 약 37 s | 2 | 551 |
| watch_main | 없음 | PASS 12/12 | 606.1 s | 102.0 s | 약 93 s | 4 | 969 |
| bk1 | 있음 | PASS 12/12 | 571.9 s | 54.8 s | 약 29 s | 2 | 441 |
| bk2 | 있음 | PASS 12/12 | 572.4 s | 36.2 s | 약 16 s | 1 | 235 |
| 참고: 오전 aI 판 | 없음 | PASS 12/12 | 572.7 s | 28.0 s | 0(틈에 안 들어감) | 0 | 138 |

- 네 판 모두 위반 0, near 0. BackUp 판의 구역 검사(`unheld_zone_pose`, `released_while_off_lane`, `released_without_position_check`)는 모두 0, `zone_holder_max` 1. 로봇 사이 최소 거리(truth) 0.172~0.173 m(없음 판 0.165~0.166 m).
- 시간 맞추기: launch 로그는 벽시계라, 관제의 `LOCAL_YIELD wait` 줄과 events.csv의 같은 사건으로 차이를 맞췄다(bk1 424.7 s, bk2 426.1 s). behavior_server 로그는 launch 로그에 나오지 않아 BackUp이 돈 것은 BT 순서로 짐작한다: controller의 "patience exceeded"가 약 1 s 간격으로 두 번 나면(FollowPath 안쪽 재시도 1번 뒤) 바깥 복구 한 차례이고, RoundRobin은 지도 지우기 → BackUp → Wait 순서다. 새 goal이 오면(GoalUpdated) RoundRobin은 처음부터 다시 센다.
- bk1: 실패 두 쌍(424.4/425.5 s → 지도 지우기, 426.9/428.0 s → BackUp). 428.3~431.0 s에 서쪽으로 약 0.16 m 움직였고, 428.7 s 실제 방향(recovery_probe.csv truth_yaw)이 -10°(동쪽)여서 후진이다. 이 판에는 pode_fsm 물러나기가 없었다(LOCAL_YIELD는 426.7 s "ends BLOCKED after 2 s"). 그러나 곧 다시 동쪽으로 가서 같은 틈(0.64, 0.29)에 또 들어가 약 12 s 섰다. 444.0 s 관제 길 바꿈 뒤 두 번째 BackUp 차례(452.7 s)에는 위치가 450.7~460.6 s 내내 (0.587, 0.313) 그대로였다. 거절됐는지는 로그가 없어 모른다. 460.4 s 두 번째 길 바꿈 뒤 M_E 쪽으로 돌아 474.8 s에 Q2.
- bk2: pode_fsm 양보가 0.10 m 물러났다가(428.1 s, 로그 "succeeded OK after 0.102 m") 429.5 s에 다시 보내져 틈(0.66, 0.29)에 섰다. 실패 두 쌍(437.7/438.7 s → 지도 지우기, 440.1/441.1 s → BackUp). BT BackUp은 441.1 s에 시작해 441.5 s에 `Failed to get result for backup in node halt!`(ERROR)로 끊겼다: 441.4 s 관제 길 바꿈(M_E 쪽)의 새 goal이 BT를 멈췄다. 그동안 움직인 거리는 약 0.02 m다. 443~446 s의 서쪽 움직임은 새 경로를 따라간 것이다(goal controller는 `allow_reversing: false`, pode_avoidance/params/avoidance.yaml). 456.2 s에 Q2. 이 판에서 빠져나온 것은 BackUp이 아니라 관제 길 바꿈이다.

## 4. 읽는 법

- [확정] BT BackUp은 낀 로봇을 실제로 뒤로 뺐다(bk1 0.16 m). 그러나 로봇은 곧 같은 틈으로 다시 들어갔고, bk1의 주문→Q2 54.8 s는 없음 판 final_main 59.7 s와 거의 같다.
- [확정] bk2의 짧은 시간(36.2 s)은 BackUp 덕이 아니다. BackUp은 0.4 s만 돌고 관제 길 바꿈에 끊겼고, 빠져나온 것은 그 길 바꿈이다.
- [추측] 그래서 이 두 판으로는 BackUp이 끼임 시간을 줄인다고 말할 수 없다. 판이 둘씩뿐이고, 없음 판끼리도 37 s와 93 s로 크게 갈린다.
- [추측] 남은 원인은 서 있는 로봇 옆 좁은 틈을 경로가 다시 고르는 것이다(aI 설정). 빼낸 뒤 같은 틈을 다시 고르지 않게 해야 BackUp이 쓸모 있다.
- [확정] 관제의 새 goal이 돌던 BT BackUp을 끊는다(bk2 ERROR 한 줄). 두 판 모두 판 실패나 구역 위반으로 이어지지는 않았다.
- 판 전체 시간은 BackUp 판 571.9/572.4 s, 없음 판 572.5/606.1 s다.

## 5. 남은 일과 위험(Codex 판단 리뷰에서)

- 계약: `docs/Pode_계약.md` 13, 204행은 "nav_goal BT에 Spin·BackUp 없음"을 기준으로 적는다. main에 넣으려면 계약을 함께 고쳐야 한다(사용자 결정).
- 두 물러나기: BT BackUp과 pode_fsm 양보 물러나기는 같은 `backup` 동작 서버를 쓴다. pode_fsm은 Nav2 goal을 먼저 취소하고 물러나지만, 관제의 새 goal이나 취소가 BT BackUp을 끊을 수 있다(bk2의 ERROR 한 줄). BT BackUp은 앞 로봇과의 우선순위를 보지 않는다.
- 구역: BackUp은 지역 costmap만 보므로 keepout과 관제 구역을 모른다. 이번 두 판에서는 구역 검사 위반이 0이었지만 보장은 아니다.
- 다음 후보: 물러난 뒤 같은 틈으로 다시 가지 않게 하기(경로가 서 있는 로봇 옆 좁은 틈을 고르지 않게). BackUp을 main에 넣을지는 그 뒤에 다시 잰다.

## 6. 참조 데이터(임시 폴더, 사라질 수 있다)

- scratchpad `gz/final_main`, `gz/watch_main`, `gz/bk1`, `gz/bk2`, `gz/corr_aI`와 같은 이름의 `.log`. 비교 스크립트 `gz/compare_backup.py`, 실행 스크립트 `gz/run_backup.sh`, bk2 시간표 `gz/bk2_timeline.py`(로그, 벽시계-시뮬 차이 1794893545.93).
