# v7 배치 변경: 칸막이 3 빼기, 진열대 A 서쪽 0.05 (2026-11-17)

사용자 요청(11-17): "충전 장소는 그대로 유지하지만 3번의 제일오른쪽 칸막이만 없앨생각이야", "진열대 a만 5CM정도 늘릴까하는데", "자리는 그대로 왼쪽으로만", "새로운지도로 이상없는지 테스트한번 해주고 시뮬도한번굴려보고있어".
브랜치 `feat/v7-no-divider3`(main 9261ad5에서). main에는 넣지 않았다.

## 1. 바꾼 것

| 파일 | 바뀐 것 |
|---|---|
| `pc/fleet_core/fleet_core/data/fleet_map_v7.yaml` | 칸막이 3 `[0.76, 0.0, 0.78, 0.10]`과 그 둘레 keepout `[0.71, 0.0, 0.86, 0.20]`을 뺐다. 진열대 A `[0.68, 0.77, 1.56, 0.81]` -> `[0.63, 0.77, 1.56, 0.81]`(길이 0.93). 진열대 서쪽 HALL 띠 `[0.50, 0.77, 0.68, 0.81]` -> `[..., 0.63, ...]`(진열대 자리는 어느 구역도 아니다). 설명 주석. 충전 자리 P-1~P-3, 입구 E-1~E-3, 다른 지점과 길은 그대로다 |
| `sim/fleet_gz_sim/worlds/arena_v7.sdf`, `sim/fleet_gz_sim/maps/arena_v7*.pgm` | `python3 -m fleet_gz_sim.arena --layout v7`로 다시 만들었다(keepout 마스크는 칸막이 3 때만 바뀐다) |
| `tools/real_pode/maps/arena_v7{,_a,_fine,_keepout}.pgm` | 시뮬 지도와 같은 파일로 바꿨다(yaml은 그대로 같다) |
| 시험 | `test_layout_v7.py`(칸막이 둘, P-3 동쪽에 상자와 keepout 없음, 진열대 A 0.93과 동쪽 끝 1.56, 진열대 자리는 구역 밖), `test_keepout_mask.py`(남은 둘레 둘, 칸막이 3 자리는 열림), `sim/fleet_gz_sim/test/test_arena_v7.py`(월드 상자, 칸막이 3 자리와 keepout 없음, P-3에서 동쪽으로 y 0.20 아래로 이어짐), `sim/fake_robots/test/test_world_planner.py`(머리 없는 세계의 keepout 점을 칸막이 2 둘레로) |

시험을 먼저 바꿔 실패하는 것(칸막이 7개, 진열대 2개)을 본 뒤 배치를 바꿨다. 다른 세션이 같은 치수로 실물 배치 도면을 만들고 있다(Codex 기록 "실물 배치 도면: 칸막이 3 제거, 진열대 A 서쪽 0.05").

## 2. 시험 [확정]

이 worktree를 colcon으로 빌드하고 그 설치본을 source해서 돌렸다.

- fleet_core 1118 passed, 5 xfailed. fleet_gz_sim 586 passed. tools 98 passed.
- fake_robots: 처음 판에서 `test_v7_keepout_extra_is_not_floor` 하나가 실패했다(지운 칸막이 3 둘레 (0.84, 0.19)를 막혀 있다고 기대, Codex 리뷰가 찾음). 나머지 884 passed, 137 skipped, 27 xfailed. 고친 뒤 그 파일 15 passed.
- 사용자 작업공간의 옛 설치본(956140d)을 source하면 launch 배선 시험 10개가 실패한다(옛 launch 파일을 읽어서). 이 변경과 관계없다.
- pc/fleet_manager 시험은 ROS 도메인 50~59를 써서 돌리지 않았다. v7을 읽는 시험이 없다(grep).

## 3. Gazebo 세 대 판 [확정, summary.json, safety.json, robots.csv, events.csv]

조건은 앞 판들과 같다: `layout:=v7 scenario:=gz_v7_varied obstacles:=v7_yard_mix battery_low_pct:=20 use_order_model:=true sim_dock_stub:=true rviz:=fleet gui:=true fleet_record:=true`, 도메인 61.

| 판 | 배치 | 결과 | 판 시간 | 위반 | near | 최소 거리 | pode2 BLOCKED 표본(시각) |
|---|---|---|---|---|---|---|---|
| final_main | 옛 배치(main) | PASS 12/12 | 572.5 s | 0 | 0 | 0.166 | 38 (429~459 s) |
| watch_main | 옛 배치(main e4aea0f) | PASS 12/12 | 606.1 s | 0 | 0 | 0.165 | 76 (437~510 s) |
| div3_1 | 새 배치 | PASS 12/12 | 581.7 s | 0 | 0 | 0.162 | 40 (437~480 s) |
| div3_2 | 새 배치 | PASS 12/12 | 682.2 s | 0 | 2 | 0.147 | 135 (431~575 s) |
| div3_watch | 새 배치(사용자가 지켜본 판) | PASS 12/12 | 574.2 s | 0 | 1 | 0.155 | 38 (432~462 s) |

- div3_1은 fake_robots 시험 묶음(약 11분)과 시간이 겹쳐 돌았다. [추측] 그 부하가 시간뿐 아니라 동작에도 영향을 주었을 수 있어, 비교의 기준은 div3_2와 div3_watch로 한다.
- div3_watch에서도 끼임이 났다: 431.9 s BLOCKED_WAIT, P-3의 pode3 (0.63, 0.12) 옆, pode2는 대개 (0.6, 0.3) 부근. 479.1 s M_E로 돌아 481.9 s Q2. div3_2는 아무것도 함께 돌지 않았다.
- 다섯 판 모두 구역 검사(summary.json checks)는 통과다.
- div3_2의 near 두 번은 425.9 s, 430.9 s에 pode2와 pode3가 0.157, 0.158 m(실제 위치)로 가까웠던 것이다. 아래 끼임 장면이다.

## 4. 남쪽 통로 끼임 장면

- [확정] 다섯 판 모두 420 s에 pode2가 Q2로 가라는 주문을 받고, P-3에 선 pode3(약 (0.62~0.64, 0.11~0.13))와 남쪽 진열대 B(y 0.365) 사이로 들어가 섰다. 처음 BLOCKED 자리는 final_main (0.53, 0.31), watch_main (0.76, 0.26), div3_1 (0.64, 0.29), div3_2 (0.65, 0.27)다.
- [확정] div3_2에서는 관제가 길을 6번 바꿨고(ZONE_MARKED_BLOCKED, REROUTE) 5초에 2 cm 미만 움직임(NO_START)이 7번 나왔다. 575.6 s 길 바꿈 뒤 584.9 s에 O_IN에 닿았고 595.8 s에 Q2.
- [확정] 이 장면은 옛 배치 판 둘에서도 났다. 새 배치에서 처음 생긴 장면은 아니다.
- [추측] 칸막이 3을 뺀 것이 이 장면에 영향을 주었는지는 이 자료로 가를 수 없다. 기록된 자리는 로봇 중심이고 계획 경로나 발자국 전체가 아니다. 옛 keepout(y 0.20 아래)이 계획을 위로 밀었을 수 있고, watch_main의 처음 BLOCKED 자리 (0.757, 0.259)는 옛 칸막이 3 위 끝 가까이였다. 첫 BLOCKED 표본부터 마지막 BLOCKED 표본까지의 시간은 판마다 크게 갈린다(옛 배치 두 판 30~73 s, 새 배치 세 판 30~144 s). 이 사이에 MOVING, IDLE 표본도 섞여 있어 연속으로 갇힌 시간은 아니다. 이 정도 판 수로는 새 배치가 더 나쁜지 말할 수 없다.
- 이 장면은 앞서 보류한 문제다(`docs/worklog_bt_backup_2026-11-17.md`, 브랜치 feat/nav-goal-bt-backup).

## 5. O_X(급행 서쪽 끝 출고 대기) 여유

- [확정] 진열대 A를 0.63까지 늘려 O_X (0.655, 0.90)가 진열대 위 면(y 0.81) 바로 북쪽이다. 거리 0.09 m로, 몸 회전 원 0.083 m보다 0.007 m 크다(Codex 리뷰 329ef03). 그 전에는 서쪽 끝 모서리에서 0.0934였다.
- [확정] 급행 고정 경로는 원래 x 0.68~1.56에서 진열대 A 위를 같은 0.09 m 간격(가운데 y 0.90, 진열대 0.81, 북쪽 벽 0.99)으로 지난다. O_X에서 로봇은 서쪽을 보고 서고 돌지 않는다.
- [확정] O_X에서 서서 기다렸다 출발한 경우가 판마다 4~5번 있다(events.csv `BASE_END_WAIT` "stands at O_X", 다섯 판 21번). 다음 구간(`ORDER_UPDATED` to O)까지 3.5~5.9 s 섰고, 그동안 robots.csv 상태 칸은 MOVING이다(주문이 살아 있어서). 앞 판(94c2442, b825351)에서 "O_X에 선 경우가 없다"고 한 것은 이 상태 칸만 보고 잘못 읽은 것이다(Codex 리뷰가 찾음).
- [확정] 선 자리(보고 위치)는 x 0.69~0.73, y 0.90~0.92로 명목 O_X (0.655, 0.90)보다 4~7 cm 동쪽이다. 21번 모두 섰다가 O까지 가는 동안 BLOCKED_WAIT, NO_START, GOAL_RETRY 사건이 없다. O_X 부근 표본의 보고 y는 0.894 아래로 내려가지 않았다.
- [확정] 이 선 자리 x 0.69~0.73은 옛 배치의 진열대 A(서쪽 끝 0.68) 위이기도 하다. 시뮬에서 로봇이 실제로 선 자리의 진열대 여유는 두 배치가 같다.
- [추측] 그래서 O_X는 급행 구간과 같은 조건이라고 본다. 다만 Gazebo는 고정 경로 controller의 충돌 검사가 꺼져 있어(실물은 켜짐) 시뮬에서는 이 여유로 멈출 일이 없다. 실물 로봇이 명목 자리 (0.655, 0.90)에 정확히 서는지, 거기서 출발할 때 멈추지 않는지는 재지 않았다.

## 6. 남은 일

- main에 넣을지(사용자 결정).
- 실물 경기장에서 지도와 실제 배치가 맞는지, O_X 출발.
- 남쪽 통로 끼임(보류 중).

## 7. 참조 데이터(임시 폴더, 사라질 수 있다)

scratchpad `gz/final_main`, `gz/watch_main`, `gz/div3_1`, `gz/div3_2`, `gz/div3_watch`와 같은 이름의 `.log`. 실행 스크립트 `gz/run_div3.sh`, 비교 `gz/compare_backup.py`.
