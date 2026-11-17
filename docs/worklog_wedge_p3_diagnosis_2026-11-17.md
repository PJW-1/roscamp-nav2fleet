# P-3 앞 남쪽 통로 끼임 진단 (2026-11-17)

사용자: "원래 전에는 통과했던거같은데 왜갑자기 멈추지", "왜 갑자기 막히는지 그리고 해결방안도 찾아놔".
브랜치 `exp/wedge-clear-timeout-off`(진단용, 실험 코드는 main에 넣지 않는다. 이 기록 문서는 main에 있다). 바탕은 `feat/v7-no-divider3`(새 배치).

## 1. 장면

시나리오 `gz_v7_varied`, 420 s: pode2가 P-2 -> E-2(follow_path) -> Q2(nav_goal, Nav2가 길을 그린다). pode3는 P-3 (0.63, 0.12~0.14)에 IDLE(400~435 s 배터리 부족). E-2에서 Q2로 가는 길은 pode3 앞 끝(y 약 0.19)과 남쪽 진열대 B 아랫면(y 0.365, x 0.68부터) 사이를 지난다.

## 2. 판 비교 [확정]

자료: scratchpad `gz/<판>/events.csv`, `robots.csv`, `gz/<판>.log`. 로그 수는 `ORDER_SENT pode2-91` 줄부터 다음 `pode2-9x update 1` 줄까지 pode2 줄을 셌다(div3_2만 이 끝이 584.9 s로 Q2 595.8 s보다 앞이다). BLOCKED는 robots.csv 표본 수이고 연속 시간이 아니다.

| 판 | 설정 | Q2 도착(s) | BLOCKED 표본 | 막힘 표시 | NO_START(목표) | planner failed to plan | clear 시간 초과 | controller patience 초과 | collision ahead |
|---|---|---|---|---|---|---|---|---|---|
| corr_base | 옛 기본(5 cm, pad 0.03, 6.0/0.25), clear 20 ms | 530.1 | 66 | 3 | 3 (Q2) | 2 | 11 | 22 | 548 |
| corr_a | A(2.5 cm, pad 0.01, 6.0/0.25), 20 ms | 433.3 | 0 | 0 | 0 | 0 | 0 | 1 | 31 |
| corr_aI | aI(2.5 cm, pad 0.01, 15/0.15), 20 ms | 448.0 | 6 | 0 | 0 | 1 | 2 | 0 | 44 |
| final_main | main: aI + keepout 틈 제거 + BLOCKED 유지(pode) + BLOCKED 타이머(fleet) + clear 1000 ms | 479.7 | 38 | 2 | 2 (Q2) | 0 | 0 | 14 | 457 |
| watch_main | main | 522.0 | 76 | 4 | 4 (Q2, O_IN x2, Q2) | 0 | 0 | 31 | 872 |
| div3_1 | main + 새 배치(시험 부하 겹침) | 496.3 | 40 | 2 | 2 (Q2) | 0 | 0 | 23 | 522 |
| div3_2 | main + 새 배치 | 595.8 | 135 | 6 | 7 (Q2, O_IN x3, Q2 x3) | 0 | 0 | 60 | 1418 |
| div3_watch | main + 새 배치 | 481.9 | 38 | 2 | 2 (Q2) | 0 | 0 | 14 | 350 |
| **wedge_noclr** | main + 새 배치, **clear 20 ms(`exp/wedge-clear-timeout-off`)** | **467.6** | **19** | **1** | **0** | **7** | **7** | **0** | **64** |

- corr_* 판은 브랜치 `exp/corridor-costmap-ab` 680fda8 빌드(aI는 nav_overrides)다. main 판과 다른 점은 clear 시간만이 아니다(keepout 틈, BLOCKED 유지, fleet 타이머도 없다).
- wedge_noclr는 div3 설치본 위에 `exp/wedge-clear-timeout-off`의 `pode_nav_config`만 얹었다(`run_wedge.sh`). 그 패키지의 차이는 Clear 노드 4곳의 `server_timeout` 1000 -> 생략(= bt_navigator `default_server_timeout` 20 ms)이다.
- wedge_noclr의 pode2: 428 s (0.68, 0.28)까지 틈에 들어갔다가 RECOVERING으로 (0.62, 0.38)(진열대 B 서쪽 끝 바깥)까지 물러났다. 431~451 s 그 자리에 섰고(441~450 s BLOCKED), 451 s 관제가 E-2->E-3를 막힘 표시하고 길을 다시 주자 북쪽 가운데 통로((0.52, 0.43) -> (0.96, 0.57) -> (1.55, 0.58))로 돌아 467.6 s Q2. 판 PASS 12/12, 571.3 s, 위반 0, near 0, min_gap 0.164. 514 s의 BLOCKED 4표본((0.14, 0.27), E-1 부근)은 다른 장면이다.
- 956140d..9261ad5에서 바뀐 Nav2 값은 global costmap뿐이다(pad 0.03->0.01, res 0.05->0.025, inflation 6.0/0.25 -> 15.0/0.15). FollowPathGoal 값(`pode/pode_avoidance/params/avoidance.yaml`)과 local costmap은 그대로다. BT는 Clear 노드 4개의 `server_timeout` 20 -> 1000 ms(1c8858e, 11e9aba).
- Gazebo는 peer 층(`peer_obstacles` 기본 true)이 global과 local costmap 둘 다에 있다. 다른 로봇을 반지름 0.08 m 원으로 찍는다(5 Hz).

## 3. 판단

- [확정] 이 장면을 막힘 없이 지난 판은 A와 aI 실험 판(각 한 번)뿐이다. 사용자가 기억한 판이 그것인지는 기록만으로 가를 수 없다. 옛 기본값(base)도 이 장면에서 막혔다. main에 합친 뒤 다섯 판은 모두 막혔다. 주문(420 s)부터 Q2까지 main 다섯 판 60~176 s, wedge_noclr 48 s(47.5), corr_aI 28 s, corr_a 13 s, corr_base 110 s다.
- [확정] 막힌 판(main 다섯)은 표의 로그 구간에서 planner가 한 번도 실패하지 않고, controller가 "collision ahead"로 patience를 넘겨 follow_path를 끊는 일을 14~60번 되풀이했다. clear 20 ms 판(corr_aI, wedge_noclr)은 planner가 1~7번 길을 못 만들었고 controller patience 초과는 0이다.
- [추측] clear 1000 ms가 이 차이와 이어져 있다. 그것만 바꾼 wedge_noclr에서 모양이 aI 쪽으로 바뀌었다. 그러나 한 판이고, 어떤 순서로 이어지는지는 모른다. 앞서 낸 설명("aI에서 clear가 시간 초과로 끝나 pode3 자국이 남아 planner가 거절했다")은 틀렸다: corr_aI 로그에서 planner 실패(628.781)가 clear 시간 초과(628.822, 628.878)보다 먼저다(Codex 리뷰). `KeepoutFilter: New filter mask arrived`도 clear 완료의 증거가 아니다.
- [추측] 막힘의 모양은 planner(global costmap)와 controller(local costmap, 다각형 footprint)의 판단 불일치로 보인다. 계획 경로(`/plan`)와 costmap 상태를 기록하지 않아 매번 같은 틈 길이었는지, 충돌 대상이 pode3였는지는 확인하지 못했다.
- [확정] clear 20 ms에서도 끼임이 0은 아니다: wedge_noclr도 약 20 s 서 있다가 관제 길 바꿈으로 빠졌다.

## 3-1. 녹화 판 두 개로 뒤집힌 판단 (18:38~18:58)

실행 `gz/run_rec.sh`: 판마다 pode2의 `/plan`, `/behavior_tree_log`, global/local costmap(+updates), `/pode2/peer_obstacles`, pode2·pode3 `sim/truth`, `/clock`을 bag(mcap)으로 남겼다. 녹화기는 판 끝에 강제로 끝나 metadata가 없다. mcap 파일을 직접 연다. 분석 `gz/wedge_bag.py`.

| 판 | 설정 | P-3 장면 | 주문 -> Q2 | BLOCKED 표본 | 막힘 표시 | NO_START | planner 실패 | patience 초과 | collision ahead | 판 |
|---|---|---|---|---|---|---|---|---|---|---|
| noclr2 | clear 20 ms | 없음: pode3가 P-2 (0.38, 0.11)에 섰고 P-3는 비었다. pode2는 P-1 -> E-1 -> Q2를 11.5 s에 지났다 | - | 0 | 0 | 0 | - | - | - | PASS 12/12, 571.9 s(마지막 사건) |
| main_rec | clear 1000 ms(main 그대로) | 있음: pode3 (0.63, 0.11) | 47.7 s | 18 | 1 | 1 | 0 | 9 | 317 | PASS 12/12, 572.0 s, near 1, min_gap 0.139 |

main_rec 녹화 [확정]:
- 421.03 s부터 1 Hz로 ComputePathToPose가 모두 성공했고, 그 길이 P-3 앞 틈(x 0.58~0.80, y 0.15~0.37)을 지났다(418~470 s 계획 39개 중 28개). 첫 global clear(`ClearGlobalCostmap-Subtree`)는 426.84 s다. 이 bag의 BehaviorTreeLog 사건 시각은 sim 시간이 아니라 벽시계라서, 트리 로그를 bag 순서대로 읽으며 그 시점의 `/clock`에 맞춰 얻었다(`wedge_bag.py`로 읽은 값이 아니다). bag에는 costmap_updates 메시지도 없다(`always_send_full_costmap`이 true). 즉 planner는 **지우기 전부터** 틈으로 길을 그렸다.
- 틈으로 그린 길은 pode3 실제 중심에서 0.122~0.156 m를 지났다(틈을 지난 계획 28개 전부 다시 계산). pode2는 426 s부터 (0.67, 0.26)에 섰다.
- 그때 global costmap(OccupancyGrid 0~100, 99 = inscribed, 100 = lethal)의 틈 세로줄(y 0.19~0.36) 최솟값: x 0.66에서 85, x 0.70에서 72, x 0.74에서 62(내부 비용으로는 각각 약 219~220, 185~187, 159~161). 틈 가운데가 inscribed 아래라 NavFn은 지날 수 있다고 본다.
- 456.9 s에 pode2가 (0.54, 0.30)으로 나온 뒤 길이 북쪽(최대 y 0.58)으로 바뀌었다.

판단:
- [확정] main_rec(1000 ms)는 이 장면에서 47.7 s, wedge_noclr(20 ms)는 47.5 s로 같다. noclr2는 장면이 없어 비교에 못 쓴다.
- [확정] planner가 틈을 고른 것은 clear 전이다. "clear 대기 때문에 틈으로 다시 들어간다"는 앞의 가설은 이 녹화 판 하나(설정마다 한 판)에서는 맞지 않는다. 다른 판에서도 그런지는 모른다.
- [추측] main 다섯 판(60~176 s)과 wedge_noclr(48 s)의 차이는 clear 대기보다 판마다 다른 것(pode3가 선 자리, pode2가 틈에 들어간 깊이)으로 보는 편이 맞다. 해결안 나는 뒤로 뺀다.
- [추측] 뿌리는 global costmap이 틈을 지날 수 있다고 보는 것(가장 좁은 곳 내부 비용 약 219~220/253)과, controller가 다각형 몸과 라이다 점으로 "부딪힘"을 보는 것의 불일치다. 해결안 가를 먼저 한다.

## 3-2. 그 뒤 원인 확인과 고침 (19:40~21:40)

원인 확인 [확정]:
- 녹화 420~458 s를 RPP식 테두리 검사로 재연한 범위에서, 멈춘 38/55 중 34장은 peer로 분류됐고(맞춘 peer 점이 겹친 칸이다. 어느 층이 막았는지는 보지 않았다) 4장은 scan/other다(진열대인지는 가르지 못했다). 실제 경고 전체의 칸 위치는 로그에 없어 모른다. Gazebo에서 로봇 라이다(높이 0.125 m)는 다른 Pode를 라이다 받침(약 5.6 cm)으로만 본다. 그래서 몸 칸은 peer 층(원 r 0.08)이다.
- [추측] pode3 동쪽 "덩어리"는 이 원이다. local costmap에서 약 2~3.5 cm 밀려 있었다(lethal 칸에 맞춘 peer 점 이동값 dx 0.02~0.035 m, 구간마다 다르고 회전은 못 정함, `phantom/fit_peer.txt`). 이 값에는 pode2 AMCL 오차와 맞춤 오차가 함께 들어 있다. 옛 칸막이, keepout, 라이다 잡음 쪽은 이 자료로 따로 가르지 않았다.
- [확정] 녹화 costmap에서 RPP 검사를 재연하면 padding 0.01은 38/55, 0.0은 6/55에서 멈춘다(자료 scratchpad `phantom/`).
- [추측] local_costmap에 footprint_padding이 없어 Jazzy 기본 0.01이 붙은 것이 직접 원인이다. 재연은 기록한 costmap에 추정 자세를 쓴 가정 계산이다. 실제 반복 시험에서도 padding 0은 그대로와 같은 2/5 실패였다.

P-3 장면 반복 시험 [확정]: 브랜치 `exp/p3-scene-test`, 자료 scratchpad `p3scene_runs/main2`.
- pode1을 P-1에 두면 모든 설정이 5/5 통과했다.
- 틈으로 몰아넣은 5판의 결과는 아래와 같다. 실패는 pode3가 y 0.14일 때만 났다.

| 설정 | 실패 | 최장 정지 | 경고 | 비고 |
|---|---|---|---|---|
| 그대로 | 2/5 | 30.7 s | 333줄 | 진열대 -2.4 mm |
| padding 0 | 2/5 | 9.0 s | 134줄 | |
| padding 0 + peer 0.075 | 1/5 | 1.3 s | 92줄 | |

- 다각형 기준으로 로봇끼리 겹침은 없었다(pode2-pode3 최소 3.9 cm).
- 물리 모양(다각형 + 바퀴 충돌 구 + screen_mount 상자) 기준으로도 pode2-pode3 최소 간격은 0.018 m(C3/B)이고 계산 간격은 양수였다.
- 물리 모양으로 계산한 pode2와 shelf_2의 최소 간격이 음수인 판이 B 묶음에서 C0 3판, C1 1판, C2 1판, C3 1판이다. A 묶음은 없었다. 음수는 접촉했을 가능성이 크다는 뜻이다. 접촉 센서가 아니라 truth 자세와 근사한 모양으로 계산한 값이다(자료 `p3scene_runs/main2/analysis_phys.txt`).
- 시뮬 바퀴 충돌은 구(r 0.028, 중심 |y| 0.0486)라 옆으로 |y| 0.0766까지 나온다. 실물과 다각형은 0.056이다.
- [추측] 좁은 틈에서 시뮬이 실물보다 더 걸릴 수 있다.

결정(사용자 2026-11-17, "지금까지 정보로 결정"):
- 브랜치 `fix/local-padding-zero`에 local padding 0(bc5adb8)과 peer 0.075를 넣었다. main은 아니다.
- 실물 적용은 보류한다. 고정 경로 컨트롤러도 이 발자국을 쓴다.

## 4. 해결안(검토 중, 정하지 않음)

| 안 | 내용 | 걱정 |
|---|---|---|
| 가. peer 층 맞추기 | global costmap의 다른 로봇 크기(지금 원 0.08 + inflation 15/0.15)를 controller가 보는 만큼 맞춰, 틈이 안 맞으면 planner가 처음부터 다른 길을 그리게 한다. 틈은 막지 않는다 | 너무 크면 갈 수 있는 길과 출발 자리가 막힌다. 먼저 잰다 |
| 나. clear 시간 다루기 | clear 뒤 다시 표시될 때까지 기다린 뒤 길을 그리거나, global clear만 1000 ms를 빼고 local은 둔다 | 1000 ms는 shelf_3 모서리 멈춤 고침으로 넣었다. 그 장면을 다시 봐야 한다 |
| 다. 관제 우회 | P-3에 로봇이 서 있으면 P-1/P-2에서 동쪽 가는 길을 O_IN -> M_E로 | 지나갈 수 있는 길을 조건부로 안 쓴다(지름길 원칙). 가운데 통로 혼잡 |
| 라. 첫 막힘에 다른 길 | 첫 막힘 표시에서 물리적으로 다른 길 | 깊이 끼면 O_IN 목표도 NO_START(watch_main, div3_2) |

다음 순서(18:38 전 제안, 3-1로 나는 뒤로 뺐다): 계획 경로와 clear 요청·응답, peer 재표시 시각을 기록하는 판을 돌리고, clear 20 ms 판을 한 번 더 돌린다. 그 결과로 가를 먼저 맞추고, clear 뒤 공백이 원인으로 보이면 나를 더한다. 다는 대비책.

## 5. 참조 데이터(임시 폴더, 사라질 수 있다)

scratchpad `gz/wedge_noclr/`, `gz/wedge_noclr.log`, 실행 `gz/run_wedge.sh`(div3 설치본 위에 `exp/wedge-clear-timeout-off`의 pode_nav_config를 얹음), 판단 초안 `judgment_wedge_cause.md`와 Codex 리뷰(`~/codex-review.md` "wedge cause and fixes judgment").
