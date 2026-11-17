# P-3 앞 남쪽 통로 끼임 진단 (2026-11-17)

사용자: "원래 전에는 통과했던거같은데 왜갑자기 멈추지", "왜 갑자기 막히는지 그리고 해결방안도 찾아놔".
브랜치 `exp/wedge-clear-timeout-off`(진단용, main에 넣지 않는다). 바탕은 `feat/v7-no-divider3`(새 배치).

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
| **wedge_noclr** | main + 새 배치, **clear 20 ms(이 브랜치)** | **467.6** | **19** | **1** | **0** | **7** | **7** | **0** | **64** |

- corr_* 판은 브랜치 `exp/corridor-costmap-ab` 680fda8 빌드(aI는 nav_overrides)다. main 판과 다른 점은 clear 시간만이 아니다(keepout 틈, BLOCKED 유지, fleet 타이머도 없다).
- wedge_noclr는 main 판과 다른 점이 clear 시간 하나다(BT `server_timeout` 1000 -> 생략 = bt_navigator `default_server_timeout` 20 ms).
- wedge_noclr의 pode2: 428 s (0.68, 0.28)까지 틈에 들어갔다가 RECOVERING으로 (0.62, 0.38)(진열대 B 서쪽 끝 바깥)까지 물러났다. 431~451 s 그 자리에 섰고(441~450 s BLOCKED), 451 s 관제가 E-2->E-3를 막힘 표시하고 길을 다시 주자 북쪽 가운데 통로((0.52, 0.43) -> (0.96, 0.57) -> (1.55, 0.58))로 돌아 467.6 s Q2. 판 PASS 12/12, 571.3 s, 위반 0, near 0, min_gap 0.164. 514 s의 BLOCKED 4표본((0.14, 0.27), E-1 부근)은 다른 장면이다.
- 956140d..9261ad5에서 바뀐 Nav2 값은 global costmap뿐이다(pad 0.03->0.01, res 0.05->0.025, inflation 6.0/0.25 -> 15.0/0.15). FollowPathGoal 값(`pode/pode_avoidance/params/avoidance.yaml`)과 local costmap은 그대로다. BT는 Clear 노드 4개의 `server_timeout` 20 -> 1000 ms(1c8858e, 11e9aba).
- Gazebo는 peer 층(`peer_obstacles` 기본 true)이 global과 local costmap 둘 다에 있다. 다른 로봇을 반지름 0.08 m 원으로 찍는다(5 Hz).

## 3. 판단

- [확정] "전에 통과했다"는 A와 aI 실험 판(각 한 번)이다. 옛 기본값(base)도 이 장면에서 막혔다. main에 합친 뒤 다섯 판은 모두 막혔다(첫 BLOCKED부터 Q2까지 47~165 s).
- [확정] 막힌 판(main 다섯)은 planner가 한 번도 실패하지 않고, controller가 "collision ahead"로 patience를 넘겨 follow_path를 끊는 일을 14~60번 되풀이했다. clear 20 ms 판(corr_aI, wedge_noclr)은 planner가 1~7번 길을 못 만들었고 controller patience 초과는 0이다.
- [추측] clear 1000 ms가 이 차이와 이어져 있다. 그것만 바꾼 wedge_noclr에서 모양이 aI 쪽으로 바뀌었다. 그러나 한 판이고, 어떤 순서로 이어지는지는 모른다. 앞서 낸 설명("aI에서 clear가 시간 초과로 끝나 pode3 자국이 남아 planner가 거절했다")은 틀렸다: corr_aI 로그에서 planner 실패(628.781)가 clear 시간 초과(628.822, 628.878)보다 먼저다(Codex 리뷰). `KeepoutFilter: New filter mask arrived`도 clear 완료의 증거가 아니다.
- [추측] 막힘의 모양은 planner(global costmap)와 controller(local costmap, 다각형 footprint)의 판단 불일치로 보인다. 계획 경로(`/plan`)와 costmap 상태를 기록하지 않아 매번 같은 틈 길이었는지, 충돌 대상이 pode3였는지는 확인하지 못했다.
- [확정] clear 20 ms에서도 끼임이 0은 아니다: wedge_noclr도 약 20 s 서 있다가 관제 길 바꿈으로 빠졌다.

## 4. 해결안(검토 중, 정하지 않음)

| 안 | 내용 | 걱정 |
|---|---|---|
| 가. peer 층 맞추기 | global costmap의 다른 로봇 크기(지금 원 0.08 + inflation 15/0.15)를 controller가 보는 만큼 맞춰, 틈이 안 맞으면 planner가 처음부터 다른 길을 그리게 한다. 틈은 막지 않는다 | 너무 크면 갈 수 있는 길과 출발 자리가 막힌다. 먼저 잰다 |
| 나. clear 시간 다루기 | clear 뒤 다시 표시될 때까지 기다린 뒤 길을 그리거나, global clear만 1000 ms를 빼고 local은 둔다 | 1000 ms는 shelf_3 모서리 멈춤 고침으로 넣었다. 그 장면을 다시 봐야 한다 |
| 다. 관제 우회 | P-3에 로봇이 서 있으면 P-1/P-2에서 동쪽 가는 길을 O_IN -> M_E로 | 지나갈 수 있는 길을 조건부로 안 쓴다(지름길 원칙). 가운데 통로 혼잡 |
| 라. 첫 막힘에 다른 길 | 첫 막힘 표시에서 물리적으로 다른 길 | 깊이 끼면 O_IN 목표도 NO_START(watch_main, div3_2) |

다음 순서(제안): 계획 경로와 clear 요청·응답, peer 재표시 시각을 기록하는 판을 돌리고, clear 20 ms 판을 한 번 더 돌린다. 그 결과로 가를 먼저 맞추고, clear 뒤 공백이 원인으로 보이면 나를 더한다. 다는 대비책.

## 5. 참조 데이터(임시 폴더, 사라질 수 있다)

scratchpad `gz/wedge_noclr/`, `gz/wedge_noclr.log`, 실행 `gz/run_wedge.sh`(div3 설치본 위에 이 브랜치의 pode_nav_config를 얹음), 판단 초안 `judgment_wedge_cause.md`와 Codex 리뷰(`~/codex-review.md` "wedge cause and fixes judgment").
