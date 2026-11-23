# nav2fleet 보관 브랜치 17개 (2026-11-22)

## 이 문서는

- nav2fleet 저장소(`PJW-1/roscamp-nav2fleet`)는 11-22에 RPP 주행의 되돌아갈 기준으로 얼렸다. 그 `main`(b9e86c5)에 주석 태그 `rpp-baseline-2026-11-21`을 붙였다.
  - 2026-11-23 사용자 결정으로 얼림을 풀었다. 지금 nav2fleet `main`은 이 저장소 `main`을 `learning/` 없이 따라간다(README "저장소 두 개", `tools/sync_nav2fleet.sh`). 태그는 그대로 둔다.
- nav2fleet의 원격 브랜치 가운데 `main`에 없는 커밋을 가진 17개를 이 저장소(pode-drive)에 `archive/nav2fleet/<원래 이름>`으로 그대로 옮겼다. 커밋은 바꾸지 않았고 끝 SHA는 두 저장소에서 같다.
- **아무것도 병합하지 않았다.** 이 문서는 각 브랜치가 무엇이고, 왜 main에 들어가지 않았고, 언제 다시 꺼내 볼지를 적는다.
- 역사가 이어진다. pode-drive `main`은 nav2fleet b9e86c5를 조상으로 가진다(`git merge-base --is-ancestor b9e86c5 main` 통과, 092fdc4 기준). 그래서 보관 브랜치는 보통 브랜치처럼 비교하고 병합할 수 있다.

꺼내는 방법:

```bash
git fetch origin
git switch -c <새 이름> origin/archive/nav2fleet/<원래 이름>
```

보관 브랜치 자체는 고치지 않는다. 일은 꺼낸 새 브랜치에서 한다.

읽는 법:

- "근거"는 기록이 있는 자리다. `main:`은 이 저장소 main의 파일, `<브랜치>:`는 그 보관 브랜치에만 있는 파일이다. 행 번호는 092fdc4 기준이다.
- 기록이 없으면 "기록 없음"이라 쓰고, 짐작은 [추측]으로 표시했다.
- "main과 충돌"은 `git merge-tree --write-tree main <브랜치>`로 092fdc4에 대해 잰 것이다. 충돌 없음이 동작이 맞는다는 뜻은 아니다.

## 한눈에

| 보관 이름 (`archive/nav2fleet/` 뒤) | 끝 SHA | 한 줄 설명 | main과 충돌 |
|---|---|---|---|
| `docs/real-two-robot-run-2026-11-18` | a5752c9 | 11-18 두 대 실물 시험 결과와 진단(문서 7절) | 없음 |
| `docs/worklog-run7-map-2026-11-18` | ccdff5a | 7차 실물 준비(SLAM 재측정, AMCL 지도 테두리, 벽 쪽 경로) 기록 | 없음 |
| `exp/p3-scene-test` | 7dbe10a | P-3 끼임 장면 반복 시험 도구(`tools/p3scene`)와 Gazebo 덮기 파일 | 없음 |
| `exp/wedge-clear-timeout-off` | 00b3e9e | nav_goal BT의 costmap clear 대기를 끈 진단 실험 | 파일 1개 |
| `feat/goal-plan-keeper` | 771c971 | "도리도리" 줄이기: 계획 요청과 planner 사이의 `pode_plan_keep` 노드 | 파일 1개 |
| `feat/nav-goal-bt-backup` | 1c34379 | nav_goal BT 복구에 짧은 BackUp 넣기(Spin 없음) | 파일 2개 |
| `feat/nav-goal-smooth` | e26f221 | goal 컨트롤러 `min_lookahead_dist` 0.15 + `rotate_to_heading_min_angle` 0.45 | 없음 |
| `feat/real-rviz-tests2-2026-11-18` | f815012 | 실물 RViz 실행 스크립트 시험 보강(시험만) | 파일 1개 |
| `feat/relocalize-fleet` | b0bb6b8 | 관제 쪽 자동 위치 다시 찾기(Relocalize)와 suspect_hold | 파일 21개 |
| `feat/relocalize-robot` | 4ac4cbd | 로봇 쪽 Relocalize 액션 서버(pode_fsm)와 Spin 복귀 | 파일 16개 |
| `feat/rename-pode` | 279082f | 이름 변경 뒤 로봇 재배포 기록(문서 7절 하나) | 없음 |
| `fix/goal-replan-on-invalid` | b33e426 | 도리도리를 BT만으로 막으려던 버린 실험(3 s 계획 유지) | 파일 1개 |
| `fix/limit-localizing-isolation-2026-11-18` | a8d3566 | 재시도 한도에 닿은 로봇이 LOCALIZING 한 번에 격리되는 결함 수정 | 없음 |
| `refactor/yield-a3-fake-ladder` | 25664cf | 가짜 로봇이 로봇 쪽 양보 사다리를 돌게 함 | 파일 2개 (내용은 이미 main에 있음) |
| `test/real-plan-keeper-2026-11-18` | 767c385 | 실물 두 대 시험용: main + plan_keep + `PLAN_KEEP=0` 스위치 | 없음 |
| `test/real-plan-keeper-2026-11-19` | 7f065a5 | 같은 목적의 11-19판(3d4254c 바탕) | 없음 |
| `tools/yield-report` | bf7848c | 양보 기록 집계 도구 `tools/yield_report.py` | 파일 1개 (내용은 이미 main에 있음) |

## 이미 main에 내용이 있는 두 브랜치

`tools/yield-report`와 `refactor/yield-a3-fake-ladder`는 커밋 SHA만 다르고 같은 변경이 main에 들어 있다.

- `git cherry main <브랜치>`가 두 브랜치의 커밋 5개를 모두 `-`(같은 패치가 main에 있음)로 낸다.
- `git patch-id --stable`로 짝을 확인했다.

| 보관 브랜치의 커밋 | main의 같은 패치 | 제목 |
|---|---|---|
| bf7848c | c1aff06 | tools(yield): report LOCAL_YIELD ladders and HALL_YIELD timing from fleet records |
| f5ce5c1 | d7f80ed | refactor(yield): let headless fake robots run the robot-side yield ladder |
| 65faa60 | 5245958 | tools(yield): add --local-yield to situations.py |
| dc43cf0 | 5efea6c | refactor(yield): count a nav_goal segment's drive from its own start |
| 25664cf | 5b46836 | refactor(yield): start a segment's trail where the segment started |

- main 쪽 커밋은 병합 커밋 ccb7336("Merge refactor/zone-cleanup (part 1 steps 5-6) into refactor/yield-owner", 11-18)으로 들어왔다. 그날 밤 기록 `main:docs/worklog_overnight_2026-11-18.md` 33-34행, 239행이 두 브랜치를 그 밤의 A1, A3 단계로 적는다.
- `merge-tree`가 충돌을 내는 것은 그 뒤 main에서 같은 파일을 다시 고쳤기 때문이다(예: 14f3476 "drop work traces from comments"). 이 두 브랜치는 다시 꺼낼 일이 없다.

## 브랜치별

### docs/real-two-robot-run-2026-11-18 (a5752c9)

- 무엇: `docs/worklog_real_two_robot_test_2026-11-18.md`에 "7. 실물 시험 결과 (2026-11-18 오후)"를 더한 문서 커밋 3개. 세 판의 결과, LOCALIZING 때문에 격리된 판, 판 3의 멈춤 두 번, RPP 충돌 검사와 관제 재시도에 대한 진단, 선택지(7.7절)가 있다. main의 같은 파일은 6절에서 끝난다.
- 병합하지 않은 이유: 기록 없음.
- 참고: 7.7절 1번(실물 고정 경로 충돌 검사 끄기)은 그 절에 적힌 대로 a14b2e5로 main에 들어갔다. 2번(LOCALIZING 동안 재시도를 쓰지 않기)과 같은 방향의 수정이 `main:docs/worklog_fleet_wait_localizing_2026-11-18.md`(a190119, a254e58)로 main에 있다.
- 다시 꺼낼 때: 기록 없음. [추측] 11-18 오후 실물 판의 원래 관찰과 진단이 필요할 때(예: 실물 격리나 RPP 정지 문제를 다시 볼 때). 문서뿐이라 충돌 없이 넣을 수 있다.

### docs/worklog-run7-map-2026-11-18 (ccdff5a)

- 무엇: 7차 실물 시험 준비 기록 `docs/worklog_run7_map_paths_2026-11-18.md` 한 파일(문서만, 시험 브랜치 `test/real-run7-2026-11-18`의 ce116ea에서 갈림). 기록이 설명하는 코드 커밋 502b227, 77725b3, 01b2fe4와 ce116ea는 모두 main에 있다. 문서만 main에 없다.
- 병합하지 않은 이유: 기록 없음.
- 다시 꺼낼 때: 기록 없음. [추측] 실물 지도의 1 cm 테두리, 벽 쪽 경로를 왜 그렇게 잡았는지 근거가 필요할 때. 문서 9절에 남은 실측(벽 틈, 경기장 치수)이 적혀 있다.

### exp/p3-scene-test (7dbe10a)

- 무엇: P-3 앞 끼임 장면을 반복해 돌리는 도구(`tools/p3scene/`: 실행기, 판 운전기, 분석)와 비교용 `nav_overrides`(local padding 0, 그 위에 global 0.02), `peer_radius_m` 인자. 마지막 커밋의 C4 덮기 파일은 커밋 메시지에 "보류, 돌리지 않음"으로 적혀 있다.
- 병합하지 않은 이유: "P-3 장면 시험 도구다. 실험용이라 넣지 않았다." (`main:docs/worklog_merge_real_test_2026-11-17.md` 2절, 22행). 이 도구로 낸 결과는 `main:docs/worklog_p3_gap_fix_2026-11-17.md` 3절에 있고, 그 결과로 정한 설정(local padding 0, peer 0.075, 브랜치 `fix/local-padding-zero`)은 main에 들어갔다(`main:docs/worklog_merge_real_test_2026-11-17.md` 1절 표, 12행).
- 다시 꺼낼 때: 기록 없음. [추측] 좁은 틈 끼임을 같은 장면으로 다시 재야 할 때(예: footprint, padding, peer 반지름을 바꿀 때).

### exp/wedge-clear-timeout-off (00b3e9e)

- 무엇: nav_goal BT의 Clear 노드 4곳의 `server_timeout` 1000 ms를 빼서 기본 20 ms로 되돌린 진단 실험(907e6d3)과 진단 기록.
- 병합하지 않은 이유: "그 브랜치의 코드(BT clear 대기 끔)는 실험이라 넣지 않았다" (`main:docs/worklog_merge_real_test_2026-11-17.md` 1절 표, 13행). 기록 문서는 main에 들어갔고(70053bf), main 쪽에서 그 뒤 문구를 더 고쳤다(6f09bc8, b9c1136). 기록 자체도 "clear 대기 때문에 틈으로 다시 들어간다"는 가설이 녹화 판에서 맞지 않았다고 적는다(`main:docs/worklog_wedge_p3_diagnosis_2026-11-17.md` 3-1절, 57행).
- 다시 꺼낼 때: 기록 없음. [추측] BT의 clear 대기 시간이 다시 원인 후보로 오를 때. 기록 문서는 main 쪽이 더 새 판이니 브랜치의 문서는 가져오지 않는다.

### feat/goal-plan-keeper (771c971)

- 무엇: 길이가 비슷한 두 길 사이에서 1 Hz 재계획이 번갈아 바뀌어 로봇이 고개를 흔드는 "도리도리"를 줄인다. 계획 요청과 planner 사이에 rclpy 노드 `pode_plan_keep`을 두어, 새 경로가 다른 길이어도 지금 길이 막히지 않았고 새 길이 확실히 짧지(0.30 m 또는 20%) 않으면 지금 길의 남은 부분을 돌려준다(기록 3절). sim과 실물 launch에 `plan_keep` 인자를 넣었다.
- 병합하지 않은 이유: `feat/goal-plan-keeper:docs/worklog_goal_plan_keeper_2026-11-18.md` 6절. 실물에서 돌려 보지 않았고, Gazebo에서 PC 부하가 높던 판은 복구가 많았는데 원인을 가르지 못했다. 로봇 배포가 달라지고(새 패키지), 시험이 한 번 원인 모르게 멈췄다. 같은 내용이 `main:docs/worklog_overnight_2026-11-18.md` 134-136행에 있고, 같은 문서 170, 254, 269행은 "실물에서 한 번 돌려 보고 정한다"고 권한다.
- 다시 꺼낼 때 (사용자 결정): 순서 3번 keeper 평가 때 본다(미뤄 둠).

### feat/nav-goal-bt-backup (1c34379)

- 무엇: nav_goal BT 복구에 짧은 BackUp을 넣는다(Spin은 넣지 않음). 시험은 Gazebo 두 판(bk1, bk2).
- 병합하지 않은 이유: `feat/nav-goal-bt-backup:docs/worklog_bt_backup_2026-11-17.md` 4절, 5절. 낀 로봇이 물러나도 곧 같은 틈으로 다시 들어갔고, 주문→Q2 시간이 BackUp 없는 판과 거의 같았다. 계약(`docs/Pode_계약.md`)이 "nav_goal BT에 Spin·BackUp 없음"을 기준으로 적으므로 넣으려면 계약을 함께 고쳐야 한다. `main:docs/worklog_merge_real_test_2026-11-17.md` 2절 21행도 "끼임을 줄인다는 근거가 없고, 계약서도 BackUp을 쓰지 않는다"고 적는다.
- 다시 꺼낼 때 (사용자 결정): 계약을 바꾸는 일이라 사용자가 정해야 한다. 기록 5절은 그 전에 "물러난 뒤 같은 틈으로 다시 가지 않게 하기"를 먼저 하고 다시 재라고 적는다.

### feat/nav-goal-smooth (e26f221)

- 무엇: O_OUT을 나온 뒤 주행이 뚝뚝 끊기는 것을 줄인다. planner는 NavFn 그대로이고, goal 컨트롤러(`pode_avoidance/params/avoidance.yaml`)의 `min_lookahead_dist` 0.15와 `rotate_to_heading_min_angle` 0.45를 함께 준다.
- 병합하지 않은 이유: 사용자 지시 "이 부분만은 마지막에 작업해서 브랜치를 따로 설정해놔 main말고" (`feat/nav-goal-smooth:docs/worklog_nav_goal_smooth_2026-11-17.md` 머리말). 부작용은 주차 칸에 들어갈 때 판마다 2~4 s 멈춤이 한 번 있고, 장애물 판에서는 나아지지 않았다(커밋 6965df4 메시지, 같은 기록 4~5절, `main:docs/worklog_merge_real_test_2026-11-17.md` 2절 20행, `main:docs/worklog_drive_handoff_2026-11-17.md` 6절 끝과 8절).
- 다시 꺼낼 때 (사용자 결정): 회피 담당 영역이다. 사용자가 마지막에 따로 작업한다.

### feat/real-rviz-tests2-2026-11-18 (f815012)

- 무엇: `tools/test/test_real_rviz.py`에 시험 두 개(`test_launcher_setup_files_may_read_unset_vars`, `test_launcher_default_overlay_path`)와 인자 경계 확인을 더한다. 시험 파일만 바꾼다. 바탕 ffd30d7은 main에 있다.
- 병합하지 않은 이유: 기록 없음. [추측] 바로 뒤 main에서 디스커버리 서버 작업(212d1f7, cf8c5a1, `main:docs/worklog_discovery_server_2026-11-19.md`)이 같은 시험 파일과 실행 스크립트를 바꿨다. 지금 main과는 그 파일에서 충돌한다.
- 다시 꺼낼 때: 기록 없음. [추측] 실물 RViz 실행 스크립트(`rviz_pc.sh`)를 다시 고칠 때. 두 시험이 지금 스크립트에도 맞는지 먼저 본다.

### feat/relocalize-fleet (b0bb6b8)

- 무엇: 관제가 로봇을 격리하기 전에 `/{pode}/relocalize`를 불러 선 채로 위치를 다시 찾게 한다(계기: 의심 검사 실패, DOCK_FAILED 등). 위치 의심 동안 구역을 잡아 두는 `suspect_hold`, Relocalize 액션 정의, fake_robots 흉내를 함께 담는다.
- 병합하지 않은 이유: 사용자 결정(2026-11-16)으로 "위치 튐 감지 + 위치 다시 찾기"(B안)를 멈추고 보관했다. 작업은 끝나지 않았고 Gazebo 종단 시험은 하지 않았다 (`feat/relocalize-fleet:docs/worklog_relocalize_fleet_2026-11-16.md` 머리말). `main:docs/worklog_merge_real_test_2026-11-17.md` 2절 23행: "B안이고 병합하지 않는다". 같은 기록 끝 절 "보관 시점 Codex 지적"에 검증하지 않은 지적이 남아 있다.
- 다시 꺼낼 때 (사용자 결정): 실물 AMCL 오차 측정(11-23 예정)에서 정상 주행 중에도 큰 위치 오차가 나오면 다시 본다. main과 충돌이 많다(파일 21개).

### feat/relocalize-robot (4ac4cbd)

- 무엇: 로봇 쪽 Relocalize 액션 서버(`pode_fsm/relocalize.py`, `relocalize_ros.py`). 무이동 갱신, 필요하면 한 바퀴 Spin, 전역 재초기화 순서로 위치를 다시 잡는다. Relocalize용으로 Nav2 Spin을 되살리고 AMCL recovery를 켰다. 마지막 커밋은 WIP다(cff6128 "park the work").
- 병합하지 않은 이유: `feat/relocalize-fleet`과 같다(`feat/relocalize-robot:docs/worklog_relocalize_robot_2026-11-16.md` 머리말, `main:docs/worklog_merge_real_test_2026-11-17.md` 23행).
- 다시 꺼낼 때 (사용자 결정): `feat/relocalize-fleet`과 같다. main과 충돌이 많다(파일 16개).

### feat/rename-pode (279082f)

- 무엇: 이름 변경(pinky → Pode) 뒤 로봇 pinky_27a6에 7c7d04b를 다시 배포한 기록. `docs/worklog_rename_pode_2026-11-16.md`에 "7. 로봇 다시 배포" 절을 더한 문서 커밋 하나다. 이름 변경 자체는 main에 있다.
- 병합하지 않은 이유: "다른 세션의 문서다." (`main:docs/worklog_merge_real_test_2026-11-17.md` 2절 24행).
- 참고: main의 같은 파일 7절(108행)은 아직 "로봇에는 옛 이름이 빌드되어 있다"고 적는다. 그 뒤 실물 시험에서 로봇에 새로 배포했으므로(예: `main:docs/worklog_run8_w_entry_arc_2026-11-18.md` 49행) 그 문장은 지금 상태가 아니다.
- 다시 꺼낼 때: 기록 없음. [추측] 11-16 재배포 과정이 필요할 때만. 문서뿐이라 충돌 없이 넣을 수 있다.

### fix/goal-replan-on-invalid (b33e426)

- 무엇: 도리도리를 BT만으로 막는 시도. goal이 바뀌었거나 `IsPathValid`가 실패하거나 계획이 3 s 지났을 때만 다시 계획한다.
- 병합하지 않은 이유: 버린 실험이다. `feat/goal-plan-keeper:docs/worklog_goal_plan_keeper_2026-11-18.md` 2절 표: 무효일 때만 다시 계획하면 Jazzy 도착 판정 때문에 목적지 0.045 m 앞에서 멈췄고, 3 s 타이머를 더한 판(이 브랜치)은 `IsPathValid`가 자주 실패해 뒤집힘이 줄지 않았다(장애물 판 9번). `main:docs/worklog_overnight_2026-11-18.md` 136행, 240행("버린 실험"). keeper 브랜치와 같은 BT 줄을 바꾸므로 함께 넣으면 충돌한다(`test/real-plan-keeper-2026-11-18:docs/worklog_real_plan_keeper_test_2026-11-18.md` 5절).
- 다시 꺼낼 때 (사용자 결정): 순서 3번 keeper 평가 때 본다(미뤄 둠).

### fix/limit-localizing-isolation-2026-11-18 (a8d3566)

- 무엇: 재시도 한도에 닿은 로봇에서 NAV_ABORTED가 먼저 오고 1 s 안에 LOCALIZING이 오면 바로 격리되던 결함을 고친다. 마지막 NAV_ABORTED를 `LOCALIZING_GRACE_S` 동안 미뤄 두고, 그 안에 LOCALIZING이 오면 판정을 지운다. 시험은 `pc/fleet_core/test/test_zone_wait_localizing.py`. 이 수정은 main에 없다(main `traffic/zone/base.py`에 미뤄 두는 판정이 없다).
- 병합하지 않은 이유: 기록 없음. 기록(`fix/limit-localizing-isolation-2026-11-18:docs/worklog_limit_localizing_isolation_2026-11-18.md` "확인하지 못한 것")은 실물과 Gazebo에서 돌려 보지 않았고, 실물의 NAV_ABORTED와 LOCALIZING 간격이 1 s 안인지 재지 않았다고 적는다.
- 다시 꺼낼 때 (사용자 결정): 11-23 두 대 디스커버리 시험에서 디스커버리 서버를 켜고도 LOCALIZING 뒤 격리가 나면 다시 본다. main과 충돌은 없다.

### test/real-plan-keeper-2026-11-18 (767c385)

- 무엇: 실물 두 대 시험용 브랜치. 1f1089d + `feat/real-fixed-line-check-off` + `feat/goal-plan-keeper`를 합치고, `start.sh`에 `PLAN_KEEP=0`(길 유지 끄기)을 더했다(9333407). 배포 명령은 적었지만 실행하지 않았다.
- 병합하지 않은 이유: "개인 브랜치, main에 넣지 않음". 시험용으로만 쓰고 실물 결과를 본 뒤 main을 정한다 (`test/real-plan-keeper-2026-11-18:docs/worklog_real_plan_keeper_test_2026-11-18.md` 머리말, 4.6절). 이 브랜치로 실물을 돌렸다는 기록은 찾지 못했다.
- 다시 꺼낼 때 (사용자 결정): 순서 3번 keeper 평가 때 본다(미뤄 둠). 쓸 때는 바탕이 오래됐으므로 지금 main에 `feat/goal-plan-keeper`를 다시 합치는 편이 낫다 [추측].

### test/real-plan-keeper-2026-11-19 (7f065a5)

- 무엇: 같은 목적의 11-19판. 3d4254c에 `feat/goal-plan-keeper`를 합치고(35d78ab), `PLAN_KEEP=0` 커밋을 다시 얹었다(7f065a5, 9333407과 같은 제목).
- 병합하지 않은 이유: 기록 없음(이 브랜치만의 기록 문서가 없다). [추측] 11-18판과 같은 시험용 브랜치라 같은 이유다.
- 다시 꺼낼 때 (사용자 결정): 순서 3번 keeper 평가 때 본다(미뤄 둠). 이 브랜치로 실물을 돌렸다는 기록은 찾지 못했다.

### refactor/yield-a3-fake-ladder (25664cf), tools/yield-report (bf7848c)

- 위 "이미 main에 내용이 있는 두 브랜치"를 본다. 병합하지 않은 것이 아니라 같은 패치가 다른 커밋으로 main에 들어갔다.
- 다시 꺼낼 때: 꺼낼 필요 없다. 도구 설명은 `main:docs/worklog_yield_report_2026-11-18.md`에 있다(보관 브랜치의 같은 파일과 내용이 같다).

## 기록이 없는 것 모음

- 병합하지 않은 이유의 기록이 없는 브랜치: `docs/real-two-robot-run-2026-11-18`, `docs/worklog-run7-map-2026-11-18`, `feat/real-rviz-tests2-2026-11-18`, `fix/limit-localizing-isolation-2026-11-18`, `test/real-plan-keeper-2026-11-19`.
- 다시 꺼낼 조건이 사용자 결정에도 기록에도 없어 [추측]으로 적은 브랜치: `docs/real-two-robot-run-2026-11-18`, `docs/worklog-run7-map-2026-11-18`, `exp/p3-scene-test`, `exp/wedge-clear-timeout-off`, `feat/real-rviz-tests2-2026-11-18`, `feat/rename-pode`.
