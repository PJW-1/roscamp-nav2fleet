# 작업 기록: 관제 쪽 자동 위치 다시 찾기 (2026-11-16)

> **보관 (2026-11-16):** 사용자 결정으로 B(위치 튐 감지 + 위치 다시 찾기) 작업을 멈추고 보관한다. 정상 주행에서 큰 위치 오차가 나는지 실물에서 재 본 뒤 다시 정한다. 이 브랜치는 개인 저장소에만 푸시하고 main에는 병합하지 않는다. 작업은 끝나지 않았고 Gazebo 종단 시험은 하지 않았다.

브랜치: `feat/relocalize-fleet` (`feat/relocalize`에서 만듦). 개인 저장소에 브랜치로만 푸시했고 머지는 하지 않았다.
계약: `interfaces/fleet_interfaces/action/Relocalize.action`. 바꾸지 않고 가져다 썼다(d757c7c, fc13a5c, 04234b7은 그 계약 커밋을 cherry-pick한 것이다).
`test/world-drive-by-estimate`(b1cb5c9)을 머지해 `pose_jumps_drive` 상황을 가져왔다.

## 1. 무엇을 했나

판단은 fleet_core가 하고, 실행은 로봇이 한다.

### 계기 셋
1. B의 기지 끝 의심 검사(`suspect_hold=check`, `_judge_doubt`)가 실패했을 때. 다시 시도하기 전에 한다.
2. `DOCK_FAILED`일 때. 다시 붙기 전에 한다.
3. 까닭이 `NOT_AT_TARGET`이나 `DOCK_FAILED`인 `ROBOT_ISOLATED` 직전. 재시도 예산을 다 쓴 뒤(늦은 계기)에 한다.

### 시도
- 첫 시도는 `allow_spin=true, allow_global=false`, 둘째는 `allow_global=true`이다. 그래도 안 되면 격리한다.
- 한 기회(`_head_for`부터 도킹 성공까지)에 이른 계기 한 번, 늦은 계기 한 번까지만 한다(`relocs`).

### 보내는 조건
- 로봇이 섰고 활성 goal이 없을 때만 보낸다. 이때 `settle_until`이 지났고, 상태가 MOVING/DOCKING/RETREATING이 아니어야 한다.
- 9c4b1c6부터 LOCALIZING일 때도 보내지 않는다(앞 시도가 아직 도는 중이라는 뜻이다).

### 다시 찾는 동안
- 잡은 구역은 모두 그대로 둔다.
- 새 배정, 대기 빠져나가기, 주문 늘리기, 재시도는 하지 않는다. `on_dispatch`는 `LOCALIZING`으로 거절한다.
- 로봇 보기에 `relocalizing`이 뜬다.
- 할 일 없음 경고와 침묵 경고를 내지 않는다.

### 사건
- `RELOCALIZE_SENT`: 계기, 시도 번호, spin/global, 잡은 구역.
- `RELOCALIZED`: 시작 자세에서 끝 자세까지 옮긴 거리, 그리고 자리.
- `RELOCALIZE_FAILED`: 까닭.

### 성공한 뒤
- 새 자세가 잡은 구역 안이면 의심을 풀고, 기존 재시도 길로 다시 보낸다. 재시도 예산은 쓰지 않는다(`attempts`와 `dock_tries`를 계기 직전 값으로 되돌린다). `DOCK_FAILED`였으면 다시 붙으러 간다.
- 잡은 구역 밖이면, 그 자리 구역이 비었을 때 그 구역을 잡고 거기서 다시 계획한다. 비지 않았으면 격리한다.
- 다른 로봇이 잡은 구역과 맞닿은 경계 위는 안으로 보지 않는다(eb7042f).

### 기한
- 시도 하나의 기한은 `relocalize_timeout_s + 5 s`이다.
- 보내기를 기다리는 동안에도 기한이 흐른다. 서지 않는 로봇도 끝이 난다(4f76362).
- STOP은 도는 goal을 곧바로 거둔다. PAUSE는 도는 goal은 두고 새로 보내지 않는다(eb7042f, 9c4b1c6).
- 운영자 해제(`confirm_clear`)는 다시 찾기를 거둔다.

### 결과 처리
- `BUSY`(로봇이 받자마자 끝냄)는 실패로 세지 않고, 같은 시도를 기한 안에 다시 보낸다.
- `pose_valid=false`나 자세가 없는 성공은 실패로 본다.
- `NOT_SENT`와 `NO_LOCALIZER`는 서버가 없다는 뜻이다. 다시 찾기를 그만두고 예전 길(재시도, 격리)로 돌아간다.

### 파라미터 (모두 OPTIONAL)
- `relocalize`: yaml은 true, dataclass 기본값은 false이다. 그래서 옛 기록과 옛 시험은 예전대로 돈다.
- `relocalize_timeout_s`: 30.0.
- `relocalize_attempts`: 2. 1 이상이어야 한다.

### fleet_manager
- 로봇마다 `Relocalize` 액션 클라이언트를 둔다. 이름은 계약 키 `relocalize`로 찾는다.
- 결과 매핑: `success`, `reason`, `pose`, `start_pose`. `pose_valid=false`면 두 자세 모두 None이다.
- 서버 준비 안 됨 → `NOT_SENT`.
- 거절이나 결과 잃음 → `REJECTED`(실패한 시도 하나).
- 취소 중에 결과를 잃으면 다른 로봇 goal처럼 취소를 열어 둔다.

### fake_robots 세계 흉내
- 선 로봇은 2 s 동안 LOCALIZING을 보고한 뒤 성공한다.
- 그 전에 시작한 `drive_by_estimate` 창의 믿는 치우침이 끝난다(`Schedule.relocalize`, `_noisy`). 보고만 미는 NoisyPose 창은 그대로다.
- 실패 주입: `RelocalizeFails(robot, t0, t1, reason)`.
- 움직이는 로봇은 곧바로 BUSY로 답한다. 취소하면 CANCELED로 답한다.

### ROS 가짜 Pode
- 새 파일 `pode_relocalize.py`, 그리고 `fake_pode_node.py`의 두 줄 훅이다.
- 약 2 s 뒤 성공하고 `pose_valid=true`이다. goal이 돌고 있으면 BUSY이다.
- 이 서버가 있어야 `test_ros_check_interfaces`(계약 대 가짜 서버)가 통과한다.

## 2. 커밋과 Codex 리뷰

| 커밋 | 내용 | Codex |
|---|---|---|
| a46f2fe | fleet_core 기능 | 읽음. [확정] 4건 모두 확인하고 4f76362에서 고침 |
| 4f76362 | 기한, STOP, confirm_clear, 남의 구역 | 읽음. [확정] 3건 고침(eb7042f), 1건은 설계대로라 주석만 고침 |
| eb7042f | STOP 즉시 취소, 공유 경계 | 읽음. PAUSE→RUN [확정] 1건과 취소 전 재전송 [추측] 1건을 9c4b1c6에서 고침 |
| 9c4b1c6 | PAUSE, LOCALIZING 대기 | 읽음. 시험 누락 [확정] 1건 → ee2b886 |
| a68dbcd | fleet_manager 연결 | 읽음. 결과 잃음/거절을 NOT_SENT로 취급 [확정] 2건, 시험 공백 → c289174 |
| c289174 | REJECTED 구분 | 읽음. 준비 확인 두 번 사이 경합 [확정] 1건 → e97045e |
| e97045e, ee2b886 | 준비 확인 한 번, LOCALIZING 고착 시험 | 읽음. 기한 단언이 느슨함 [확정] → 558b30e |
| 558b30e | 기한 단언 조임 | 마지막 보고 참조 |
| 5e5882f | fake_robots | 마지막 보고 참조 |

## 3. 시험 (출력 그대로)

- fleet_core 전체, `ZONE_TICK_ORDER=manager`와 `zone` 각각(9c4b1c6 시점): `1148 passed, 1 skipped, 3 xfailed`.
- `test_zone_relocalize.py`, 두 순서 모두: `31 passed`.
- fleet_manager(`test_adapters, test_node_wiring, test_goal_tracker, test_order_goals, test_node_order, test_ask_clear, test_launch_args`, 내 install, 도메인 76): `115 passed, 1 skipped`.
  - `test_scenario_integration.py`와 `test_fms_restart.py`는 지시대로 돌리지 않았다.
- `test_world_relocalize.py`: `6 passed`.
- `test_ros_pode_relocalize.py` + `test_ros_check_interfaces.py`: `5 passed`.
- fake_robots 전체: 마지막 보고 참조.

시험 먼저 원칙:
- fleet_core와 세계 흉내는 시험을 먼저 썼다. 세계 흉내 시험은 구현 전에 import 오류로 실패했다.
- fleet_manager의 a68dbcd 시험과 ROS 가짜 Pode 시험은 코드를 쓴 뒤에 썼다(원칙을 어긴 것이다).
- 이후 고친 것들(c289174, e97045e)은 시험과 함께 넣었다.

주의: fleet_manager 시험과 fake_robots 전체 시험을 같은 도메인(76)에서 동시에 돌리면 서로 섞여 실패한다(O-1 대 O-001 같은 남의 메시지). 한 번에 하나씩 돌려야 한다.

## 4. 세계 흉내 전후

`measure.py`(scratchpad), 씨드 1-3, 주문 6건.
칸: 상황 | 배치 | 씨드 | 격리 | 끝난 주문 | I3_OUTSIDE_HELD | I4_COMMAND | 끝난 시각 s.

### 전 (fleet_core 바꾸기 전, 머지 직후)
| 상황 | 배치 | 씨드 | 격리 | 끝남 | I3 | I4 | 시각 |
|---|---|---|---|---|---|---|---|
| pose_jumps_drive | d2 | 1/2/3 | 0/0/0 | 6/6 ×3 | 2/2/2 | 1/1/1 | 204/209/203 |
| pose_jumps_drive | v7 | 1/2/3 | 2/2/2 | 0/6 ×3 | 0/0/0 | 3/3/3 | 900(시간 끝) |
| pose_jumps | d2 | 1/2/3 | 1/1/1 | 6/6 ×3 | 0 | 0 | 243/243/242 |
| pose_jumps | v7 | 1/2/3 | 0/1/0 | 6/6, 0/6, 6/6 | 0 | 0 | 168/900/166 |

### 후 (relocalize: true)
| 상황 | 배치 | 씨드 | 격리 | 끝남 | I3 | I4 | 시각 |
|---|---|---|---|---|---|---|---|
| pose_jumps_drive | d2 | 1/2/3 | 0/0/0 | 6/6 ×3 | 0/0/0 | 0/0/0 | 186/186/233 |
| pose_jumps_drive | v7 | 1/2/3 | 0/0/0 | 6/6 ×3 | 0/0/0 | 1/1/1 | 182/195/182 |
| pose_jumps | d2 | 1/2/3 | 0/0/0 | 6/6 ×3 | 0 | 0 | 190/187/190 |
| pose_jumps | v7 | 1/2/3 | 0/0/0 | 6/6 ×3 | 0 | 0 | 168/241/166 |

해석:
- 격리는 7건에서 0건이 되었고, 끝난 주문은 48/72에서 72/72가 되었다. I3는 6건에서 0건, I4는 12건에서 3건이 되었다.
- `pose_jumps`(보고만 튐)에서도 격리가 사라졌다. 다시 찾기 자체는 보고만 미는 창을 고치지 못한다. 그래도 2 s 기다리는 동안 창이 지나가고, 재시도 예산을 되돌려 받기 때문이다.
- v7 drive에 남은 I4 1건(t=24.7, `DockCmd WORK`인데 실제 자리는 0.3 m 떨어짐)은 다시 찾기 전에 일어난다. B의 의심 검사가 믿는 자리로 통과한 직후 도킹 명령을 낸 것이다. 그 뒤 `DOCK_FAILED` → 다시 찾기 → 다시 붙기로 회복한다. 의심 검사가 믿는 자리만 보는 한계라 이번 범위 밖으로 둔다.

## 5. 열린 점

1. 계약에 없는 것:
   - 결과에 계기(`why`)를 실을 자리가 없다. 관제 사건에만 남는다.
   - feedback(stage 진행)은 관제가 쓰지 않는다.
   - `std_xy_m`, `std_yaw_rad`, `scan_match`, `fresh_updates`, `stage`, `stamp`는 넘기지 않는다. 품질로 성공을 다시 판정하지 않고 로봇의 success를 그대로 믿는다.
   - 시작 자세의 yaw에는 회전이 섞여 있다. 그래서 RELOCALIZED에는 거리만 적는다.
   - 거절(goal REJECT)을 쓰는 경우가 계약에 없다. 관제는 REJECTED로 보고 실패한 시도 하나로 센다.
2. 통신이 끊기면 다시 찾기를 거두고 예전 통신 끊김 길(격리)을 탄다.
3. 다시 찾는 중의 직접 배정은 `LOCALIZING`으로 거절한다. 운영 화면이 이 까닭을 보여 줄지는 정해지지 않았다.
4. 기다림은 두 단계다(보내기 대기 + 응답 대기). PAUSE와 STOP 동안에는 기한이 미뤄진다. RUN에서는 서지 않는 로봇이라도 시도마다 `timeout + 5 s` 안에 끝난다.
5. 빠른 STOP→RUN 뒤의 재전송:
   - 앞 goal의 취소가 끝났는지 대신 LOCALIZING 보고가 풀리기를 기다린다.
   - 실제 로봇이 취소 뒤에도 잠깐 LOCALIZING을 보고하면 그만큼 늦어진다. 기한 안이다.
6. ROS 가짜 Pode는 LOCALIZING을 보고하지 않는다. 다시 찾는 중에 오는 move/dock/retreat도 거절하지 않는다. 그 동작은 세계 흉내에만 있다.
7. 실제 로봇에서 재 보지 않았다.
