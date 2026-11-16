# 작업 기록: 세계 흉내의 "틀린 자리로 달리기" (2026-11-16)

브랜치 `test/world-drive-by-estimate` (origin/main 71bdca0에서 시작). 푸시하지 않았다.

## 왜

`NoisyPose`(sim/fake_robots/fake_robots/world/injections.py)는 지금까지 **보고만** 밀었다. 몸은 바르게
달리고 보고만 옆으로 간다. 실제 로봇은 AMCL이 틀리면 거꾸로다. 틀린 믿음대로 달리므로 보고는 길 위에서
"도착"하고 몸은 치우침만큼 벗어난다. 관제의 위치 튐 가드는 보고가 조각 끝 목표에 닿으면 의심을 푸는데,
이 약한 곳(믿는 자리로 도착해 몸이 벗어난 채 풀림)은 실제 자리를 보는 불변식 I3_OUTSIDE_HELD로만
드러난다. 그것을 재려고 이 흉내를 넣었다.

## 바뀐 것

- `injections.py`: `NoisyPose.drive_by_estimate: bool = False` 추가. `Schedule.belief_dx(robot, t)`가
  켜진 창의 `jump_m` 합(믿는 자리 - 실제 자리, +x)을 돌려준다.
- `robot.py`: `SimRobot.belief_dx` 추가. 믿는 자리로 달리는 걸음 안에서만 0이 아니다.
- `world.py`:
  - `_step_run`: 창이 열려 있으면 몸을 믿는 자리로 옮겨 `run.step`을 돌리고 되돌린다. 창이 없으면
    전과 똑같이 `run.step`만 부른다.
  - 믿는 치우침이 바뀌는 걸음(창 시작, 창 끝)에는 `_rebase`가 달리던 길의 첫 점(`way[0]`, 로봇 자리)을
    새 자리로 바꾼다. order goal이면 달리는 조각(`sub`)의 것을 바꾼다. 끝에 닿아 도는 중(길에 한 점만
    남음)이면 새 자리에서 그 끝 점으로 다시 간다. Nav2 goal의 나아감 기준점(`mark`)도 같이 옮겨서 창
    경계를 나아간 것으로 세지 않는다(Codex 지적 1, 2를 2fc4cb8에서 고침).
  - `discs`: 그 걸음에서는 다른 로봇과 물체도 같은 치우침만큼 옮겨 보인다(센서는 몸 기준으로 본다).
  - `clear_to`(세계의 몸 닿음 막기): 실제 자리로 되돌려 본다.
- `situations.py`: `pose_jumps_drive` 상황 추가. `pose_jumps`와 같고 `drive_by_estimate=True`만 다르다.
  약한 곳을 재는 상황이라 `NAMES`(표와 시험이 도는 목록)에는 넣지 않았다.
- 보고(`link.py _sample`)는 손대지 않았다. 원래도 `실제 + jump + 잡음`이었고, 켜면 그것이 곧 믿는 자리
  + 잡음이다.

## 켜는 법

```python
from fake_robots.world import NoisyPose, ZoneWorld
NoisyPose("pode1", 20.0, 40.0, sigma_m=0.03, jump_m=0.30, drive_by_estimate=True)

# 표 도구로
from fake_robots.world.table import run_situation
run_situation("pose_jumps_drive", "d2", seed=1)
```

`run_situation`의 `overrides`는 관제 파라미터라 이 흉내를 켜지 못한다. 상황 이름으로 켠다.

## 시험 (나온 그대로)

먼저 쓴 시험(test_world_faults.py) 세 개는 구현 전에 3 failed, 12 passed였다.

- 켜면 창 안에서 보고 - 실제 = 치우침, 보고는 길(y) 위, 믿는 자리로 N_E에 도착하고 몸은 N_E - 0.2에 선다.
- 창이 끝나면 보고 = 실제, 실제 자리에서 이어 가 N_E에 선다.
- 끄면 몸의 궤적이 주입 없는 세계와 걸음마다 같다. 기본값은 False.
- `pose_jumps_drive`는 `pose_jumps`에 플래그만 켠 것이고 `NAMES`에 없다.

Codex 지적으로 더한 시험 둘(끝에서 도는 중에 창이 닫힘, `_rebase` 단위)은 고치기 전에 2 failed, 16 passed였다.

명령과 결과(저장소 뿌리에서, `ROS_DOMAIN_ID=75 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`):

| 명령 | 커밋 | 결과 |
|---|---|---|
| `PYTHONPATH=pc/fleet_core:sim/fake_robots python3 -m pytest -q -p no:cacheprovider sim/fake_robots/test/test_world_faults.py sim/fake_robots/test/test_situation_faults.py sim/fake_robots/test/test_traffic_bugs.py sim/fake_robots/test/test_world_invariants.py sim/fake_robots/test/test_world_follow.py sim/fake_robots/test/test_world_nav_goal.py` | 8993c84 / 2fc4cb8 | 76 passed / 78 passed |
| `PYTHONPATH=pc/fleet_core:sim/fake_robots python3 -m pytest -q -p no:cacheprovider sim/fake_robots/test` | 8993c84 | 883 passed, 6 skipped (365 s) |
| 같은 전체 | 2fc4cb8 | 885 passed, 6 skipped (526 s) |
| `python3 -m pytest -q -p no:cacheprovider pc/fleet_core/test` (PYTHONPATH 없이) | 8993c84 | 수집 오류 (`No module named 'fleet_core'`), rc=4 |
| `PYTHONPATH=pc/fleet_core python3 -m pytest -q -p no:cacheprovider pc/fleet_core/test` | 8993c84 | 1049 passed (46 s) |

fleet_core는 이 변경이 건드리지 않는다. `test_scenario_integration.py`, `test_fms_restart.py`(ROS 도메인 50-59)는
돌리지 않았다.

## Codex 리뷰

- 8993c84: 도착해서 읽음. [확정] 끝에서 도는 중 창 경계에서 끝 점을 도착으로 침, [확정] NavGoalRun `mark`가
  옛 좌표에 남음 → 둘 다 직접 확인하고 시험 먼저 써서 2fc4cb8에서 고침. [확정] 켜진 것과 꺼진 것이 겹칠 때
  보고 → 의도한 뜻이라 남은 것에 적음. 기본 경로는 전과 같은 연산이라는 확인, 시험 범위가 좁다는 지적.
- 2fc4cb8: 도착해서 읽음. 결함 없음, 시험 범위가 좁다는 지적만.

## 재 본 것 (참고)

`run_situation(name, layout, seed)`, 씨드 1-3:

| 배치 | 상황 | 결과 |
|---|---|---|
| d2 | pose_jumps | 셋 다 실패 없음 (약 245 s) |
| d2 | pose_jumps_drive | 셋 다 I3_OUTSIDE_HELD (21.6 s 쯤, 34-35 s 쯤)와 I4_COMMAND(DockCmd인데 몸이 도크 밖). 씨드 1은 1/6만 끝냄 |
| v7 | pose_jumps | 셋 다 0/6 (origin/main 사본에서도 같다. 이 변경 전부터) |
| v7 | pose_jumps_drive | I4_COMMAND(DockCmd인데 몸이 도크 밖) 되풀이. 씨드 2만 6/6 끝냄 |

d2에서는 몸이 벽 밖(x < 0)까지 나간다. 세계는 벽에 몸이 닿는 것을 막지 않는다.

## 남은 것

- 치우침은 +x 한 방향뿐이다(`jump_m`이 원래 그렇다). 옆 방향, 회전 오차는 없다.
- 창이 열리는 걸음에 길 첫 점만 믿는 자리로 바꾼다. 길에서 옆으로 벗어난 믿음이면 RPP처럼 길로 천천히
  돌아오지 않고 다음 꼭짓점으로 곧게 간다.
- 세계는 벽과 선반에 몸이 닿는 것을 막지 않는다. 틀린 믿음으로 벽을 뚫고 지나갈 수 있다.
- `drive_by_estimate`가 켜진 것과 꺼진 것이 한 로봇에 겹치면 보고 = 믿는 자리 + 꺼진 것의 jump + 잡음이다
  (꺼진 것은 원래대로 보고만 민다). 의도한 뜻이지만 시험은 없다.
- 새 시험은 한 대의 FollowRun과 `_rebase` 단위 시험뿐이다. NavGoalRun, RetreatRun, OrderRun, 다른 로봇이나
  물체가 있을 때의 충돌 좌표(`discs`, `clear_to`)는 코드를 읽어 맞다고 봤고 시험으로 보지 않았다.
- v7의 `pose_jumps`가 origin/main에서도 0/6이다. 이 작업과 관계없다(따로 볼 일).
- `pose_jumps_drive`의 결과는 아직 판정 기대가 없다. 가드를 고친 뒤 기대를 정해 `NAMES`에 넣을지 정한다.
