# 실물 고정 경로 충돌 검사 끄기 (2026-11-18)

## 1. 왜

사용자 결정(11-18): 실물 로봇도 고정 경로 컨트롤러의 RPP 충돌 검사를 시뮬처럼 끈다.
실물 시험에서 좁은 곳의 고정 경로마다 "RegulatedPurePursuitController detected collision ahead!"와 "Controller patience exceeded"가 반복됐다. 두 대 모두 격리됐다.
시뮬은 이미 끈다(`sim/fleet_gz_sim/fleet_gz_sim/launch_parts.py:119-125`, `206-213`). `fixed_line_check`가 true일 때만 켠다. FollowPathGoal(마당 goal)은 켜 둔다.

## 2. 바꾼 것

- `tools/real_pode/make_params.py`: `COLLISION_TARGETS`(FollowPath, FollowPathThrough, FollowPathReverse의 `use_collision_detection`)를 기본으로 false로 쓴다. 줄 단위 바꾸기와 다시 읽어 견주는 확인은 그대로이고, 확인에 새 키를 넣었다. `--fixed-line-check`를 주면 원본 값과 상관없이 true로 쓴다(처음 커밋은 원본을 그대로 뒀는데 고쳤다). 마지막에 `fixed-line check on|off` 한 줄을 찍는다. `avoidance.yaml` 사본에서는 `follow_path_goal.desired_linear_vel`이 속도 S로 바뀐다. 충돌 검사 설정은 원본 값 그대로다(저장소 원본은 true).
- `tools/real_pode/start.sh`: `FIXED_LINE_CHECK` 검사(`DOCK_STUB`와 같은 방식). 안 주거나 0이면 끔, 1이면 `--fixed-line-check`를 넘김, 그 밖은 launch 전에 exit 2. 머리글 주석 한 줄.
- `tools/real_pode/README.md`: "Gazebo와 다른 실물 기본값"의 고정 경로 항목을 바꿨다.
- `tools/test/test_real_pode.py`: 새 시험 추가, 줄 바뀜 수를 세던 `test_make_params_keeps_comments`와 `test_make_params_sets_forward_speeds_only`를 새 기본에 맞춰 고침.

위험: 고정 경로에서는 앞의 충돌을 미리 알아채는 곳이 없다. 로봇 사이 간격은 관제(fleet)에 달렸다.

## 3. 시험 결과

- 시험을 먼저 썼다. 구현 전 관련 17개 중 14개 실패, 3개 통과(`-k "fixed_line or keeps_comments or sets_forward"`).
- 구현 뒤 `ROS_DOMAIN_ID=69 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m pytest tools/test/test_real_pode.py -q`: `84 passed, 2 skipped in 111.74s`.

## 4. 배포

로봇은 꺼져 있어 배포하지 않았다. 이 커밋이 HEAD인 체크아웃에서 `tools/real_pode/deploy.sh`로 배포한다. 로봇에서 다시 빌드할 필요는 없다. 켜 두려면 `FIXED_LINE_CHECK=1 ./start.sh ...`.

## 5. 수정 (Codex 리뷰 지적)

`--fixed-line-check`가 true를 쓰지 않고 원본 값을 그대로 뒀다. 이제 세 키에 true를 명시해 쓰고, 다시 읽어 견주는 확인도 두 경우를 모두 덮는다. 원본이 반대 값인 사본으로 시험을 먼저 썼고(True 쪽 실패 확인) 고쳤다. README의 make_params 행과 위 설명도 고쳤다.
시험 결과: `86 passed, 2 skipped in 111.99s`

## 6. main 합침과 배포 (2026-11-18)

- 사용자 요청(11-18): 로봇에 새로 배포하고 main도 이 설정으로 간다.
- main: a14b2e5에 `feat/real-fixed-line-check-off`(d3b9487, 245c244)를 --no-ff로 합쳤다. 합친 트리는 245c244와 같다.
- 시험: 245c244에서 `ROS_DOMAIN_ID=68 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m pytest tools/test/test_real_pode.py -q` 결과 `88 passed in 119.66s`. 새 시험을 d3b9487의 make_params로 돌리면 True 쪽이 실패한다(`1 failed, 1 passed`).
- 문서 수정(Codex 지적): 마당 goal(FollowPathGoal)의 검사는 "늘 true"가 아니라 원본 값 그대로다(저장소 원본은 true). README와 2절을 고쳤다.
- 배포: 로봇 두 대(192.168.0.200, .201) 모두 ping이 없어(꺼짐) 아직 배포하지 않았다. 켜지면 HEAD가 이 커밋 이후인 main 체크아웃에서 아래를 돌린다.

  ```
  tools/real_pode/deploy.sh pinky@192.168.0.200
  tools/real_pode/deploy.sh pinky@192.168.0.201
  ```

  바뀐 것은 `tools/real_pode`뿐이라 로봇에서 colcon build는 필요 없다. 다음 `start.sh`가 `params/`를 다시 만들 때 적용된다. start.sh 출력에 `make_params: fixed-line check off`가 찍히는지 본다.
