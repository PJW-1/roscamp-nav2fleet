# pode-drive 보관 브랜치 6개 (2026-11-27)

## 이 문서는

- 이 저장소에서 main에 넣지 않고 닫았거나 다른 것으로 대체된 브랜치 6개를 `archive/pode-drive/<원래 이름>`으로 옮겼다(사용자 지시 11-27). 커밋은 바꾸지 않았고 끝 SHA는 아래 표와 같다. 원래 이름의 브랜치는 지웠다.
- **아무것도 병합하지 않았다.** 각 브랜치가 무엇이고 왜 main에 들어가지 않았는지만 적는다. nav2fleet에서 옮긴 보관 브랜치는 `docs/archive_nav2fleet_branches.md`에 있다.

꺼내는 방법:

```bash
git fetch origin
git switch -c <새 이름> origin/archive/pode-drive/<원래 이름>
```

보관 브랜치 자체는 고치지 않는다. 일은 꺼낸 새 브랜치에서 한다.

- "기록"은 그 보관 브랜치에만 있는 파일이다.
- "main과 충돌"은 `git merge-tree --write-tree`로 main 51706709에 대해 잰 것이다.

## 한눈에

| 보관 이름 (`archive/pode-drive/` 뒤) | 끝 SHA | 무엇 | 왜 닫았나 | main과 충돌 | 기록 |
|---|---|---|---|---|---|
| `feat/work-dock-bidir-2026-11-23` | 5b516308 | WORK 도크 양방향 진입 Gazebo 비교(6+6판) | 처리량 이득이 없고 급행 출구 옆을 스쳐 실물로 가져갈 가치가 없다는 결론, 11-24 닫음 | 있음 | `docs/report_2026-11-23_work_dock_bidir.md` |
| `feat/real-side-by-side-2026-11-23` | f361ee4d | 실물 키트를 nav2fleet 기준판 옆에 다른 이름으로 배포 | 11-23에 nav2fleet 얼림을 풀고 `tools/sync_nav2fleet.sh`로 main을 따라가게 해 나란히 둘 필요가 없어짐 | 있음 | `docs/worklog_real_side_by_side_2026-11-23.md` |
| `fix/w-bend-clearance-2026-11-23` | 48412dc9 | W 진입 굽이를 동쪽 벽에서 더 띄움(1커밋) | 실물 회전 반응을 본 뒤 정하기로 보관. 11-26 실측에서 회전 명령 반응은 정상(모터 반응은 W 입구 밀림 원인 후보에서 빠짐, 바퀴 미끄러짐은 못 가림). 지금 지도와 충돌 | 있음 | `docs/worklog_w_bend_clearance_2026-11-23.md` |
| `diag/dock-release-2026-11-23` | 276e687c | `test_fms_restart`의 WORK_DOCK 조기 해제 진단(코드 바꾸지 않음) | 11-24 결정으로 도크 조기 해제는 그대로 둠. 제안한 수정은 적용하지 않음 | 없음 | `docs/worklog_dock_release_2026-11-23.md` |
| `feat/eta-cause-2026-11-22` | b905ac4e | ETA 오차 원인 분석 기록 | 관제 ETA 모델은 바꾸지 않기로 함. 그중 F1은 main에 들어감, 나머지는 실물 기록이 필요 | 없음 | `docs/worklog_eta_cause_2026-11-22.md` |
| `feat/fake-scale-widen-2026-11-23` | e3286004 | 가짜 로봇 시간 배율 넓히기, 벽시계 환산 메모와 ETA 시험 비교 | push·병합하지 않은 실험. 뒤의 main 변경과 충돌 | 있음 | `docs/worklog_fake_scale_widen_2026-11-23.md` |

## 같은 날 정리한 것

- 실측 기록 브랜치 `data/real-2026-11-23`, `data/real-2026-11-24`, `data/real-2026-11-26`은 main에 합쳤다(`results/` 아래만, bag 포함).
- main에 다 들어간 브랜치는 원격과 로컬에서 지웠다. 지운 이름과 끝 SHA는 PC의 `~/ros2camp/podedrive_ws/branch_cleanup_2026-11-27.txt`에 있다.
