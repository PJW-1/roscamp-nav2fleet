# 0.12 m/s 네 컨트롤러 비교 (2026-11-22)

- 조건: Gazebo v7 경기장 한 대, `nav_max_speed` 0.12 m/s(속도 원천 `pode/pode_nav_config/params/nav_speed.yaml`), 기준 커밋 b4d17b9
  (8064b64 + MPPI `time_steps` 83). RL 정책은 `store_arena_4b`(v_max 0.12, `ppo_k/c2/model_998.pt`).
- 컨트롤러: RPP = `FollowPathGoal`, MPPI = `FollowPathMPPI`, RL = `FollowPathRL`(가드 끔), RL+shim = `FollowPathRLShim`
  (RotationShim이 FollowPathRL을 감쌈, `rl_and_shim.yaml`).
- `main/`: O_IN → M_E, 시나리오 a_hall, e_corridor, d_robot, c_moving, 칸마다 6판(Williams 순서), 2026-11-22 03:05~04:28.
  `tracks.png`과 `compare.png`의 b_static 칸은 돌리지 않아 비어 있다.
- `q2e3/`: Q2 (1.66, 0.15, 0°) → E-3 (0.64, 0.24, 90°), 장애물 없음, 칸마다 3판, 02:54~03:02. 설정은 `q2e3.yaml`.
- 파일: `table.md`, `legs.csv`(판별 값), `compare.png`, `tracks.png`, `runs.txt`. 해석은 `docs/worklog_week2_compare_2026-11-22.md`.
- 판별 원본 JSON과 launch 로그는 WSL `~/rl_gz_check/cmp012_025415/`에 있다.
- 다시 만들기: `python3 tools/nav_bench/report.py --config tools/nav_bench/scenarios.yaml --legs ~/rl_gz_check/cmp012_025415/main/legs --out <폴더>`
  (Q2 → E-3은 `--config results/2026-11-22/compare_012/q2e3.yaml --legs ~/rl_gz_check/cmp012_025415/q2e3/legs`)
