# a+c+d 레시피 seed 셋 학습과 평가(작업 4) — 2026-11-24

브랜치 `feat/rl-acd-train-2026-11-24`(worktree `C:/Users/pjw/ros2camp/pd-acd`, 시작 `origin/feat/recipe-2026-11-22` = 9c58d67). 사용자가 자는 동안 혼자 돌린 밤 작업이다. 앞선 결정과 방법은 `docs/worklog_recipe_2026-11-22.md`(B62~B80)에 있다.

## 1. 무엇을 하나

a+c+d(행동 변화 벌점 + 이전 행동 관측, 비대칭 actor-critic, 에피소드마다 최고 속도 U(0.08, 0.16)와 그 값 관측)를 ppo_k에서 seed 1~3으로 500번씩 학습한다. 그다음 5·6번과 같은 방식으로 평가하고 4b(ppo_k), 대조군(ctrl), 6번 "의도 없음"(= c 단독 + 500번) seed 1~3과 비교한다. 가장 좋은 seed를 ONNX 형식 3으로 내보낸다. 코디네이터가 "go"라고 하면 Gazebo에서 4b와 비교한다.

사용자의 질문은 셋이다.
1. 성공이 4b보다 나은가?
2. 흔들림(분당 w 부호 바뀜)이 얼마나 줄었나?
3. 실물 시험 후보로 볼 만한가?

## 2. 큐

`docs/rl_acd_2026-11-24/queue.sh`가 작업 목록 파일을 한 줄씩 돌린다. 작업마다 잠금(`C:/Users/pjw/ros2camp/.heavy_run.lock`)을 잡고, 끝나면 놓고, 90 s 쉰다. `.heavy_run.queue/`에 파일이 있으면 잠금을 잡지 않고 30 s마다 다시 본다. 이미 `run.json`(학습)이나 `eval.json`(평가)이 있는 작업은 건너뛴다. 출력은 `D:/isaac/runs/store_arena/acd24/`, 작업마다 끝난 줄은 `acd24/queue_status.txt`에 남는다.

| 목록 | 내용 |
|---|---|
| `jobs_seeds.txt` | seed마다 학습 250번 × 2와 평가 7개(0.12의 무작위·마주 오기·인계 출발, 0.08과 0.16의 무작위·마주 오기) |
| `jobs_run.txt` | `--capped` 연기 시험(64 env, 10 s 에피소드) + `jobs_seeds.txt`. 첫 학습 조각 뒤 이 목록으로 바꿨다(§5) |
| `jobs_base.txt` | 비교 대상의 빠진 평가: c 단독 seed 1~3의 인계 출발(0.12), 4b·ctrl·c 단독의 0.08·0.16 무작위·마주 오기(`--capped`) |

학습 명령(seed s, 조각 1; 조각 2는 `--init` 대신 조각 1의 마지막 모델을 `--resume`):

```
train.py --num_envs 4096 --max_iterations 250 --seed s --init D:/isaac/runs/store_arena/recipe/init_combo_acd.pt \
  --set prev_action_obs=1 --set critic_privileged=1 --set speed_cap_obs=1 --set speed_cap_range=0.08,0.16 \
  --set w_action_rate=-0.05 --set w_w_flip=-0.5 --set robot_max=2 --set robot_priority_prob=0.5 --set handover_start_prob=0.15
```

평가 명령: `play.py --num_envs 4096 --episodes 1 --seed 7 --set robot_max=2 --set robot_priority_prob=0.5` + 정책의 관측 블록(`--set prev_action_obs=1 --set critic_privileged=1 --set speed_cap_obs=1`) + `--set speed_cap_range=c,c` + 세트(`--set robot_headon=1` 또는 `--set handover_start_prob=1`).

## 3. 결정 목록 (B81부터)

| 번호 | 무엇 | 정한 것 | 다른 후보 | 이유 |
|---|---|---|---|---|
| B81 | 시작 가중치와 학습 길이 | 5번 a+c+d 판이 쓴 넓힌 체크포인트 `recipe/init_combo_acd.pt`(ppo_k를 a+c+d 모양으로 넓힘)를 그대로 쓴다. 250번 × 2 조각: 조각 1은 `--init`(Adam 상태 버림, B68), 조각 2는 `--resume`(Adam 이어 씀). 6번과 같다. seed 1, 2, 3 | ppo_k를 다시 넓히기 | 넓힘은 결정적이고(`test_widen_checkpoint.py`), 그 파일의 `widened_spec`이 a+c+d 모양(입력 41 → 44, critic 특권 20칸)임을 확인했다. 같은 파일을 쓰면 5번 판(200번, seed 1)과 시작점이 같다 |
| B82 | 작업 사이 쉼 | 90 s | 180 s(run_train.sh) | 평가가 21개 + 비교 23개라 180 s면 쉼만 2시간이 넘는다. 다른 트랙은 60 s마다 잠금을 다시 보므로 90 s 안에 잡을 수 있다. 코디네이터 요청으로 `.heavy_run.queue/`에 파일이 있으면 잠금을 잡지 않는다 |
| B83 | 속도 상한 관측이 없는 정책(4b, ctrl, c)의 0.08·0.16 평가 | `play.py --capped`: 정책의 v 목표를 학습 최고 속도 0.12로 바꾼 뒤 그 에피소드 상한으로 자른다(0.16에서는 0.12를 넘지 않는다). 경기장(스크립트 로봇 속도, 에피소드 시간)은 상한을 받은 정책과 같다 | `--capped` 없이 `speed_cap_range`만 주기(v를 [v_min, 상한]으로 줄여 맞춤) | 플러그인(`pd-plugin2` follow_path_rl.cpp)은 형식 2 정책의 v를 [v_min, 정책 v_max]로 바꾼 뒤 `speed_limit_`(min(최고 속도, 정책 v_max))로 자른다. 실제로 그 정책을 다른 속도에서 쓸 때와 같은 평가다. 줄여 맞추기는 정책이 보지 못한 행동 공간이다 |
| B84 | seed 퍼짐 안팎 | seed 평균 차이와 그 SE(단일 체크포인트 대상: seed별 짝지은 차이의 sd/√3, c 단독 대상: 두 쪽 seed 성공률의 분산으로 √(var_a/3 + var_c/3))를 쓴다. \|차이\| > 2 SE면 "밖", 아니면 "안". 6번이 잰 seed 하나 짝의 퍼짐(무작위 약 0.4 %p, 마주 오기 약 1.7 %p)도 함께 적는다 | t 분포(자유도 2, 4.3 SE) | 결과를 보기 전에 정했다. seed 셋으로 잰 sd는 그 자체가 흔들리므로 2 SE는 느슨한 기준이다. 그래서 "밖"이라도 크기가 6번 퍼짐보다 작으면 보고서에 그렇게 적는다 |
| B85 | 내보낼 seed(결과를 보기 전에 정함) | 평가 칸 7개(0.12 무작위·마주 오기·인계 출발, 0.08과 0.16 무작위·마주 오기) 성공률 평균이 가장 높은 seed. 0.1 %p 안에서 같으면 0.12 무작위의 분당 부호 바뀜이 적은 쪽 | 0.12만 보기 | 사용자는 여러 상한에서 쓰는 한 정책을 원한다(B78). 첫 실물 시험은 0.08이다 |
| B86 | 내보내기 | 앞보기 0.40 m, 창 20, 형식 3(입력 44, obs_blocks `prev_action`, `speed_cap`, 상한 0.08~0.16), 정책 폴더 `pode/pode_rl_controller/policies/store_arena_acd2`(spec과 fixture는 커밋, onnx는 gitignore). onnx 사본은 `D:/isaac/runs/store_arena/acd24/export/` | 5번 `store_arena_acd` 덮어쓰기 | 5번 판과 구별한다. 기본 정책 `store_arena_4b`와 기본 설정은 건드리지 않는다 |
| B87 (코디네이터, 2026-11-24 06:00 무렵) | 순서와 비교 평가 줄이기 | seed 3이 끝나면 내보내기와 parity를 먼저 하고, Gazebo 비교를 "go"를 기다리지 않고 바로 돌린다(잠금이 비어 있고 1번 트랙 회귀가 끝났다). 비교 평가는 Gazebo 뒤에 4b의 0.08·0.16 무작위·마주 오기 넷만 돌린다(`jobs_base2.txt`). ctrl·c 단독의 0.08·0.16과 c 단독의 인계 출발(19개)은 돌리지 않는다 | `jobs_base.txt` 23개 모두 | 사용자 질문은 "4b보다 나은가"다. 첫 실물 시험이 0.08이라 4b의 0.08이 필요하다. 0.16은 상한 입력이 무엇을 사는지 보여 준다. 0.12의 4b·ctrl·c 평가는 같은 seed와 설정으로 이미 있어 다시 쓴다(첫 에피소드 확인 통과) |
| B88 | Gazebo 비교 | `gz_run.sh`: RL+shim 4b 대 RL+shim acd2, Q2 → E-3(`results/2026-11-22/compare_012/q2e3.yaml`)과 c_moving(`tools/nav_bench/scenarios.yaml`), 칸마다 20판, 도메인 61, GUI 없음. shim 설정은 item 8과 같다(angular_disengage_threshold 0.39). 표는 `gz_results.py`(판마다 성공, 충돌, 시간, 분당 부호 바뀜, 몸 여유 최솟값, 이른 넘김; acd2 − 4b를 성공은 Wald SE와 Fisher 정확 p, 나머지는 Welch로) | item 8 시나리오 다섯 | 코디네이터 지시. `gz_results.py`를 item 8 legs에 돌려 4b 줄이 `item8_results.md`와 같음을 확인했다 |

## 4. 알아야 할 포인트

- c 단독(6번 의도 없음)은 ppo_k에서 700번(c 200번 + 500번) 더 학습했고, 이번 a+c+d는 500번이다. 학습 길이가 같지 않다.
- 4b·ctrl·c의 0.16 평가는 v가 0.12를 넘지 않는다(B83). 0.16에서 a+c+d가 이기면 그 일부는 더 빨리 갈 수 있어서다. 시간이 늘어난 에피소드 길이 안에서 시간 초과가 날 수도 있다.
- 짝짓기는 같은 상한·같은 세트 안에서만 한다. `acd_tables.py`가 추적 env의 첫 경로와 사람·카트·로봇 위치가 같은지 확인한다(5번 판끼리는 같고, 다른 세트나 다른 상한끼리는 다름을 확인했다).

## 5. 진행 기록

- 23:52 seed 1 조각 1 시작(잠금 잡음).
- 23:5x 코디네이터가 잠금 규칙을 더했다(`.heavy_run.queue/` 확인). 돌던 큐는 이 규칙을 모르므로, 조각 1이 끝나면 그 큐를 쉼 동안 멈추고 새 큐(`jobs_run.txt`)를 시작하는 셸을 걸었다.
- 00:22 seed 1 조각 1 끝(exit 0, 1793 s). 00:22:10에 큐를 바꿨다. 새 큐는 다른 트랙의 Gazebo 두 판(gz_v7, kitmaps)을 기다렸다가 00:35에 시작했다.
- 00:37 `--capped` 연기 시험 exit 0(81 s). 0.08에서 명령 v의 최댓값이 정확히 0.08이었다.
- 02:08 seed 1의 학습 조각 2와 평가 7개가 모두 exit 0으로 끝났다. 평가 한 판은 0.12와 0.16에서 5~7분, 0.08에서 약 10분 걸렸다.
- 이번 평가는 `play.py` 기본값 `--track 16`으로 돌았다. 5·6번 평가는 `--track 1024`였다. 이 값은 `rollouts.npz`에 저장할 env 수와 추적 env의 시간 초과 분류에만 쓰이고, 성공 집계와는 상관없다. `acd_tables.py`의 같은 에피소드 확인은 앞 16개 env를 비교한다.
