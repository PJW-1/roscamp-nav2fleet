# 양보 기록 도구와 Gazebo 9판 결과 (2026-11-18)

밤샘 지시서 2부 A1의 결과다. 동작은 바꾸지 않았고, 새 파일은 `tools/yield_report.py`와
`tools/test/test_yield_report.py` 둘이다.

## 도구 사용법

```bash
python tools/yield_report.py 기록1.jsonl.gz 기록2.jsonl [--encounters] [--grace 10]
```

`fleet_record.jsonl`과 `.jsonl.gz`를 여러 개 읽어 Markdown 표를 낸다. 표는 판별 한 줄과 합계 한 줄이다.
`--encounters`를 주면 마주침(로봇 사다리) 하나마다 한 줄씩 시간 순서를 더 낸다. 도구는 기록을 읽기만 한다.

`tools/triage`의 하위 명령으로 넣지 않고 따로 만들었다. triage의 읽기(`readers.py`)는 일반 텍스트만
읽고, 주문 하나의 원인을 가르는 구조(주문판과 보고)라서 로봇별 사다리와 문구 집계에는 맞지 않는다.
그래서 기록 읽기는 이 파일에 따로 둔 수십 줄이고, `.gz`도 읽는다.

## 집계 규칙과 추정의 한계

- LOCAL_YIELD: `wait`, `backup`, `retry`는 이벤트 detail 첫 낱말의 건수이고, `ends <상태>`는 상태별 건수다.
- HALL_YIELD 문구: `stops N s for`, `backs N m, then stops`, `out of the way of`, `waits for`,
  `backs out to`, 그 밖(`other`)으로 센다.
- BLOCKED 전과 뒤는 추정이다. 이벤트 줄 바로 앞에서 그 로봇이 마지막으로 보고한 상태가 BLOCKED면 뒤
  (`_yield_to_robot`), 아니면 앞(`_watch_near`)으로 본다. 지시서 규칙에 두 가지를 더했다. `out of the way
  of <로봇>`은 이벤트를 낸 로봇이 비키는 쪽이 아니라 보내지는 쪽이므로 이름이 나온 로봇의 상태도 본다.
  `waits for`와 `backs out to ... for`는 코드에서 `_yield_to_robot`만 내므로 문구로 뒤라고 정한다.
- 마주침 하나는 한 로봇의 LOCAL_YIELD 사다리(첫 단계부터 `ends`까지)다. 그 로봇이 낸 또는 이름이
  나온 HALL_YIELD를 사다리 시작 10 s 전부터 끝난 뒤 10 s까지 묶는다.
- 관제가 먼저 끼어든 건수는 이렇게 센다. 한 HALL_YIELD에 관계된 로봇(낸 로봇과 이름이 나온 로봇)
  누구의 사다리도 그 시각에 진행 중이 아니고 끝난 뒤 10 s 안도 아니면 먼저로 센다. 사다리가 아예 없었던
  경우를 포함한다. 10 s는 `--grace`로 바꾼다. 같은 9판에서 2 s는 59번, 0 s는 61번이다.
- 틀릴 수 있는 경우는 다음과 같다.
  1. 상태 보고가 주기적으로 들어오므로 BLOCKED 판정이 한 주기 늦을 수 있다.
  2. 사다리가 `ends BLOCKED`로 끝난 직후 관제가 나서는 경우는 먼저가 아닌데, 10 s를 넘기면 먼저로 센다.
  3. 사다리를 만들지 않은 BLOCKED(장애물 앞에서 곧바로 BLOCKED)도 사다리가 없으므로 먼저로 센다.
  4. 두 로봇이 동시에 사다리를 타면 이름 묶기가 맞는 짝을 못 고를 수 있다.
  5. 판이 끝날 때까지 `ends`가 없는 사다리는 끝 없음으로 둔다.

## 9판 결과 (`docs/결과_추가지시서3/gazebo/*/game*/fleet_record.jsonl.gz`)

### LOCAL_YIELD 단계별 건수

| 판 | wait | backup | retry | ends |
|---|---|---|---|---|
| reg_inf/game1 | 79 | 0 | 0 | BLOCKED 20, IDLE 1, MOVING 58 |
| reg_inf/game2 | 23 | 0 | 0 | BLOCKED 7, MOVING 16 |
| reg_inf/game3 | 37 | 1 | 1 | BLOCKED 15, IDLE 2, MOVING 20 |
| reg_nocheck/game1 | 66 | 1 | 1 | BLOCKED 16, IDLE 2, MOVING 48 |
| reg_nocheck/game2 | 34 | 1 | 1 | BLOCKED 11, IDLE 2, MOVING 20 |
| reg_nocheck/game3 | 63 | 0 | 0 | BLOCKED 17, IDLE 3, MOVING 43 |
| reg_stall/game1 | 28 | 0 | 0 | BLOCKED 8, MOVING 20 |
| reg_stall/game2 | 8 | 0 | 0 | BLOCKED 2, MOVING 6 |
| reg_stall/game3 | 10 | 0 | 0 | BLOCKED 4, IDLE 2, MOVING 4 |
| 합계 | 348 | 3 | 3 | BLOCKED 100, IDLE 12, MOVING 235 |

### HALL_YIELD 문구별 건수와 추정

| 판 | 전체 | backs then stops | stops | out of the way | waits for | backs out | other | BLOCKED 전 | BLOCKED 뒤 | 관제가 먼저 |
|---|---|---|---|---|---|---|---|---|---|---|
| reg_inf/game1 | 4 | 0 | 3 | 1 | 0 | 0 | 0 | 3 | 1 | 0 |
| reg_inf/game2 | 2 | 0 | 2 | 0 | 0 | 0 | 0 | 2 | 0 | 1 |
| reg_inf/game3 | 4 | 0 | 2 | 1 | 1 | 0 | 0 | 2 | 2 | 3 |
| reg_nocheck/game1 | 4 | 0 | 3 | 1 | 0 | 0 | 0 | 0 | 4 | 3 |
| reg_nocheck/game2 | 6 | 0 | 4 | 2 | 0 | 0 | 0 | 2 | 4 | 4 |
| reg_nocheck/game3 | 4 | 0 | 3 | 1 | 0 | 0 | 0 | 1 | 3 | 2 |
| reg_stall/game1 | 38 | 0 | 20 | 17 | 1 | 0 | 0 | 15 | 23 | 34 |
| reg_stall/game2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| reg_stall/game3 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 합계 | 62 | 0 | 37 | 23 | 2 | 0 | 0 | 25 | 37 | 47 |

지시서 1절의 값과 모두 같다. `wait`는 판마다 8~79번이고, `backup`과 `retry`는 9판 합계 3번씩이며,
HALL_YIELD는 합계 62번(`stops` 37, `out of the way` 23, `waits for` 2)이다.

## 읽는 법과 눈에 띄는 점

- 합계에서 BLOCKED 뒤로 추정된 HALL_YIELD가 37번, 앞(`_watch_near` 후보)이 25번이다. 근접 감시를 뺄지
  (2부 4절 게이트)는 이 숫자를 보고 사용자가 정한다.
- 관제가 먼저 끼어든 HALL_YIELD는 62번 가운데 47번이다. 그 가운데 `reg_stall/game1`이 34번이다(그 판은
  HALL_YIELD가 38번이고 LOCAL_YIELD 사다리는 28번이다). 사다리 진행 중이거나 끝난 뒤 10 s 안인 것은 15번이다.
- `backup`이 나온 3번은 모두 사다리 시작 2 s 뒤였다. 그 가운데 두 번(`reg_nocheck/game1`, `game2`)은 사다리
  시작 4 s쯤에 HALL_YIELD `stops`가 이어서 나왔다. 로봇이 먼저 후진한 뒤 관제도 나선 경우다. 나머지
  한 번(`reg_inf/game3`)은 관제가 끼어들지 않았고 BLOCKED로 끝났다.
- `ends BLOCKED`가 100번인데 `backup`은 3번뿐이다. 대부분의 마주침은 후진 없이 2 s 대기 뒤 곧바로
  BLOCKED로 끝났다. 이 기록에서 로봇이 낮은 쪽만 물러나는 일은 드물다.

## 건너뛴 것

- `~/real_runs/`의 실물 기록은 이 컴퓨터에 없어 돌리지 못했다. 기록이 모이면 같은 명령으로 돌린다.
