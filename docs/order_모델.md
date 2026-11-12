# order 모델 (사용자 기준)

이동 하나를 "조각 목록"으로 한 번에 보내는 방식입니다. VDA 5050의 order(base, horizon, 이어 붙이기)를 옮겼습니다. 기본은 꺼져 있고(`use_order_model: false`), 꺼져 있으면 관제가 보내는 명령은 전과 똑같습니다(사건 기록 전체를 비교하는 시험 `test_zone_order_off.py`로 확인합니다).

메시지 칸은 [핑키_계약.md](핑키_계약.md) 2절과 3절 11번에 있습니다. 이 문서는 관제가 그 칸을 어떻게 쓰는지를 적습니다.

## 1. 무엇이 달라지나

| | 꺼짐(지금) | 켜짐 |
|---|---|---|
| 관제가 보내는 것 | 구간마다 goal 하나. 구간이 끝난 결과를 받고 다음 goal을 보냅니다 | 목적지까지의 조각을 한 번에 보냅니다. 허락한 조각이 base, 아직 허락하지 않은 조각이 horizon입니다 |
| 구간 사이 | 로봇이 서서 다음 goal을 기다립니다 | 로봇이 base 안에서는 서지 않고 다음 조각으로 넘어갑니다 |
| 다음 구역 허락 | 구간이 끝난 뒤에 잡습니다 | base 끝에 닿기 `lookahead_s`(2.0 s) 전에 미리 잡아 update로 base를 늘립니다 |
| 통신이 끊기면 | 로봇이 goal 끝에서 섭니다 | 기본(`comm_loss_policy: stop`)은 order가 아닐 때처럼 바로 섭니다. `base_end`로 두면 base 끝까지 가서 서고 base 밖으로는 가지 않습니다 |

## 2. 낱말

| 낱말 | 뜻 |
|---|---|
| 조각(segment) | 한 가지 방식으로 가는 한 토막입니다. `follow_path`(선 따라가기), `nav_goal`(Nav2가 길을 짬), `retreat`(뒤로 나오기) 가운데 하나입니다 |
| base | 관제가 구역을 잡아 준 조각들입니다. 로봇은 여기까지만 갑니다 |
| horizon | 그 뒤에 갈 예정인 조각들입니다. 구역은 아직 잡지 않았습니다. 로봇은 참고만 하고 화면에 다르게 그립니다 |
| update | base를 늘리거나 horizon을 바꾸는 새 goal입니다. 번호(`order_update_id`)가 1씩 오릅니다 |
| 이음 지점 | update가 붙는 곳, 곧 지금까지 받은 base의 마지막 지점입니다. 틀리면 로봇이 update를 거절합니다 |

## 3. 관제가 하는 일

| 상황 | 관제가 하는 일 | 사건 코드 |
|---|---|---|
| 이동을 시작합니다 | 다음 안전 지점까지의 구역을 한꺼번에 잡고, 거기까지를 base로, 나머지를 horizon으로 보냅니다 | `ORDER_SENT` |
| base 끝이 `lookahead_s` 안으로 다가옵니다 | 그다음 안전 지점까지의 구역을 한꺼번에 잡을 수 있으면 update로 base를 늘립니다. 새로 허락한 조각만 보냅니다 | `ORDER_UPDATED` |
| 구역을 못 잡았습니다 | 아무것도 보내지 않습니다. 로봇은 base 끝에서 서서 기다리고, 구역이 비면 그때 update를 보냅니다 | `BASE_END_WAIT`, `ZONE_WAIT` |
| 가려는 도크가 차 있습니다 | 기다리는 자리(O_Q, W_Q)까지를 base로, 도크로 들어가는 조각을 horizon으로 보냅니다. 차례가 오고 도크가 비면 update를 보냅니다 | `DOCK_QUEUED` |
| base 뒤의 길이 달라졌습니다 | base 끝이 `lookahead_s` 안으로 다가왔거나 로봇이 base 끝에 서 있을 때 그 뒤의 길을 다시 구합니다. 달라졌으면 horizon만 바꾸는 update를 보냅니다(base는 비움). 5 s에 한 번을 넘지 않습니다 | `HORIZON_CHANGED` |
| base를 줄여야 합니다(목적이 바뀜) | base는 줄이지 않습니다. 로봇이 넓은 구역에 나오면 order를 취소하고 새 order를 보냅니다. 갈림목이나 급행 안에서는 취소하지 않습니다 | `ORDER_SENT`(새 order) |
| 조각 하나가 끝났습니다 | 진행을 기록합니다. 지나온 구역은 조각이 끝날 때가 아니라, 위치 보고가 그 구역을 벗어난 것이 `release_confirm_samples`번 이어질 때 돌려받습니다(order 모델을 꺼도 같습니다) | `SEGMENT_DONE` |
| 로봇이 base 밖의 조각을 끝냈다고 보고했습니다 | 믿지 않습니다. 진행을 바꾸지 않고 기록만 합니다(같은 order에 한 번) | `SEGMENT_BEYOND_BASE` |
| 로봇이 horizon을 미리 보고 경고했습니다 | 기록만 합니다(같은 order에 한 번) | `HORIZON_INVALID` |
| 통신이 끊겼습니다 | base의 구역을 "base 끝 예상 도착 + 3 s"까지 잡아 둡니다. 그때까지는 운영자의 치움 확인도 받지 않습니다 | `COMM_LOST` |

## 4. update가 잘못됐을 때

| 무슨 일 | 관제가 하는 일 |
|---|---|
| 로봇이 update를 거절했습니다(`ORDER_UPDATE_REJECTED`) | 이전 goal은 살아 있으므로 로봇은 이전 base 끝까지 갑니다. 거기 서면 order를 끝내고 새 order를 보냅니다. 실패 횟수로 세지 않습니다 |
| update를 보내지 못했습니다(서버가 준비되지 않음) | 위와 같습니다 |
| update에 2 s 동안 답이 없습니다 | 어느 goal이 살아 있는지 알 수 없으므로 두 goal을 모두 취소하고 새 order를 보냅니다 |
| update goal이 받아들여진 뒤 결과가 사라졌습니다 | 위와 같습니다 |
| 이전 goal이 `SUPERSEDED`로 끝났습니다 | 실패가 아닙니다. update가 붙었다는 뜻입니다 |

## 5. 켜는 법

```bash
ros2 launch iponic_bringup pc.launch.py use_order_model:=true lookahead_s:=2.0
ros2 launch fleet_gz_sim gz.launch.py layout:=d2 use_order_model:=true   # Gazebo
```

관제는 `/fleet/robots`(RobotStatus)의 `base`와 `horizon` 칸에 앞에 남은 길의 지점 이름을 나눠 싣습니다(기준(사용자)). 관제 화면은 이 두 칸으로 허락받은 길과 아직 허락받지 않은 길을 다르게 그립니다. order 모델을 꺼도 두 칸은 채워집니다.

rviz(`pinky_nav_config/rviz/pinky_nav.rviz`)에서는 `/<로봇>/order_markers`로 남은 base가 초록 실선, horizon이 주황 점선으로 보입니다.

## 6. 정한 값(가정)

| 값 | 얼마 | 까닭 |
|---|---|---|
| `lookahead_s` | 2.0 s | 추가 지시서의 값입니다. 0.12 m/s에서 0.24 m 앞입니다 |
| update 답 기다림 | 2.0 s | 넘으면 두 goal을 모두 끝냅니다. 코드 상수 `UPDATE_ANSWER_S`입니다 |
| horizon 바꾸기 간격 | 5.0 s | 길이 자주 바뀔 때 update가 쏟아지지 않게 합니다. 코드 상수 `HORIZON_HOLD_S`입니다 |
| 통신 끊김 여유(`comm_base_hold_extra_s`) | 3.0 s | 추가 지시서의 값입니다 |
| 통신 끊김 규칙(`comm_loss_policy`) | `stop` | 사용자 기준입니다. `base_end`로 바꿀지는 주행 완료 기준을 통과한 뒤 사용자가 정합니다 |

## 7. 알려진 한계

| 한계 | 지금 |
|---|---|
| `comm_loss_policy: base_end`이면 통신이 끊긴 로봇이 base 끝까지 갑니다(기본 `stop`은 바로 섭니다) | base의 구역은 잡혀 있으므로 다른 로봇과 겹치지 않습니다. 다만 그 사이 사람이 들어오는 것은 로봇의 회피에 맡깁니다. 로봇은 끊김을 `/fleet/heartbeat`가 `heartbeat_timeout_s` 동안 오지 않는 것으로 압니다(핑키 계약 3절) |
| 두 goal을 취소한 뒤 새 order가 취소 확인보다 먼저 갈 수 있습니다 | 로봇은 update 번호 0을 새 order로 받아 달리던 이동을 밀어냅니다(계약 3절 11번). 다른 취소 경로와 같습니다 |
| 이은 자리의 위치 오차 | Nav2 goal은 0.05~0.10 m 앞에서 섭니다. 그 뒤 조각이 `follow_path`이면 그만큼 어긋난 채 시작합니다(측정은 작업 기록에 있습니다) |
| base 끝에서 속도를 미리 줄이는 선택지 | 만들지 않았습니다. 로봇은 base 끝에서 지금 방식대로 섭니다 |
