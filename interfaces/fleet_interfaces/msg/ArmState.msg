# ---- 팔이 관제에 보내는 자기 상태 (/{arm}/state) ----

# ---- state에 넣는 값 ----
string STATE_IDLE="IDLE"
string STATE_PICK_WAITING="PICK_WAITING"
string STATE_PICKING="PICKING"
string STATE_PLACE_WAITING="PLACE_WAITING"
# 바구니 공간 안. arm_left와 arm_right가 동시에 이 값을 보고하면 안 된다.
string STATE_PLACING="PLACING"
string STATE_RETRYING="RETRYING"
# 스스로 멈춤(계획 · 실행 · 미세조정 실패) · lock 해제(쥔 채여도) · 거리 정지 한도 초과. 담기 목표는 살아 있다.
# [복구] = /fleet/resume_arm → 두 팔 set_pause(false). 팔은 see_center 복귀 → 재캡처 → 남은 상품 담기까지 한다(따로 만든 인터페이스 없음)
string STATE_PAUSED="PAUSED"
string STATE_SWAPPING="SWAPPING"
string STATE_TAKE_WAITING="TAKE_WAITING"     # Phony
string STATE_TAKING_FULL="TAKING_FULL"       # Phony
string STATE_PUTTING_EMPTY="PUTTING_EMPTY"   # Phony
# 복구할 수 없는 고장만. 관제는 받으면 그 담기를 포기하고 HELD
string STATE_ERROR="ERROR"

# ---- gripper에 넣는 값 ----
string GRIPPER_OPEN="OPEN"
string GRIPPER_CLOSED="CLOSED"

# ---- 이름, 재시작 감지, 상태, 락, 하던 일 ----
string arm_id
string boot_id               # 바뀌면 팔 재시작([초기화]): 하던 담기 목표는 사라지고 관제가 [복구] 때 남은 상품으로 다시 보냄
string state
string detail                # 2.2 사유 코드. 관제 heartbeat가 30 s 끊기면 HEARTBEAT_LOST, 작업은 계속
bool lock                    # 잠김 = true. Pode · Phody 팔 · Phony 모두 (명세 4.5)
string last_request_id       # 마지막으로 반영한 명령 (관측 보조 · 디버깅용, 결과 전달 경로 아님, #25 ④)
string gripper
bool holding                 # 상품을 쥐고 있는지. lock=false를 보낸 상태나 바로 앞 상태에서 true여야 관제가 쥔 채 락 해제로 본다
string order_id              # 하던 운반 건 (trip_id)
int32 placed_total
string mode
string[] forced
builtin_interfaces/Time stamp
