# ---- 팔이 관제에 보내는 자기 상태 (/{arm}/state) ----

# ---- state에 넣는 값 ----
string STATE_IDLE="IDLE"
string STATE_PICK_WAITING="PICK_WAITING"
string STATE_PICKING="PICKING"
string STATE_PLACE_WAITING="PLACE_WAITING"
# 바구니 공간 안. arm_left와 arm_right가 동시에 이 값을 보고하면 안 된다.
string STATE_PLACING="PLACING"
string STATE_RETRYING="RETRYING"
string STATE_PAUSED="PAUSED"
string STATE_SWAPPING="SWAPPING"
string STATE_TAKE_WAITING="TAKE_WAITING"     # Phony
string STATE_TAKING_FULL="TAKING_FULL"       # Phony
string STATE_PUTTING_EMPTY="PUTTING_EMPTY"   # Phony
string STATE_ERROR="ERROR"

# ---- gripper에 넣는 값 ----
string GRIPPER_OPEN="OPEN"
string GRIPPER_CLOSED="CLOSED"

# ---- 이름, 재시작 감지, 상태, 락, 하던 일 ----
string arm_id
string boot_id
string state
string detail                # 2.2 사유 코드
bool lock                    # 잠김 = true. Pode · Phody 팔 · Phony 모두 (명세 4.5)
string last_request_id       # 마지막으로 반영한 명령 (관측 보조 · 디버깅용, 결과 전달 경로 아님, #25 ④)
string gripper
bool holding                 # 상품을 쥐고 있는지
string order_id              # 하던 운반 건 (trip_id)
int32 placed_total
string mode
string[] forced
builtin_interfaces/Time stamp
