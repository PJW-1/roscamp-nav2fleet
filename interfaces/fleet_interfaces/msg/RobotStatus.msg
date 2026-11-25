# ---- 관제가 보는 로봇 한 대의 상태 ----

# ---- fault에 넣는 값 ----
string FAULT_NONE="NONE"
string FAULT_ERROR="ERROR"
string FAULT_COMM_LOST="COMM_LOST"
string FAULT_OUT_OF_SERVICE="OUT_OF_SERVICE"
string FAULT_RECOVERING="RECOVERING"

# ---- 로봇의 마지막 보고와 관제가 매긴 상태 ----
string robot_id
string fsm_state
string task_type
string order_id
string fault
string fault_reason
string wait_zone
string[] wait_robots
float32 battery
float64 x
float64 y
float64 yaw

# ---- 앞에 남은 길: 허락받은 곳과 아직 아닌 곳 ----
string[] base
string[] horizon

# ---- 마지막으로 보낸 MoveTo의 우선순위 ----
int32 priority -1

# ---- 로봇 종류, 락, 운영 모드, 강제 값, 재시작 감지: 칸 추가 ----
string KIND_POD="POD"
string KIND_ARM="ARM"
string kind
bool lock
string mode
string[] forced
string boot_id
# 위치 · 경로 칸(x · y · yaw · base · horizon)은 Pode만 씀
