# ---- Pode가 관제에 보내는 자기 상태 ----

# ---- state에 넣는 값 ----
string STATE_IDLE="IDLE"
string STATE_MOVING="MOVING"
string STATE_WAITING="WAITING"
string STATE_DOCKING="DOCKING"
string STATE_DOCKED="DOCKED"
string STATE_ERROR="ERROR"
string STATE_BLOCKED="BLOCKED"
string STATE_LOCALIZING="LOCALIZING"
string STATE_RECOVERING="RECOVERING"

# ---- 이름, 위치, 상태, 실행 중인 목표 ----
string robot_id
geometry_msgs/Pose2D pose
float32 battery
string state
string current_goal
builtin_interfaces/Time stamp

# ---- BLOCKED일 때 막은 것의 종류와 위치 ----
string OBSTACLE_UNKNOWN="unknown"
string OBSTACLE_PERSON="person"
string OBSTACLE_OBJECT="object"
string OBSTACLE_ROBOT="robot"
float64 obstacle_x
float64 obstacle_y
string obstacle_kind

# ---- RECOVERING일 때 로봇단 교착 처리 단계 ----
string LOCAL_YIELD_WAIT="wait"
string LOCAL_YIELD_BACKUP="backup"
string LOCAL_YIELD_RETRY="retry"
string local_yield

# ---- 재시작 감지, 락, 운영 모드, 강제 값, 사유 ----
string boot_id               # 로봇 재시작 감지
bool lock                    # 잠김 = true, 켜질 때 true
string last_request_id       # 마지막으로 반영한 명령 (관측 보조 · 디버깅용, 결과 전달 경로 아님, #25 ④)
string mode                  # SystemMode.MODE_* 또는 "UNKNOWN"(켜질 때)
string[] forced              # 강제 중인 항목
string detail                # 2.2 사유 코드 (ERROR 원인 등)
