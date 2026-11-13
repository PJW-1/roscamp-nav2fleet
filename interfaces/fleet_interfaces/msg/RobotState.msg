# ---- 핑키가 관제에 보내는 자기 상태 ----

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
