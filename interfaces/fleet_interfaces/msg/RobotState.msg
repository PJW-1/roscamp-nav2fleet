string STATE_IDLE="IDLE"
string STATE_MOVING="MOVING"
string STATE_WAITING="WAITING"
string STATE_DOCKING="DOCKING"
string STATE_DOCKED="DOCKED"
string STATE_ERROR="ERROR"
string STATE_BLOCKED="BLOCKED"
string STATE_LOCALIZING="LOCALIZING"
string STATE_RECOVERING="RECOVERING"

string robot_id
geometry_msgs/Pose2D pose
float32 battery
string state
string current_goal
builtin_interfaces/Time stamp

# 기준(사용자, 계약 0.12-draft, 추가 지시서 3의 4-1): BLOCKED일 때 막은 것.
# obstacle_x, obstacle_y는 map 좌표다. 로봇이 local costmap에서 가던 길 앞 obstacle_look_m(0.30 m)
# 안의 가장 가까운 장애물 칸으로 채운다. 모르거나 BLOCKED가 아니면 NaN이고 obstacle_kind는 빈 값이다.
# obstacle_kind가 비면 관제는 위치를 쓰지 않는다(이 칸이 없던 로봇은 0, 0, 빈 값을 보낸다).
string OBSTACLE_UNKNOWN="unknown"
string OBSTACLE_PERSON="person"
string OBSTACLE_OBJECT="object"
string OBSTACLE_ROBOT="robot"
float64 obstacle_x
float64 obstacle_y
string obstacle_kind    # OBSTACLE_*. 지금은 unknown만 쓴다. 비전이 들어오면 채운다
