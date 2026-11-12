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

# 기준(사용자, 계약 0.13-draft, 추가 지시서 3의 7-4): 로봇단 교착 처리의 단계. state가 RECOVERING일
# 때만 채운다. wait: 물체 앞에 서서 local_wait_s 기다린다. backup: 앞을 막은 다른 로봇보다 낮아
# 지나온 길 안에서 물러난다. retry: 물러난 뒤 다시 간다. 비면 로봇단 처리가 아니다(RECOVERING이면
# 0.12까지의 자가 복구). 관제는 사건 LOCAL_YIELD로 남기기만 하고, 움직임은 BLOCKED 뒤에 정한다
string LOCAL_YIELD_WAIT="wait"
string LOCAL_YIELD_BACKUP="backup"
string LOCAL_YIELD_RETRY="retry"
string local_yield
