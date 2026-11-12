# FMS가 보는 로봇 한 대의 상태. 모니터링 UI용(SR-17).
string FAULT_NONE="NONE"
string FAULT_ERROR="ERROR"
string FAULT_COMM_LOST="COMM_LOST"
string FAULT_OUT_OF_SERVICE="OUT_OF_SERVICE"
string FAULT_RECOVERING="RECOVERING"

# fsm_state, battery, x, y, yaw는 마지막 보고값이며 아무리 오래됐어도 그대로다.
# 로봇이 조용해졌는지는 fault로만 알 수 있고, 다른 fault가 없을 때만 그렇다.
string robot_id
string fsm_state        # 핑키가 보고한 값 그대로. 첫 보고 전에는 빈 문자열
string task_type        # Assignment.TYPE_*. 현장에서 빠졌으면 빈 문자열
string order_id
string fault            # FAULT_*
string fault_reason     # 이유(한 단어 또는 운영자 메모). fault가 NONE이면 빈 문자열
string wait_zone        # 대기 중인 구역. 대기 중이 아니면 빈 문자열
string[] wait_robots    # 먼저 비켜 줘야 하는 로봇
float32 battery
float64 x
float64 y
float64 yaw
# 기준(사용자): 앞에 남은 길의 지점 이름. base는 구역을 허락받은 부분, horizon은 아직
# 허락받지 않은 부분이다. 화면이 둘을 다르게 그리는 데 쓴다. 갈 길이 없으면 둘 다 빈다
string[] base
string[] horizon
# 기준(사용자, 계약 0.13-draft, 추가 지시서 3의 7-4): 관제가 이 로봇에 마지막으로 보낸 MoveTo의
# priority. 로봇은 앞을 막은 다른 로봇의 값을 여기서 읽는다. -1이면 비었다
int32 priority -1
