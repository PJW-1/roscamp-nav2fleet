string FAULT_NONE="NONE"
string FAULT_ERROR="ERROR"
string FAULT_COMM_LOST="COMM_LOST"
string FAULT_OUT_OF_SERVICE="OUT_OF_SERVICE"
string FAULT_RECOVERING="RECOVERING"

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
string[] base
string[] horizon
int32 priority -1
