# ---- order 모델의 이동 조각 하나 ----

uint32 seq_id

# ---- 가는 방식 ----
string MODE_FOLLOW_PATH="follow_path"
string MODE_NAV_GOAL="nav_goal"
string MODE_RETREAT="retreat"
string mode

# ---- 따라갈 경로 또는 갈 곳 ----
string controller_id
nav_msgs/Path path
geometry_msgs/PoseStamped target_pose

# ---- 끝나는 지점과 지나는 구역 ----
string end_node_id
string[] zone_ids
