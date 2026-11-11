# 제안(팀 합의 전, 계약 0.11): order 모델의 조각 하나. MoveTo의 base와 horizon에 담는다.
# 이동 안에서 1부터 늘어나는 번호. update를 거쳐도 이어진다
uint32 seq_id
# follow_path / nav_goal / retreat
string MODE_FOLLOW_PATH="follow_path"
string MODE_NAV_GOAL="nav_goal"
string MODE_RETREAT="retreat"
string mode
# follow_path와 retreat: FollowPath / FollowPathThrough / FollowPathReverse. nav_goal이면 비운다
string controller_id
# follow_path와 retreat일 때 따를 경로(map 프레임)
nav_msgs/Path path
# nav_goal일 때 갈 곳(map 프레임)
geometry_msgs/PoseStamped target_pose
# 이 조각이 끝나는 지점 이름
string end_node_id
# 지나는 구역(표시와 검사용)
string[] zone_ids
