# ---- 중앙 서버의 주문 목록과 운영 상태 ----

# ---- run_mode에 넣는 값 ----
string MODE_RUN="RUN"
string MODE_PAUSE="PAUSE"
string MODE_STOP="STOP"

# ---- 운영 상태와 끝나지 않은 주문들 ----
string run_mode
Order[] orders
builtin_interfaces/Time stamp

# ---- 마감 보류: 칸 추가 ----
bool assign_hold             # 마감 중 배정 보류 (SetAssignHold로 받은 값)
string close_id
# orders[]의 순서 = 처음 접수 시각(Order.created_at = trip.submitted_at) 순. 되돌린 운반 건도 자리를 바꾸지 않음 (#21)
