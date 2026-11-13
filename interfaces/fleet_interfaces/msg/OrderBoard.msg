# ---- 중앙 서버의 주문 목록과 운영 상태 ----

# ---- run_mode에 넣는 값 ----
string MODE_RUN="RUN"
string MODE_PAUSE="PAUSE"
string MODE_STOP="STOP"

# ---- 운영 상태와 끝나지 않은 주문들 ----
string run_mode
Order[] orders
builtin_interfaces/Time stamp
