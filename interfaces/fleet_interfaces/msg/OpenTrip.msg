# ---- 끝나지 않은 운반 건 하나 (ListOpenTrips 답) ----
string trip_id
string order_id
uint32 trip_index            # 몇 번째 운반 건 (1부터)
uint32 trip_count            # 주문의 전체 운반 건 수
OrderItem[] items
string status                # ReportTrip.STATUS_*
string reason
string robot_id              # 맡은 Pode, 없으면 ""
OrderItem[] placed
OrderItem[] missing
builtin_interfaces/Time submitted_at
uint32 report_epoch          # 마지막으로 반영한 보고 번호
uint64 report_n
