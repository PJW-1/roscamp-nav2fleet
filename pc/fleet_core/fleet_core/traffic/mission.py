"""교통 층이 함께 쓰는 상수: 임무 단계, 고장 상태, 로봇이 보고하는 상태, goal 종류.

로봇 하나의 기록은 traffic/zone/robots.py에 있다.
"""
from __future__ import annotations

# 임무 단계
IDLE = "IDLE"
PARKED = "PARKED"
TO_PARK = "TO_PARK"
TO_WORK = "TO_WORK"
TO_OUTBOUND = "TO_OUTBOUND"
DOCKING = "DOCKING"
AT_STATION = "AT_STATION"
# 단계 대신 보이는 고장 상태
ERROR = "ERROR"
OUT_OF_SERVICE = "OUT_OF_SERVICE"
COMM_LOST = "COMM_LOST"
RECOVERING = "RECOVERING"
PLACEMENT_CONFLICT = "PLACEMENT_CONFLICT"
NO_FAULT = "NONE"  # 이상 없을 때 /fleet/robots에 나가는 값
# 로봇 상태 보고에 오는 값
# 로봇이 스스로 짧게 후진하고 다시 간다(계약 4절, 제안). 위 fault RECOVERING과 값만 같다.
# 이동 중으로 보되 막힘 시계와 멈춤 감시는 쉰다(zone/blocked.py, zone/watch.py).
SELF_RECOVERING = "RECOVERING"
ON_A_GOAL = ("MOVING", "DOCKING", SELF_RECOVERING)
BLOCKED = "BLOCKED"  # 경로 앞 물체로 섰다(계약 4절)
LOCALIZING = "LOCALIZING"  # 위치를 아직 못 잡았다. 자세를 믿지 않는다
NOT_PLACEABLE = ON_A_GOAL + (BLOCKED, LOCALIZING)  # 이 상태로는 지도에 올리지 않는다
HEARTBEAT_LOST = "HEARTBEAT_LOST"  # heartbeat가 끊겨 로봇이 스스로 끝낸 이동의 이유
_COMM_REASON = {COMM_LOST: "NO_STATE", RECOVERING: "CHECKING_POSE"}
STALLED = "STALLED"  # 목표가 열린 채 제자리
NOT_AT_TARGET = "NOT_AT_TARGET"  # 이동은 성공이라는데 자세는 아니라고 할 때
PATH_NOT_CONNECTED = "PATH_NOT_CONNECTED"  # 로봇이 받은 고정 경로에서 너무 멀어 따라가지 않았다

RUN = "RUN"
STOP = "STOP"

MOVE = "move"
DOCK = "dock"
# 관제가 시키는 물러나기(RetreatCmd, 제안). 임무 경로(path)를 쓰지 않는다. 판단은 교통 층이 한다
RETREAT = "retreat"

# 맵에 적힌 스테이션 동작
FILL = "fill"
SWAP = "swap"

