# ---- 관리 명령 상태: 락, 운영 모드, 강제 값, 요청 번호 기억 ----
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass

# ---- 켜질 때 값, 모드, 거절 사유, 기억 한도 ----
BOOT_MODE = "UNKNOWN"
MODES = ("NORMAL", "TEST")
TEST_MODE = "TEST"
NOT_STOPPED = "NOT_STOPPED"
NOT_TEST_MODE = "NOT_TEST_MODE"
MODE_MISMATCH = "MODE_MISMATCH"
BATTERY_LOW = "BATTERY_LOW"
UNKNOWN_TARGET = "UNKNOWN_TARGET"
REMEMBER_COUNT = 32
REMEMBER_S = 60.0
CHARGE_END_MIN_PCT = 35.0


@dataclass(frozen=True)
class Reply:
    accepted: bool
    reason: str = ""
    detail: str = ""


# ---- 로봇 노드가 마지막으로 판단하는 관리 명령 ----
class AdminState:
    def __init__(self, robot_id: str, boot_id: str):
        self.robot_id = robot_id
        self.boot_id = boot_id
        self.lock = True
        self.mode = BOOT_MODE
        self.forced: list[str] = []
        self.last_request_id = ""
        self._seen: OrderedDict[str, tuple[float, Reply]] = OrderedDict()

    def set_lock(self, request_id: str, robot_id: str, locked: bool, stopped: bool,
                 now: float) -> Reply:
        def judge() -> Reply:
            if robot_id != self.robot_id:
                return Reply(False, UNKNOWN_TARGET, f"this is {self.robot_id}")
            if bool(locked) == self.lock:
                return Reply(True)
            if not stopped:
                return Reply(False, NOT_STOPPED, "the robot has a goal")
            self.lock = bool(locked)
            return Reply(True)
        return self._once(request_id, now, judge)

    def set_mode(self, request_id: str, mode: str, now: float) -> Reply:
        def judge() -> Reply:
            if mode not in MODES:
                return Reply(False, MODE_MISMATCH, f"unknown mode {mode!r}")
            self.mode = mode
            return Reply(True)
        return self._once(request_id, now, judge)

    def charge_end(self, request_id: str, robot_id: str, battery_pct: float,
                   now: float) -> Reply:
        def judge() -> Reply:
            if robot_id != self.robot_id:
                return Reply(False, UNKNOWN_TARGET, f"this is {self.robot_id}")
            if self.mode != TEST_MODE:
                return Reply(False, NOT_TEST_MODE, f"mode is {self.mode}")
            if battery_pct < CHARGE_END_MIN_PCT:
                return Reply(False, BATTERY_LOW, f"battery {battery_pct:.0f}%")
            return Reply(True)
        return self._once(request_id, now, judge)

    # ---- IR-20 · IR-21: 같은 request_id는 한 번만 반영하고 같은 답 ----
    def _once(self, request_id: str, now: float, judge) -> Reply:
        self._forget(now)
        if request_id and request_id in self._seen:
            return self._seen[request_id][1]
        reply = judge()
        if request_id:
            self._seen[request_id] = (now, reply)
            while len(self._seen) > REMEMBER_COUNT:
                self._seen.popitem(last=False)
            if reply.accepted:
                self.last_request_id = request_id
        return reply

    def _forget(self, now: float) -> None:
        while self._seen and now - next(iter(self._seen.values()))[0] > REMEMBER_S:
            self._seen.popitem(last=False)
