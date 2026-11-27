# ---- 관리 명령 판단: 같은 번호는 한 번, fleet 판단만 곧바로 답, 결과는 ReportCommand (W-30 ~ W-34) ----
from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass

from fleet_manager.robot_calls import ACCEPTED, REJECTED

CACHE_SIZE = 256   # IR-20: 기억하는 request_id 수
CACHE_S = 600.0    # 그 기억을 지니는 시간
TEST_MODE = "TEST"  # SystemMode.MODE_TEST


class Codes:
    """계약 reason_codes에 있는 이름만 꺼낸다 (IR-05). 없는 이름은 바로 오류다."""

    def __init__(self, reason_codes: dict):
        self._all = {c for group in reason_codes.values() for c in group}

    def __getattr__(self, name: str) -> str:
        if name.startswith("_") or name not in self._all:
            raise AttributeError(f"{name} is not a contract reason code")
        return name


@dataclass(frozen=True)
class Reply:
    accepted: bool
    reason: str = ""
    detail: str = ""
    extra: tuple = ()  # 명령마다 더 싣는 값 (confirm_clear의 푼 구역)


@dataclass(frozen=True)
class Facts:
    """set_lock 판단에 쓰는 로봇 상태 (노드가 채운다)."""
    stopped: bool
    holding: bool = False
    why: str = ""  # 멈춤이 아닌 까닭


@dataclass(frozen=True)
class Targets:
    set_lock: frozenset
    fault_resolved: frozenset
    charge_end: frozenset
    force_input: frozenset


# ---- IR-20: 같은 request_id는 한 번만 반영하고 같은 답 ----
class RequestCache:
    def __init__(self, size: int = CACHE_SIZE, keep_s: float = CACHE_S):
        self._size, self._keep_s = size, keep_s
        self._items: OrderedDict[str, tuple[float, Reply]] = OrderedDict()

    def get(self, request_id: str, now: float) -> Reply | None:
        while self._items and now - next(iter(self._items.values()))[0] > self._keep_s:
            self._items.popitem(last=False)
        hit = self._items.get(request_id)
        return hit[1] if hit else None

    def put(self, request_id: str, reply: Reply, now: float) -> None:
        self._items[request_id] = (now, reply)
        while len(self._items) > self._size:
            self._items.popitem(last=False)


# ---- fleet 자기 판단 (IR-60) ----
def judge_set_lock(codes, targets: Targets, robot, locked, confirm_drop,
                   facts: Callable[[str], Facts]) -> Reply:
    if robot not in targets.set_lock:
        return Reply(False, codes.UNKNOWN_TARGET, f"no lock on {robot!r}")
    f = facts(robot)
    if not f.stopped:
        return Reply(False, codes.NOT_STOPPED, f.why)
    # 락을 풀면 팔은 안전한 자세로 가고 그리퍼를 풀어 쥔 상품이 떨어질 수 있다 (모터 힘은 유지)
    if not locked and f.holding and not confirm_drop:
        return Reply(False, codes.HOLDING_ITEM, f"{robot} holds an item, confirm_drop first")
    return Reply(True)


def judge_target(codes, allowed, robot) -> Reply:
    if robot not in allowed:
        return Reply(False, codes.UNKNOWN_TARGET, f"{robot!r} takes no such command")
    return Reply(True)


def judge_test_only(codes, allowed, robot, system_mode) -> Reply:
    # W-53 · IR-63: 시험 모드에서만
    reply = judge_target(codes, allowed, robot)
    if reply.accepted and system_mode != TEST_MODE:
        return Reply(False, codes.NOT_TEST_MODE, f"system mode is {system_mode or 'not known'}")
    return reply


# ---- 명령 처리 틀: 같은 번호 확인, 판단, ReportCommand ----
class AdminCommands:
    def __init__(self, codes: Codes, report_command: Callable, clock: Callable,
                 cache_size: int = CACHE_SIZE, cache_s: float = CACHE_S):
        self.codes = codes
        self._report = report_command
        self._clock = clock
        self._cache = RequestCache(cache_size, cache_s)

    def local(self, command, request_id, target, apply: Callable[[], Reply]) -> Reply:
        """로봇에 보내지 않는 명령: 처리 직후 ReportCommand 한 번 (W-32)."""
        def run():
            reply = apply()
            self._report(request_id, command, target, ACCEPTED if reply.accepted else REJECTED,
                         reply.reason, reply.detail)
            return reply
        return self._once(request_id, run)

    def robot(self, command, request_id, target, judge: Callable[[], Reply],
              send: Callable[[], None]) -> Reply:
        """로봇까지 가는 명령: 받으면 send()가 부르기를 맡고, 결과는 RobotCalls가 알린다 (W-31)."""
        def run():
            reply = judge()
            if reply.accepted:
                send()
            else:
                self._report(request_id, command, target, REJECTED, reply.reason, reply.detail)
            return reply
        return self._once(request_id, run)

    def _once(self, request_id, run) -> Reply:
        now = self._clock()
        if request_id:
            cached = self._cache.get(request_id, now)
            if cached is not None:
                return cached
        reply = run()
        if request_id:
            self._cache.put(request_id, reply, now)
        return reply
