# ---- 동작 없음 모드(설계 4.5): Nav2 대신 정해진 시간 뒤 성공을 알리고 자세를 끝점으로 옮긴다 ----
# TODO(no-motion): 실주행으로 돌아가면 no_motion:=false로 끄고, 이 파일과 분기를 지운다
from __future__ import annotations

import math
import threading
from dataclasses import dataclass

# ---- 일의 종류: FollowPath, NavigateToPose, BackUp, 도킹 ----
FOLLOW = "follow"
GOAL = "goal"
BACKUP = "backup"
DOCK = "dock"


@dataclass(frozen=True)
class NoMotionParams:
    move_s: float = 2.0
    retreat_s: float = 2.0
    dock_s: float = 3.0
    # 관제가 보내는 도킹 이름(스테이션 이름). 좌표는 모른다
    dock_ids: tuple[str, ...] = ("WORK", "OUTBOUND")


@dataclass
class Job:
    kind: str
    key: object
    due: float
    end: tuple[float, float, float] | None
    canceled: bool = False


# ---- 기다리는 일 목록과 지금 자세 ----
class NoMotion:
    def __init__(self, start, params: NoMotionParams):
        self.p = params
        self.pose = tuple(float(v) for v in start)
        self._jobs: list[Job] = []
        self._lock = threading.Lock()

    def add(self, kind: str, key, now: float, wait: float, end=None) -> None:
        with self._lock:
            self._jobs.append(Job(kind, key, now + wait, end))

    # ---- 같은 끝점으로 다시 보낸 경로는 앞 일의 남은 시간을 잇는다(Nav2 경로 교체처럼) ----
    def replace(self, kind: str, key, now: float, wait: float, end) -> None:
        due = now + wait
        with self._lock:
            for job in self._jobs:
                if job.kind == kind and not job.canceled:
                    if job.end is not None and math.dist(job.end[:2], end[:2]) < 1e-6:
                        due = min(due, job.due)
                    job.canceled, job.due = True, -math.inf
            self._jobs.append(Job(kind, key, due, end))

    # ---- 취소: 다음 step에서 취소 결과를 알린다 ----
    def cancel(self, kind: str) -> None:
        with self._lock:
            for job in self._jobs:
                if job.kind == kind and not job.canceled:
                    job.canceled, job.due = True, -math.inf

    def busy(self, kind: str) -> bool:
        with self._lock:
            return any(j.kind == kind and not j.canceled for j in self._jobs)

    # ---- 때가 된 일을 꺼내고, 끝난 일의 끝점으로 자세를 옮긴다 ----
    def step(self, now: float) -> list[Job]:
        with self._lock:
            due = [j for j in self._jobs if j.due <= now]
            self._jobs = [j for j in self._jobs if j.due > now]
            due.sort(key=lambda j: j.due)
            for job in due:
                if not job.canceled and job.end is not None:
                    self.pose = job.end
        return due

    def backup_end(self, distance_m: float) -> tuple[float, float, float]:
        x, y, yaw = self.pose
        return (x - math.cos(yaw) * distance_m, y - math.sin(yaw) * distance_m, yaw)


# ---- Nav2 실행기와 같은 모양의 대역 셋 ----
class _Runner:
    def __init__(self, nm: NoMotion, kind: str, clock):
        self._nm, self._kind, self._clock = nm, kind, clock

    # 기다릴 서버가 없으니 늘 준비됨(LOCALIZING에 묶이지 않게)
    def ready(self) -> bool:
        return True

    def cancel_all(self) -> None:
        self._nm.cancel(self._kind)

    def destroy(self) -> None:
        pass


class FollowStub(_Runner):
    def __init__(self, nm: NoMotion, clock):
        super().__init__(nm, FOLLOW, clock)

    def send(self, seq: int, poses, final_goal: bool, reverse: bool,
             free_heading: bool = False) -> bool:
        if not poses:
            return False
        wait = self._nm.p.retreat_s if reverse else self._nm.p.move_s
        end = tuple(float(v) for v in poses[-1])
        self._nm.replace(FOLLOW, seq, self._clock(), wait, end)
        return True


class NavGoalStub(_Runner):
    def __init__(self, nm: NoMotion, clock):
        super().__init__(nm, GOAL, clock)

    def send(self, seq: int, pose) -> bool:
        end = tuple(float(v) for v in pose)
        self._nm.replace(GOAL, seq, self._clock(), self._nm.p.move_s, end)
        return True


class BackUpStub(_Runner):
    def __init__(self, nm: NoMotion, clock):
        super().__init__(nm, BACKUP, clock)

    def send(self, seq: int, distance_m: float, speed_mps: float,
             time_allowance_s: float) -> bool:
        end = self._nm.backup_end(float(distance_m))
        self._nm.replace(BACKUP, seq, self._clock(), self._nm.p.retreat_s, end)
        return True


# ---- 도킹 진행 단계(PC 도킹 대역 real_dock_wait와 같은 나눔) ----
def dock_phase(elapsed: float, duration: float) -> str:
    done = elapsed / max(duration, 1e-6)
    return "APPROACH" if done < 0.5 else "ALIGN" if done < 1.0 else "DONE"


# ---- 도킹 이름 확인: 거리는 보지 않고, 모르는 이름이면 경고 문구만 돌려준다 ----
def dock_warning(dock_id: str, known) -> str | None:
    if dock_id in known:
        return None
    return (f"no_motion: unexpected dock_id {dock_id!r} (expected one of {list(known)}); "
            "succeeding anyway")
