"""복구 중 AMCL 자세 오차를 잰다. recovery_probe 노드의 ROS 없는 논리.
RECOVERING 구간마다 start, end(길이와 최대 오차), settled(끝난 뒤 settle_s 이후 첫 보고) 행을 CSV로 쓴다.
각 행은 보고의 AMCL 자세를 stamp에 가장 가까운 Gazebo 실제 위치와 비교한다.
"""
from __future__ import annotations

import bisect
import csv
import math
import os
from collections import deque

RECOVERING = "RECOVERING"
NO_POSE = "LOCALIZING"  # 이때 RobotState.pose는 (0, 0, 0)이라 AMCL 자세가 아니다
MATCH_S = 0.1
KEEP_S = 5.0  # 로봇마다 남기는 실제 위치 기록 길이
COLUMNS = ["sim_time", "robot", "event", "episode", "state", "truth_x", "truth_y", "truth_yaw",
           "amcl_x", "amcl_y", "amcl_yaw", "err_m", "yaw_err_deg", "report_age_s", "next_state",
           "duration_s", "max_err_m", "note"]


def wrap_deg(deg: float) -> float:
    """[-180, 180]으로 감는다."""
    wrapped = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 and deg > 0 else wrapped


class _Episode:
    def __init__(self, number: int):
        self.number = number
        self.start = 0.0
        self.max_err = None
        self.settle_due = None  # 구간이 끝나면 정해진다


class RecoveryProbe:
    def __init__(self, robots, settle_s: float = 2.0):
        self.settle_s = settle_s
        self.truth = {r: deque() for r in robots}  # (stamp s, x, y, yaw)
        self.episodes = {r: _Episode(0) for r in robots}
        self.active = {r: False for r in robots}

    def on_truth(self, robot, stamp, x, y, yaw) -> None:
        buf = self.truth[robot]
        buf.append((stamp, x, y, yaw))
        while buf[0][0] < stamp - KEEP_S:
            buf.popleft()

    def _nearest(self, robot, stamp):
        buf = self.truth[robot]
        i = bisect.bisect_left([s[0] for s in buf], stamp)
        near = [buf[j] for j in (i - 1, i) if 0 <= j < len(buf)]
        if not near:
            return None
        best = min(near, key=lambda s: abs(s[0] - stamp))
        return best if abs(best[0] - stamp) <= MATCH_S else None

    def _row(self, now, robot, event, state, pose, stamp, **extra):
        ep = self.episodes[robot]
        row = dict.fromkeys(COLUMNS, "")
        row.update(sim_time=now, robot=robot, event=event, episode=ep.number, state=state,
                   **extra)
        if state == NO_POSE:
            row["note"] = "no_pose"
            return row, None
        row.update(amcl_x=pose[0], amcl_y=pose[1], amcl_yaw=pose[2])
        truth = self._nearest(robot, stamp)
        if truth is None:
            row["note"] = "no_truth"
            return row, None
        err = math.hypot(pose[0] - truth[1], pose[1] - truth[2])
        row.update(truth_x=truth[1], truth_y=truth[2], truth_yaw=truth[3], err_m=err,
                   yaw_err_deg=wrap_deg(math.degrees(pose[2] - truth[3])),
                   report_age_s=truth[0] - stamp)
        return row, err

    def on_state(self, robot, now, state, x, y, yaw, stamp) -> list[dict]:
        """/state 보고 하나를 받아 그로 인한 CSV 행을 돌려준다(보통 없음)."""
        pose, ep, rows = (x, y, yaw), self.episodes[robot], []
        if state == RECOVERING:
            if not self.active[robot]:
                self.episodes[robot] = ep = _Episode(ep.number + 1)
                self.active[robot] = True
                ep.start = now
                row, err = self._row(now, robot, "start", state, pose, stamp)
                rows.append(row)
            else:
                _, err = self._row(now, robot, "", state, pose, stamp)
            if err is not None:
                ep.max_err = err if ep.max_err is None else max(ep.max_err, err)
        elif self.active[robot]:
            self.active[robot] = False
            _, err = self._row(now, robot, "", state, pose, stamp)
            if err is not None:
                ep.max_err = err if ep.max_err is None else max(ep.max_err, err)
            ep.settle_due = now + self.settle_s
            row, _ = self._row(now, robot, "end", state, pose, stamp, next_state=state,
                               duration_s=now - ep.start,
                               max_err_m="" if ep.max_err is None else ep.max_err)
            rows.append(row)
        elif ep.settle_due is not None and now >= ep.settle_due:
            ep.settle_due = None
            row, _ = self._row(now, robot, "settled", state, pose, stamp)
            rows.append(row)
        return rows


class CsvSink:
    """CSV에 행을 덧붙인다. 파일이 새것이거나 비었을 때만 머리 줄을 쓰고 행마다 flush한다. 빈 경로는 쓰지 않는다."""

    def __init__(self, path: str):
        self.file = self.writer = None
        if not path:
            return
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        fresh = not os.path.exists(path) or os.path.getsize(path) == 0
        self.file = open(path, "a", newline="", encoding="utf-8")  # noqa: SIM115
        self.writer = csv.DictWriter(self.file, COLUMNS, extrasaction="ignore")
        if fresh:
            self.writer.writeheader()
            self.file.flush()

    def write(self, row: dict) -> None:
        if self.writer:
            self.writer.writerow(row)
            self.file.flush()

    def close(self) -> None:
        if self.file:
            self.file.close()
