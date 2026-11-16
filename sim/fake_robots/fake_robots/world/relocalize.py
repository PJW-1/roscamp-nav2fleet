"""가짜 Pode의 위치 다시 찾기(Relocalize): 선 로봇이 RELOCALIZE_S 뒤 답한다.

그동안 LOCALIZING을 보고한다. 성공하면 그때까지 시작한 drive_by_estimate 창의 믿는 치우침이 끝난다
(믿는 자리 = 실제 자리). 보고만 미는 NoisyPose 창은 그대로다. RelocalizeFails 창이면 실패로 답한다.
움직이는 로봇은 받자마자 BUSY로 끝낸다(일과 상태는 그대로). 답의 자세는 보고와 같은 추정이다.
"""
from __future__ import annotations

RELOCALIZE_S = 2.0
BUSY, CANCELED = "BUSY", "CANCELED"


class Relocalizer:
    """ZoneWorld에 섞는다."""

    def _reloc_answers(self) -> list:
        if not hasattr(self, "_reloc_out"):
            self._reloc_out = []  # 다음 걸음에 낼 답 (로봇, 인자들)
        return self._reloc_out

    def _estimate(self, r) -> tuple:
        """로봇이 믿는(보고하는) 자세. 잡음은 넣지 않는다."""
        return (r.x + self.schedule.noise(r.name, self.t)[1], r.y, r.yaw)

    def _relocalize_cmd(self, cmd) -> None:
        if not self._reachable(cmd.robot):
            return  # 닿지 않은 goal: 관제는 기한으로 안다
        r = self.robots[cmd.robot]
        if r.run is not None or r.reloc is not None:
            self._reloc_answers().append((r.name, (cmd.goal_id, False, BUSY, None, None)))
            return
        r.reloc = (cmd.goal_id, round(self.t + RELOCALIZE_S, 6), self._estimate(r))
        r.localizing_until = r.reloc[1]

    def _cancel_relocalize(self, cmd) -> bool:
        r = self.robots[cmd.robot]
        if r.reloc is None or r.reloc[0] != cmd.goal_id or not self._reachable(cmd.robot):
            return False
        goal_id, _, start = r.reloc
        r.reloc, r.localizing_until = None, self.t
        pose = self._estimate(r)
        self._reloc_answers().append((r.name, (goal_id, False, CANCELED, pose, start)))
        return True

    def _run_relocalize(self) -> None:
        out, self._reloc_out = self._reloc_answers(), []
        for name, args in out:
            self._answer_relocalize(name, args)
        for name, r in self.robots.items():
            if r.reloc is None or self.t < r.reloc[1] - 1e-9:
                continue
            goal_id, _, start = r.reloc
            r.reloc = None
            reason = self.schedule.relocalize_fails(name, self.t)
            if not reason:
                self.schedule.relocalize(name, self.t)
                self._last_dx[name] = self.schedule.belief_dx(name, self.t)
            self._answer_relocalize(name, (goal_id, not reason, reason, self._estimate(r), start))

    def _answer_relocalize(self, name, args) -> None:
        if self._reachable(name):
            self.manager.on_relocalize_result(name, *args)
